/*
 * FreeRTOS V202112.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 */

/******************************************************************************
 * FreeRTOS 任务创建入口 — 系统数据流总览
 *
 * SYSCFG_DL_init() (main.c)
 *   └─ 初始化原理图 GPIO / TIMA0-PWM / I2C1 / UART0~3
 *
 * main_blinky() 创建 3 个任务:
 *
 *   ChassisBoardTask (100Hz, 最高优先级) ── 核心控制回路
 *   │  [传感器输入]
 *   │   ├─ GPIO中断 → g_encoderA_cnt / g_encoderB_cnt (bsp_encoder.c)
 *   │   │     → chassis_feedback_update() → motor[].speed (RPM)
 *   │   └─ UART3 RX中断 → JY901S RX buffer (bsp_uart.c)
 *   │         → bsp_jy901s_process() → chassis_move.imu
 *   │  [控制计算]
 *   │   └─ chassis_control_loop() → PID_Calculate() → motor[].pwm_out
 *   │  [执行输出]
 *   │   └─ chassis_send_cmd() → bsp_motor_set_pwm()
 *   │         └─ TIMA0 CCP3/CCP2 占空比 + GPIO 方向
 *   │
 *   OledDisplayTask (5Hz) ── 显示刷新
 *   │   └─ chassis_move 全局实例 → oled_printf() → I2C1 → SSD1306
 *   │
 *   defaultTask (最低优先级) ── 空闲填充
 *
 * 关键数据流向:
 *   编码器 ──ISR──→ g_encoderA/B_cnt ──Task──→ motor[].speed ──PID──→ motor[].pwm_out
 *   IMU    ──UART3 ISR──→ RX buffer ──Task──→ chassis_move.imu ──显示/控制
 *   摄像头 ──UART0 RX(PA11)──→ bsp_camera_process() ──→ chassis_move.camera
 *****************************************************************************/

/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* TI includes */
#include "ti_msp_dl_config.h"

/* App includes */
#include "app_chassis.h"
#include "app_oled_task.h"
#include "app_watchdog.h"
#include "app_ZDT_task.h"
#include "bsp_track.h"

/*-----------------------------------------------------------*/

/* 任务优先级 */
#define DEFAULT_TASK_PRIORITY            (tskIDLE_PRIORITY + 1)
#define OLED_TASK_PRIORITY               (tskIDLE_PRIORITY + 2)
#define ZDT_TASK_PRIORITY                (tskIDLE_PRIORITY + 2)
#define CHASSIS_TASK_PRIORITY            (tskIDLE_PRIORITY + 3)

/* 任务栈大小 */
#define DEFAULT_TASK_STACK_SIZE          configMINIMAL_STACK_SIZE
#define OLED_TASK_STACK_SIZE             (configMINIMAL_STACK_SIZE * 2)
#define ZDT_TASK_STACK_SIZE              (configMINIMAL_STACK_SIZE * 2)
#define CHASSIS_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE * 4)

/* Keep PA31/C1 electrically disconnected while the tracking board powers up. */
#define TRACK_C1_STARTUP_DELAY_MS         (3000U)
#define WATCHDOG_MONITOR_PERIOD_MS         (500U)

/* Task creation diagnostics. pdPASS means allocation succeeded. */
volatile BaseType_t g_chassis_task_create_result = pdFAIL;
volatile BaseType_t g_zdt_task_create_result = pdFAIL;
volatile BaseType_t g_oled_task_create_result = pdFAIL;
volatile BaseType_t g_default_task_create_result = pdFAIL;
volatile size_t g_free_heap_after_task_create = 0U;

/*-----------------------------------------------------------*/

static void DefaultTask(void *pvParameters);

extern void main_blinky(void);

/*-----------------------------------------------------------*/

void main_blinky(void)
{
    /*
     * C1 startup ownership is independent from ChassisBoardTask.  This keeps
     * the PA31 hardware workaround working even if a control task ever fails
     * to start, and prevents UART/IMU initialization from delaying it.
     */
    bsp_track_init();
    app_watchdog_init(&chassis_move.watchdog);

    /* 创建 defaultTask */
    g_default_task_create_result = xTaskCreate(DefaultTask,
                "defaultTask",
                DEFAULT_TASK_STACK_SIZE,
                &chassis_move,
                DEFAULT_TASK_PRIORITY,
                NULL);

    /* 创建 OLED 显示刷新任务 */
    g_oled_task_create_result = xTaskCreate(oled_display_task,
                "OledDisplayTask",
                OLED_TASK_STACK_SIZE,
                &chassis_move,
                OLED_TASK_PRIORITY,
                NULL);

    /* 创建底盘控制任务 */
    g_zdt_task_create_result = xTaskCreate(zdt_motor_task,
                "ZdtMotorTask",
                ZDT_TASK_STACK_SIZE,
                &chassis_move,
                ZDT_TASK_PRIORITY,
                NULL);

    g_chassis_task_create_result = xTaskCreate(chassis_task,
                "ChassisBoardTask",
                CHASSIS_TASK_STACK_SIZE,
                &chassis_move,
                CHASSIS_TASK_PRIORITY,
                NULL);
    g_free_heap_after_task_create = xPortGetFreeHeapSize();

    /* 启动调度器 */
    vTaskStartScheduler();

    /* 正常情况下不会执行到这里 */
    for (;;)
        ;
}
/*-----------------------------------------------------------*/

static void DefaultTask(void *pvParameters)
{
    chassis_move_t *chassis = (chassis_move_t *)pvParameters;

    if (chassis == NULL) {
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(TRACK_C1_STARTUP_DELAY_MS));
    bsp_track_enable_c1_pullup();

    for (;;) {
        app_watchdog_monitor(&chassis->watchdog,
                             chassis->zdt.task_heartbeat);
        vTaskDelay(pdMS_TO_TICKS(WATCHDOG_MONITOR_PERIOD_MS));
    }
}
/*-----------------------------------------------------------*/
