#include "app_chassis.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include <FreeRTOS.h>
#include <task.h>

/* 全局唯一的底盘实例 (外部不可直接访问, 仅通过指针传递) */
static chassis_move_t    chassis_move    = {0};

/* Keil Watch 调试: 直接输入 chassis_debug 即可展开结构体 */
chassis_move_t *const chassis_debug = &chassis_move;

/* ============================================================
 *  内部: 停车 (归零全向移动目标速度)
 * ============================================================ */
static void chassis_stop(chassis_move_t *chassis)
{
    chassis->Vx_set = 0.0f;
    chassis->Vy_set = 0.0f;
    chassis->Wz_set = 0.0f;
}

/* ============================================================
 *  公开: 切换车辆工作模式
 * ============================================================ */
void Chassis_SetMode(chassis_move_t *chassis, CarMode_t mode)
{
    if (chassis == NULL) return;

    chassis_stop(chassis);
    chassis->mode = mode;
}

/* ============================================================
 *  步骤1: 底盘控制模式切换
 *  处理遥控/自主模式切换请求
 * ============================================================ */
void chassis_mode_change(chassis_move_t *chassis)
{
    if (chassis == NULL) return;

    /* 预留: 根据上位机指令切换模式
     * 当前默认保持 CAR_MODE_REMOTE
     */
    if (chassis->mode == CAR_MODE_STOP) {
        chassis->mode = CAR_MODE_REMOTE;
    }
}

/* ============================================================
 *  步骤2: 传感器数据刷新
 *  编码器 → chassis->motor[i].speed
 *  每周期在控制循环之前调用
 * ============================================================ */
void chassis_feedback_update(chassis_move_t *chassis)
{
    if (chassis == NULL) return;

    for (uint8_t i = 0; i < 4; i++)
    {
        int16_t rpm;
        Encoder_Rpm_Get(i, &rpm);
        chassis->motor[i].speed = (double)rpm;
    }
}

/* ============================================================
 *  步骤3: 底盘控制量设置
 *  优先级: UART遥控指令 → 默认停车
 * ============================================================ */
void chassis_set_control(chassis_move_t *chassis)
{
    if (chassis == NULL) return;

    switch (chassis->mode) {
    case CAR_MODE_REMOTE:
        /* 遥控模式: Vx/Vy/Wz 由外部 (UART中断) 写入 chassis->cmd_vel
         * 在此转换为内部速度目标
         * cmd_vel.vx → Vx_set (前向), cmd_vel.vy → Vy_set (横向),
         * cmd_vel.vz → Wz_set (旋转)
         */
        chassis->Vx_set = chassis->cmd_vel.vx * 100.0f;
        chassis->Vy_set = chassis->cmd_vel.vy * 100.0f;
        chassis->Wz_set = chassis->cmd_vel.vz * 30.0f;
        break;

    case CAR_MODE_AUTO:
        /* 预留: 自主导航 */
        break;

    case CAR_MODE_STOP:
    default:
        chassis_stop(chassis);
        break;
    }
}

/* ============================================================
 *  步骤4: 底盘核心控制循环
 *    Vx/Vy/Wz (已由步骤3写入) → 运动学分解 → PID 计算
 *    PID 输出由步骤5 chassis_send_cmd 统一发送到电机
 * ============================================================ */
void chassis_control_loop(chassis_move_t *chassis)
{
    /* 1. 全向运动学分解: Vx/Vy/Wz → 4路电机目标转速 */
    chassis->motor[0].speed_set =  chassis->Vx_set + chassis->Vy_set + chassis->Wz_set;
    chassis->motor[1].speed_set =  chassis->Vx_set - chassis->Vy_set - chassis->Wz_set;
    chassis->motor[2].speed_set =  chassis->Vx_set - chassis->Vy_set + chassis->Wz_set;
    chassis->motor[3].speed_set =  chassis->Vx_set + chassis->Vy_set - chassis->Wz_set;

    /* 2. PID 速度闭环 (结果存入 motor[i].speed_pid.Out) */
    for (uint8_t i = 0; i < 4; i++)
    {
        PID_Calculate(&chassis->motor[i].speed_pid,
                      chassis->motor[i].speed,
                      chassis->motor[i].speed_set);
    }
}

/* ============================================================
 *  步骤5: 底盘控制指令发送
 *    PID 输出 → 电机PWM
 * ============================================================ */
void chassis_send_cmd(chassis_move_t *chassis)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        Motor_SetPWM((int16_t)chassis->motor[i].speed_pid.Out, i);
    }
}

/* ============================================================
 *  初始化: 编码器 / 电机 / PID
 *  在 FreeRTOS 任务中调用一次
 * ============================================================ */
static void chassis_init(chassis_move_t *chassis)
{
    const static double speed_pid_param[3] =
    {
        MOTOR_SPEED_PID_KP,
        MOTOR_SPEED_PID_KI,
        MOTOR_SPEED_PID_KD
    };

    Encoder_Init();
    Motor_Init();

    for (uint8_t i = 0; i < 4; i++)
    {
        PID_init(&chassis->motor[i].speed_pid, PID_POSITION,
                 speed_pid_param,
                 MOTOR_SPEED_PID_MAX_OUT,
                 MOTOR_SPEED_PID_MAX_IOUT);
    }
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis->motor[i].speed_set = 0;
    }
}

/* ============================================================
 *  FreeRTOS 任务入口
 * ============================================================ */
void chassis_task(void *pvParameters)
{
    /* -- 一次性初始化 -- */
    chassis_init(&chassis_move);

    /* -- 默认模式: 遥控 -- */
    chassis_move.mode = CAR_MODE_REMOTE;

    /* -- 主循环 (100Hz) -- */
    while (1)
    {
        chassis_mode_change(&chassis_move);
        chassis_feedback_update(&chassis_move);
        chassis_set_control(&chassis_move);
        chassis_control_loop(&chassis_move);
        chassis_send_cmd(&chassis_move);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
