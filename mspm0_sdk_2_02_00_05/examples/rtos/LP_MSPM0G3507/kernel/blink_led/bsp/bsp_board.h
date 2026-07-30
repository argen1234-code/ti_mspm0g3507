#ifndef BSP_BOARD_H
#define BSP_BOARD_H

/*
 * BSP 聚合头 — 全工程唯一公共包含入口
 *   每个 .c 文件 #include "bsp_board.h" 即可获得:
 *     - TI 驱动库 (ti_msp_dl_config.h)
 *     - 所有 BSP 子模块 (motor/encoder/uart/oled)
 *     - PID 控制器 (pid.h)
 *     - 常用类型别名 (s32, u16, ...)
 *     - 工具宏 ABS(), Flag_Stop
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "ti_msp_dl_config.h"

/* BSP modules */
#include "bsp_buzzer.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_uart.h"
#include "bsp_oled.h"
#include "bsp_track.h"
#include "bsp_watchdog.h"
#include "bsp_yabo_IMU.h"
#include "bsp_jy61s.h"
#include "bsp_jy901s.h"
#include "bsp_ZDT.h"
#include "pid.h"

#define ABS(a) ((a) > 0 ? (a) : -(a))

typedef int32_t  s32;
typedef int16_t  s16;
typedef int8_t   s8;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;

extern int Flag_Stop;   /* 全局启停标志: 1=停止, 0=运行 */

void bsp_board_led_set(uint8_t led, bool on);
void bsp_board_buzzer_set(bool on);

#endif
