/*
 * BSP 板级聚合模块
 *
 * 职责:
 *   - 定义全局标志 Flag_Stop (底盘启停控制)
 *   - bsp_board.h 聚合所有 BSP 子模块头文件
 *
 * 数据流:
 *   Flag_Stop ──→ chassis_move.flag_stop ──→ 控制逻辑 (当前未实现, 预留)
 */

#include "bsp_board.h"

int Flag_Stop = 1;

void bsp_board_led_set(uint8_t led, bool on)
{
    if ((led < 1U) || (led > 3U)) return;
    bsp_led_set((bsp_led_id_t) (led - 1U), on);
}

void bsp_board_buzzer_set(bool on)
{
    bsp_buzzer_set(on);
}
