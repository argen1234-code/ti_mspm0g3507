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
    GPIO_Regs *port;
    uint32_t pin;

    switch (led) {
    case 1U:
        port = LEDS_LED1_PORT;
        pin = LEDS_LED1_PIN;
        break;
    case 2U:
        port = LEDS_LED2_PORT;
        pin = LEDS_LED2_PIN;
        break;
    case 3U:
        port = LEDS_LED3_PORT;
        pin = LEDS_LED3_PIN;
        break;
    default:
        return;
    }

    if (on) {
        DL_GPIO_clearPins(port, pin);
    } else {
        DL_GPIO_setPins(port, pin);
    }
}

void bsp_board_buzzer_set(bool on)
{
    if (on) {
        DL_GPIO_setPins(BUZZER_GPIO_PORT, BUZZER_GPIO_BUZZER_PIN);
    } else {
        DL_GPIO_clearPins(BUZZER_GPIO_PORT, BUZZER_GPIO_BUZZER_PIN);
    }
}
