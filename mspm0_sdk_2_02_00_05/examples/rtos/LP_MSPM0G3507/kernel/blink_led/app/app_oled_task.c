/*
 * OLED 显示刷新任务 — 5Hz 只读显示
 *
 * 数据流向:
 *   bsp_track_read()                    → OLED 第1行 (C1~C8)
 *   chassis_move (全局, 由 ChassisBoardTask 100Hz 更新)
 *     ├─ motor[0].speed / motor[1].speed → OLED 第2行 (左/右轮 RPM)
 *     ├─ imu.roll / imu.pitch            → OLED 第3行 (欧拉角)
 *     └─ imu.yaw                         → OLED 第4行 (偏航)
 *
 *   刷新周期: 200ms (5Hz), 低于底盘任务(100Hz)以保证数据一致性
 *
 * 引脚: PB2(SCL)/PB3(SDA) → SSD1306 I2C
 */

#include "app_oled_task.h"
#include "app_chassis.h"
#include "bsp_buzzer.h"
#include "bsp_key.h"
#include "bsp_oled.h"
#include "bsp_track.h"

#include "FreeRTOS.h"
#include "task.h"

/* ============================================================
 *  OLED 刷新任务 (5Hz)
 * ============================================================ */
void oled_display_task(void *pvParameters)
{
    chassis_move_t *chassis = (chassis_move_t *)pvParameters;
    uint8_t data_refresh_divider = 0U;

    if (chassis == NULL) {
        vTaskDelete(NULL);
        return;
    }

    OLED_Init();
    OLED_Clear();

    while (1) {
        char key_display = ' ';

        app_watchdog_task_heartbeat(&chassis->watchdog,
                                    APP_WATCHDOG_TASK_OLED);

        if (bsp_key_is_pressed(BSP_KEY_FRONT)) {
            key_display = '1';
        } else if (bsp_key_is_pressed(BSP_KEY_BACK)) {
            key_display = '2';
        } else if (bsp_key_is_pressed(BSP_KEY_LEFT_PHYSICAL)) {
            key_display = '3';
        } else if (bsp_key_is_pressed(BSP_KEY_RIGHT_PHYSICAL)) {
            key_display = '4';
        } else if (bsp_key_is_pressed(BSP_KEY_MIDDLE)) {
            key_display = '5';
        }
        bsp_buzzer_set(key_display != ' ');

        if (data_refresh_divider == 0U) {
            uint8_t track = bsp_track_read();

            oled_printf(1, 1, "TRK:%u%u%u%u%u%u%u%u    ",
                        (unsigned int) ((track >> 0U) & 1U),
                        (unsigned int) ((track >> 1U) & 1U),
                        (unsigned int) ((track >> 2U) & 1U),
                        (unsigned int) ((track >> 3U) & 1U),
                        (unsigned int) ((track >> 4U) & 1U),
                        (unsigned int) ((track >> 5U) & 1U),
                        (unsigned int) ((track >> 6U) & 1U),
                        (unsigned int) ((track >> 7U) & 1U));

            oled_printf(2, 1, "L:%5.1f R:%5.1f",
                        (double)chassis->motor[0].speed,
                        (double)chassis->motor[1].speed);

            oled_printf(3, 1, "R:%4.1f P:%4.1f",
                        (double)chassis->imu.roll,
                        (double)chassis->imu.pitch);
            oled_printf(4, 1, "Y:%5.1f",
                        (double)chassis->imu.yaw);
        }
        OLED_ShowChar(1, 16, key_display);

        data_refresh_divider++;
        if (data_refresh_divider >= 4U) {
            data_refresh_divider = 0U;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
