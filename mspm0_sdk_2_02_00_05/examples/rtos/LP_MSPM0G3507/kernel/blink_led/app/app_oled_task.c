/*
 * OLED 显示刷新任务 — 5Hz 只读显示
 *
 * 数据流向:
 *   chassis_move (全局, 由 ChassisBoardTask 100Hz 更新)
 *     ├─ motor[0].speed / motor[1].speed → OLED 第2行 (双轮 RPM)
 *     ├─ imu.roll / imu.pitch            → OLED 第3行 (欧拉角)
 *     └─ imu.yaw                         → OLED 第4行 (偏航)
 *
 *   刷新周期: 200ms (5Hz), 低于底盘任务(100Hz)以保证数据一致性
 *
 * 引脚: PB2(SCL)/PB3(SDA) → SSD1306 I2C
 */

#include "app_oled_task.h"
#include "app_chassis.h"
#include "bsp_oled.h"

#include "FreeRTOS.h"
#include "task.h"

extern chassis_move_t chassis_move;

/* ============================================================
 *  OLED 刷新任务 (5Hz)
 * ============================================================ */
void oled_display_task(void *pvParameters)
{
    (void)pvParameters;

    OLED_Init();
    OLED_Clear();

    while (1) {
        if (chassis_move.mode == CAR_MODE_STOP) {
            oled_printf(1, 1, "Mode: STOP     ");
        } else {
            oled_printf(1, 1, "Mode: RUN      ");
        }

        oled_printf(2, 1, "A:%5.1f B:%5.1f",
                    (double)chassis_move.motor[0].speed,
                    (double)chassis_move.motor[1].speed);

        oled_printf(3, 1, "R:%4.1f P:%4.1f",
                    (double)chassis_move.imu.roll,
                    (double)chassis_move.imu.pitch);
        oled_printf(4, 1, "Y:%5.1f",
                    (double)chassis_move.imu.yaw);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
