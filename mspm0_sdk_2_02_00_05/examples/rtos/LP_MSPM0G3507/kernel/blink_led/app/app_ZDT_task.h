#ifndef APP_ZDT_TASK_H
#define APP_ZDT_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_ZDT.h"

#define APP_ZDT_CONTROL_MODE_PULSE  (0U)
#define APP_ZDT_CONTROL_MODE_UART   (1U)

/*
 * Verified pulse-control wiring retained for later reuse:
 * COM=3.3V, STEP=PA7, DIR=PA16, EN=PB17, active-high MCU outputs.
 * UART control uses UART2: PB15 TX, PB16 RX, 115200-8-N-1.
 */
#ifndef APP_ZDT_CONTROL_MODE
#define APP_ZDT_CONTROL_MODE        APP_ZDT_CONTROL_MODE_UART
#endif

/* X42S UART position safety range and the motor's configured subdivision. */
#define APP_ZDT_ANGLE_LOWER_DEG                (240.0f)
#define APP_ZDT_ANGLE_UPPER_DEG                (335.0f)
/* 3200 pulses/rev gives 0.1125 degree/pulse; use less than half a pulse. */
#define APP_ZDT_ANGLE_EPSILON_DEG              (0.05f)
#define APP_ZDT_FEEDBACK_TOLERANCE_DEG         (5.0f)
#define APP_ZDT_UART_PULSES_PER_REVOLUTION     (3200U)

/* Automatic UART sweep test: 240 <-> 335 degrees. */
#ifndef APP_ZDT_UART_SWEEP_TEST_ENABLE
#define APP_ZDT_UART_SWEEP_TEST_ENABLE          (0U)
#endif
#define APP_ZDT_SWEEP_SPEED_RPM                 (20U)
#define APP_ZDT_SWEEP_ACCELERATION              (20U)
#define APP_ZDT_SWEEP_DWELL_MS                  (1000U)
#define APP_ZDT_SWEEP_POLL_MS                   (50U)
#define APP_ZDT_SWEEP_TIMEOUT_MS                (3000U)
#define APP_ZDT_SWEEP_MAX_CORRECTIONS           (3U)
#define APP_ZDT_SWEEP_ARRIVAL_EPSILON_DEG       (5.0f)
#define APP_ZDT_SWEEP_ENDPOINT_MARGIN_DEG        (0.5f)
#define APP_ZDT_SWEEP_RECOVERY_WINDOW_DEG        (5.0f)
#define APP_ZDT_SWEEP_RECOVERY_SPEED_RPM         (10U)
#define APP_ZDT_SWEEP_RECOVERY_ACCELERATION      (10U)
#define APP_ZDT_SWEEP_LOWER_TARGET_DEG           \
    (APP_ZDT_ANGLE_LOWER_DEG + APP_ZDT_SWEEP_ENDPOINT_MARGIN_DEG)
#define APP_ZDT_SWEEP_UPPER_TARGET_DEG           \
    (APP_ZDT_ANGLE_UPPER_DEG - APP_ZDT_SWEEP_ENDPOINT_MARGIN_DEG)

/* Change to ZDT_DIR_CCW only if CCW makes the 0x31 angle increase. */
#ifndef APP_ZDT_ENCODER_INCREASE_DIRECTION
#define APP_ZDT_ENCODER_INCREASE_DIRECTION     ZDT_DIR_CW
#endif

void zdt_motor_task(void *pvParameters);

/* Retained STEP/DIR/EN pulse-control API. */
zdt_result_t app_zdt_pulse_init(zdt_motor_t *motor);
zdt_result_t app_zdt_enable(zdt_motor_t *motor, bool enable);
zdt_result_t app_zdt_move_pulses(zdt_motor_t *motor,
                                 zdt_direction_t direction,
                                 uint32_t frequency_hz,
                                 uint32_t pulses);
zdt_result_t app_zdt_run_continuous(zdt_motor_t *motor,
                                    zdt_direction_t direction,
                                    uint32_t frequency_hz);
void app_zdt_stop(zdt_motor_t *motor);

/* Active X42S Emm serial-control API on UART2: PB15 TX, PB16 RX. */
zdt_result_t app_zdt_uart_init(zdt_uart_t *motor);
zdt_result_t app_zdt_uart_enable(zdt_uart_t *motor, bool enable);
zdt_result_t app_zdt_uart_move(zdt_uart_t *motor,
                               zdt_direction_t direction,
                               uint16_t speed_rpm,
                               uint8_t acceleration,
                               uint32_t pulses);
zdt_result_t app_zdt_uart_move_to_angle(zdt_uart_t *motor,
                                        float target_degrees,
                                        uint16_t speed_rpm,
                                        uint8_t acceleration);
zdt_result_t app_zdt_uart_run(zdt_uart_t *motor,
                              zdt_direction_t direction,
                              uint16_t speed_rpm,
                              uint8_t acceleration);
zdt_result_t app_zdt_uart_stop(zdt_uart_t *motor);
zdt_result_t app_zdt_uart_request_angle_once(zdt_uart_t *motor);

#endif
