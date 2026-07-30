#include "app_ZDT_task.h"
#include "app_chassis.h"

#include "bsp_uart.h"

#include "FreeRTOS.h"
#include "task.h"

/*
 * Verified hardware baseline:
 * COM is connected to 3.3 V and STEP/DIR/EN are driven active-high.
 * Set this option only for a different board that is proven active-low.
 */
#ifndef APP_ZDT_INPUT_ACTIVE_LOW
#define APP_ZDT_INPUT_ACTIVE_LOW  (0)
#endif

#define APP_ZDT_STARTUP_DELAY_MS       (2000U)
#define APP_ZDT_MONITOR_PERIOD_MS      (5U)
#define APP_ZDT_DIRECTION_PAUSE_MS     (1000U)
#define APP_ZDT_COMMAND_RETRY_MS       (500U)
#define APP_ZDT_TEST_SPEED_RPM         (60.0f)
#define APP_ZDT_UART_ADDRESS            BSP_ZDT_UART_DEFAULT_ADDRESS
#define APP_ZDT_UART_FEEDBACK_DELAY_MS    (20U)
#define APP_ZDT_UART_SERVICE_PERIOD_MS   (5U)
#define APP_ZDT_UART_BOOTSTRAP_RETRY_MS (100U)
#define APP_ZDT_UART_FEEDBACK_TIMEOUT_MS (150U)

static const zdt_config_t g_zdt_config = {
    .step_port = ZDT_GPIO_STEP_PORT,
    .step_pin = ZDT_GPIO_STEP_PIN,
    .dir_port = ZDT_GPIO_DIR_PORT,
    .dir_pin = ZDT_GPIO_DIR_PIN,
    .enable_port = ZDT_GPIO_EN_PORT,
    .enable_pin = ZDT_GPIO_EN_PIN,
    .timer = ZDT_PULSE_TIMER_INST,
    .timer_irqn = ZDT_PULSE_TIMER_INST_INT_IRQN,
    .timer_clock_hz = CPUCLK_FREQ,

#if APP_ZDT_INPUT_ACTIVE_LOW
    .step_active_high = false,
    .dir_cw_high = false,
    .enable_active_high = false,
#else
    .step_active_high = true,
    .dir_cw_high = true,
    .enable_active_high = true,
#endif

    .microsteps = BSP_ZDT_DEFAULT_MICROSTEPS,
    .motor_step_angle_deg = BSP_ZDT_DEFAULT_MOTOR_STEP_ANGLE_DEG,
};

static const zdt_uart_config_t g_zdt_uart_config = {
    .uart = UART_2_INST,
    .address = APP_ZDT_UART_ADDRESS,
    .checksum = BSP_ZDT_UART_DEFAULT_CHECKSUM,
};

static bool app_zdt_uart_angle_limit_is_valid(const zdt_uart_t *motor)
{
    return (motor != NULL) && motor->angle_limit.enabled &&
           (motor->angle_limit.lower_degrees >= 0.0f) &&
           (motor->angle_limit.upper_degrees < 360.0f) &&
           (motor->angle_limit.lower_degrees <
            motor->angle_limit.upper_degrees) &&
           (motor->angle_limit.pulses_per_revolution != 0U);
}

static zdt_direction_t app_zdt_direction_from_delta(float delta_degrees)
{
    if (delta_degrees > 0.0f) {
        return APP_ZDT_ENCODER_INCREASE_DIRECTION;
    }

    return (APP_ZDT_ENCODER_INCREASE_DIRECTION == ZDT_DIR_CW)
               ? ZDT_DIR_CCW
               : ZDT_DIR_CW;
}

static zdt_result_t app_zdt_queue_angle_feedback(zdt_uart_t *motor)
{
    if (motor == NULL) {
        return ZDT_RESULT_NULL;
    }
    if (!motor->initialized) {
        return ZDT_RESULT_NOT_INITIALIZED;
    }

    motor->angle_feedback_requests++;
    return ZDT_RESULT_OK;
}

static zdt_result_t app_zdt_uart_recover_near_limit(zdt_uart_t *motor,
                                                     float target_degrees)
{
    zdt_angle_limit_t *limit;
    float current_degrees;
    float delta_degrees;
    float absolute_delta;
    uint32_t pulses;
    zdt_result_t result;

    if (motor == NULL) {
        return ZDT_RESULT_NULL;
    }
    if (!motor->initialized) {
        return ZDT_RESULT_NOT_INITIALIZED;
    }
    if (!motor->feedback.encoder_absolute_valid ||
        !app_zdt_uart_angle_limit_is_valid(motor)) {
        return ZDT_RESULT_FEEDBACK_NOT_VALID;
    }

    limit = &motor->angle_limit;
    current_degrees = motor->feedback.encoder_absolute_degrees;
    if ((target_degrees < limit->lower_degrees) ||
        (target_degrees > limit->upper_degrees)) {
        return ZDT_RESULT_ANGLE_LIMIT;
    }

    if (current_degrees > limit->upper_degrees) {
        if ((current_degrees >
             (limit->upper_degrees + APP_ZDT_SWEEP_RECOVERY_WINDOW_DEG)) ||
            (target_degrees >= current_degrees)) {
            return ZDT_RESULT_ANGLE_LIMIT;
        }
    } else if (current_degrees < limit->lower_degrees) {
        if ((current_degrees <
             (limit->lower_degrees - APP_ZDT_SWEEP_RECOVERY_WINDOW_DEG)) ||
            (target_degrees <= current_degrees)) {
            return ZDT_RESULT_ANGLE_LIMIT;
        }
    } else {
        return ZDT_RESULT_OK;
    }

    delta_degrees = target_degrees - current_degrees;
    absolute_delta = (delta_degrees < 0.0f)
                         ? -delta_degrees
                         : delta_degrees;
    pulses = (uint32_t)((absolute_delta *
                         (float)limit->pulses_per_revolution / 360.0f) +
                        0.5f);
    if (pulses == 0U) {
        return ZDT_RESULT_OK;
    }

    limit->requested_target_degrees = target_degrees;
    limit->limited_target_degrees = target_degrees;
    limit->last_delta_degrees = delta_degrees;
    limit->last_command_pulses = pulses;

    result = bsp_zdt_uart_position(
        motor,
        app_zdt_direction_from_delta(delta_degrees),
        APP_ZDT_SWEEP_RECOVERY_SPEED_RPM,
        APP_ZDT_SWEEP_RECOVERY_ACCELERATION,
        pulses,
        ZDT_POSITION_RELATIVE_CURRENT,
        false);
    if (result == ZDT_RESULT_OK) {
        (void)app_zdt_queue_angle_feedback(motor);
    }
    return result;
}

void zdt_motor_task(void *pvParameters)
{
    chassis_move_t *chassis = (chassis_move_t *)pvParameters;
    chassis_zdt_t *zdt;

#if APP_ZDT_CONTROL_MODE == APP_ZDT_CONTROL_MODE_PULSE
    zdt_direction_t next_direction = ZDT_DIR_CW;
    uint32_t pulse_frequency_hz;
    uint32_t one_revolution_pulses;
#elif APP_ZDT_CONTROL_MODE == APP_ZDT_CONTROL_MODE_UART
    TickType_t feedback_due_tick = 0U;
    TickType_t bootstrap_next_query_tick = 0U;
    TickType_t position_feedback_deadline_tick = 0U;
    bool feedback_delay_active = false;
    uint32_t position_feedback_sample = 0U;
#if APP_ZDT_UART_SWEEP_TEST_ENABLE
    TickType_t sweep_next_action_tick = 0U;
    TickType_t sweep_next_poll_tick = 0U;
    TickType_t sweep_deadline_tick = 0U;
    uint32_t sweep_last_angle_sample = 0U;
    bool sweep_initialized = false;
    bool sweep_recovery_active = false;
    bool sweep_recovery_from_upper = false;
    bool sweep_settle_query_sent = false;
    float sweep_recovery_previous_angle = 0.0f;
#endif
#else
#error "Unsupported APP_ZDT_CONTROL_MODE"
#endif

    if (chassis == NULL) {
        vTaskDelete(NULL);
        return;
    }
    zdt = &chassis->zdt;
    zdt->task_running = true;
    zdt->task_heartbeat = 0U;
    zdt->init_result = ZDT_RESULT_NOT_INITIALIZED;
    zdt->motion_result = ZDT_RESULT_NOT_INITIALIZED;
    zdt->poll_result = ZDT_RESULT_NOT_INITIALIZED;
    zdt->sweep_target_degrees = 0.0f;
    zdt->sweep_test_active = false;
    zdt->sweep_motion_active = false;
    zdt->sweep_waiting_settle_feedback = false;
    zdt->sweep_fault = false;
    zdt->sweep_fault_reason = ZDT_SWEEP_FAULT_NONE;
    zdt->sweep_correction_attempts = 0U;
    zdt->sweep_moves_completed = 0U;
    zdt->sweep_corrections = 0U;
    zdt->sweep_settle_queries = 0U;
    zdt->position_applied_sequence = 0U;
    zdt->position_waiting_feedback = false;
    zdt->position_commands_sent = 0U;
    zdt->position_noop_updates = 0U;
    zdt->position_command_errors = 0U;
    zdt->position_feedback_retries = 0U;
    zdt->bootstrap_feedback_requests = 0U;

    /* Allow the closed-loop driver to finish its own power-on initialization. */
    vTaskDelay(pdMS_TO_TICKS(APP_ZDT_STARTUP_DELAY_MS));

#if APP_ZDT_CONTROL_MODE == APP_ZDT_CONTROL_MODE_PULSE
    zdt->init_result = app_zdt_pulse_init(&zdt->pulse_motor);
    if (zdt->init_result == ZDT_RESULT_OK) {
        zdt->init_result = app_zdt_enable(&zdt->pulse_motor, true);
    }

    one_revolution_pulses =
        bsp_zdt_pulses_per_revolution(&zdt->pulse_motor);
    pulse_frequency_hz =
        bsp_zdt_rpm_to_frequency(&zdt->pulse_motor, APP_ZDT_TEST_SPEED_RPM);

    if ((zdt->init_result == ZDT_RESULT_OK) &&
        ((one_revolution_pulses == 0U) || (pulse_frequency_hz == 0U))) {
        zdt->init_result = ZDT_RESULT_INVALID_CONFIG;
    }
#else
    zdt->init_result = app_zdt_uart_init(&zdt->uart_motor);
    if (zdt->init_result == ZDT_RESULT_OK) {
        /* Start enabled so the X42S closed loop holds its current angle. */
        zdt->init_result = app_zdt_uart_enable(&zdt->uart_motor, true);
    }
    bsp_zdt_uart_reset_observed_angle_range(&zdt->uart_motor);
    bootstrap_next_query_tick = xTaskGetTickCount();
#endif

    for (;;) {
        zdt->task_heartbeat++;

        if (zdt->init_result != ZDT_RESULT_OK) {
            vTaskDelay(pdMS_TO_TICKS(APP_ZDT_COMMAND_RETRY_MS));
            continue;
        }

#if APP_ZDT_CONTROL_MODE == APP_ZDT_CONTROL_MODE_PULSE
        zdt->motion_result = app_zdt_move_pulses(
            &zdt->pulse_motor,
            next_direction,
            pulse_frequency_hz,
            one_revolution_pulses);

        if (zdt->motion_result == ZDT_RESULT_OK) {
            while (bsp_zdt_is_busy(&zdt->pulse_motor)) {
                vTaskDelay(pdMS_TO_TICKS(APP_ZDT_MONITOR_PERIOD_MS));
            }

            next_direction = (next_direction == ZDT_DIR_CW)
                                 ? ZDT_DIR_CCW
                                 : ZDT_DIR_CW;
            vTaskDelay(pdMS_TO_TICKS(APP_ZDT_DIRECTION_PAUSE_MS));
        } else {
            vTaskDelay(pdMS_TO_TICKS(APP_ZDT_COMMAND_RETRY_MS));
        }
#else
        TickType_t now = xTaskGetTickCount();

        /* Consume every UART2 byte before sending the next query/command. */
        (void)bsp_zdt_uart_process_rx(&zdt->uart_motor);

        /*
         * Position control cannot calibrate its neutral angle before the
         * first 0x31 response arrives.  Retry the read request periodically
         * so one lost startup response cannot leave the controller disabled
         * forever.
         */
        if (!zdt->uart_motor.feedback.encoder_absolute_valid &&
            ((int32_t)(now - bootstrap_next_query_tick) >= 0) &&
            (zdt->uart_motor.angle_feedback_requests ==
             zdt->uart_motor.angle_feedback_queries)) {
            zdt->poll_result = app_zdt_uart_request_angle_once(
                &zdt->uart_motor);
            if (zdt->poll_result == ZDT_RESULT_OK) {
                zdt->bootstrap_feedback_requests++;
            }
            bootstrap_next_query_tick = now +
                pdMS_TO_TICKS(APP_ZDT_UART_BOOTSTRAP_RETRY_MS);
        }
        if (zdt->uart_motor.feedback.encoder_absolute_valid &&
            zdt->uart_motor.angle_limit.enabled) {
            zdt->uart_motor.angle_limit.lower_active =
                zdt->uart_motor.feedback.encoder_absolute_degrees <=
                (zdt->uart_motor.angle_limit.lower_degrees +
                 APP_ZDT_ANGLE_EPSILON_DEG);
            zdt->uart_motor.angle_limit.upper_active =
                zdt->uart_motor.feedback.encoder_absolute_degrees >=
                (zdt->uart_motor.angle_limit.upper_degrees -
                 APP_ZDT_ANGLE_EPSILON_DEG);
            zdt->uart_motor.angle_limit.feedback_outside_range =
                (zdt->uart_motor.feedback.encoder_absolute_degrees <
                 (zdt->uart_motor.angle_limit.lower_degrees -
                  APP_ZDT_FEEDBACK_TOLERANCE_DEG)) ||
                (zdt->uart_motor.feedback.encoder_absolute_degrees >
                 (zdt->uart_motor.angle_limit.upper_degrees +
                  APP_ZDT_FEEDBACK_TOLERANCE_DEG));

            if (zdt->uart_motor.angle_limit.feedback_outside_range) {
                if (!zdt->uart_motor.angle_limit.emergency_stop_latched) {
                    zdt->motion_result =
                        bsp_zdt_uart_stop(&zdt->uart_motor, false);
                    zdt->uart_motor.angle_limit.emergency_stop_latched = true;
                    if (zdt->motion_result == ZDT_RESULT_OK) {
                        zdt->uart_motor.angle_limit.emergency_stop_events++;
                    }
                }
            } else {
                zdt->uart_motor.angle_limit.emergency_stop_latched = false;
            }
        }

#if APP_ZDT_UART_SWEEP_TEST_ENABLE
        if (zdt->uart_motor.feedback.encoder_absolute_valid) {
            float current_degrees =
                zdt->uart_motor.feedback.encoder_absolute_degrees;
            float midpoint_degrees =
                (APP_ZDT_ANGLE_LOWER_DEG + APP_ZDT_ANGLE_UPPER_DEG) * 0.5f;

            if (zdt->uart_motor.angle_limit.feedback_outside_range) {
                if (!sweep_initialized && !zdt->sweep_fault) {
                    zdt->sweep_target_degrees =
                        (current_degrees > APP_ZDT_ANGLE_UPPER_DEG)
                            ? APP_ZDT_SWEEP_UPPER_TARGET_DEG
                            : APP_ZDT_SWEEP_LOWER_TARGET_DEG;
                    zdt->motion_result = app_zdt_uart_recover_near_limit(
                        &zdt->uart_motor,
                        zdt->sweep_target_degrees);

                    if (zdt->motion_result == ZDT_RESULT_OK) {
                        sweep_initialized = true;
                        sweep_recovery_active = true;
                        sweep_recovery_from_upper =
                            current_degrees > APP_ZDT_ANGLE_UPPER_DEG;
                        sweep_recovery_previous_angle = current_degrees;
                        sweep_last_angle_sample =
                            zdt->uart_motor.feedback.angle_sample_count;
                        zdt->sweep_test_active = true;
                        zdt->sweep_motion_active = true;
                        sweep_next_poll_tick = now +
                            pdMS_TO_TICKS(APP_ZDT_SWEEP_POLL_MS);
                        sweep_deadline_tick = now +
                            pdMS_TO_TICKS(APP_ZDT_SWEEP_TIMEOUT_MS);
                    } else {
                        zdt->sweep_test_active = false;
                        zdt->sweep_motion_active = false;
                        zdt->sweep_fault = true;
                        zdt->sweep_fault_reason =
                            ZDT_SWEEP_FAULT_OUTSIDE_RANGE;
                    }
                } else if (sweep_recovery_active && !zdt->sweep_fault) {
                    if (zdt->uart_motor.feedback.angle_sample_count !=
                        sweep_last_angle_sample) {
                        bool moving_farther_out =
                            sweep_recovery_from_upper
                                ? (current_degrees >
                                   (sweep_recovery_previous_angle +
                                    APP_ZDT_FEEDBACK_TOLERANCE_DEG))
                                : (current_degrees <
                                   (sweep_recovery_previous_angle -
                                    APP_ZDT_FEEDBACK_TOLERANCE_DEG));

                        sweep_last_angle_sample =
                            zdt->uart_motor.feedback.angle_sample_count;
                        sweep_recovery_previous_angle = current_degrees;
                        if (moving_farther_out ||
                            (current_degrees >
                             (APP_ZDT_ANGLE_UPPER_DEG +
                              APP_ZDT_SWEEP_RECOVERY_WINDOW_DEG)) ||
                            (current_degrees <
                             (APP_ZDT_ANGLE_LOWER_DEG -
                              APP_ZDT_SWEEP_RECOVERY_WINDOW_DEG))) {
                            zdt->motion_result =
                                app_zdt_uart_stop(&zdt->uart_motor);
                            zdt->sweep_test_active = false;
                            zdt->sweep_motion_active = false;
                            zdt->sweep_fault = true;
                            zdt->sweep_fault_reason =
                                ZDT_SWEEP_FAULT_OUTSIDE_RANGE;
                            sweep_recovery_active = false;
                        }
                    }

                    if (!zdt->sweep_fault &&
                        ((int32_t)(now - sweep_deadline_tick) >= 0)) {
                        zdt->motion_result =
                            app_zdt_uart_stop(&zdt->uart_motor);
                        zdt->sweep_test_active = false;
                        zdt->sweep_motion_active = false;
                        zdt->sweep_fault = true;
                        zdt->sweep_fault_reason = ZDT_SWEEP_FAULT_TIMEOUT;
                        sweep_recovery_active = false;
                    } else if (!zdt->sweep_fault &&
                               ((int32_t)(now - sweep_next_poll_tick) >= 0) &&
                               (zdt->uart_motor.angle_feedback_requests ==
                                zdt->uart_motor.angle_feedback_queries)) {
                        zdt->poll_result = app_zdt_uart_request_angle_once(
                            &zdt->uart_motor);
                        sweep_next_poll_tick = now +
                            pdMS_TO_TICKS(APP_ZDT_SWEEP_POLL_MS);
                    }
                } else {
                    zdt->sweep_test_active = false;
                    zdt->sweep_motion_active = false;
                    zdt->sweep_fault = true;
                    zdt->sweep_fault_reason =
                        ZDT_SWEEP_FAULT_OUTSIDE_RANGE;
                }
            } else if (!zdt->sweep_fault) {
                if (sweep_recovery_active) {
                    sweep_recovery_active = false;
                }
                if (!sweep_initialized) {
                    zdt->sweep_target_degrees =
                        (current_degrees <= midpoint_degrees)
                            ? APP_ZDT_SWEEP_UPPER_TARGET_DEG
                            : APP_ZDT_SWEEP_LOWER_TARGET_DEG;
                    zdt->sweep_test_active = true;
                    sweep_next_action_tick = now;
                    sweep_last_angle_sample =
                        zdt->uart_motor.feedback.angle_sample_count;
                    sweep_initialized = true;
                }

                if (zdt->sweep_motion_active) {
                    if (zdt->uart_motor.feedback.angle_sample_count !=
                        sweep_last_angle_sample) {
                        float angle_error = current_degrees -
                                            zdt->sweep_target_degrees;

                        sweep_last_angle_sample =
                            zdt->uart_motor.feedback.angle_sample_count;
                        if (angle_error < 0.0f) {
                            angle_error = -angle_error;
                        }

                        if (angle_error <=
                            APP_ZDT_SWEEP_ARRIVAL_EPSILON_DEG) {
                            zdt->sweep_motion_active = false;
                            zdt->sweep_waiting_settle_feedback = true;
                            sweep_settle_query_sent = false;
                            sweep_last_angle_sample =
                                zdt->uart_motor.feedback.angle_sample_count;
                            sweep_next_action_tick = now +
                                pdMS_TO_TICKS(APP_ZDT_SWEEP_DWELL_MS);
                        }
                    }

                    if (zdt->sweep_motion_active &&
                        ((int32_t)(now - sweep_deadline_tick) >= 0)) {
                        if (zdt->sweep_correction_attempts <
                            APP_ZDT_SWEEP_MAX_CORRECTIONS) {
                            zdt->motion_result = app_zdt_uart_move_to_angle(
                                &zdt->uart_motor,
                                zdt->sweep_target_degrees,
                                APP_ZDT_SWEEP_SPEED_RPM,
                                APP_ZDT_SWEEP_ACCELERATION);

                            if (zdt->motion_result == ZDT_RESULT_OK) {
                                zdt->sweep_correction_attempts++;
                                zdt->sweep_corrections++;
                                sweep_last_angle_sample =
                                    zdt->uart_motor.feedback.angle_sample_count;
                                sweep_next_poll_tick = now +
                                    pdMS_TO_TICKS(APP_ZDT_SWEEP_POLL_MS);
                                sweep_deadline_tick = now +
                                    pdMS_TO_TICKS(APP_ZDT_SWEEP_TIMEOUT_MS);
                            } else {
                                zdt->sweep_motion_active = false;
                                zdt->sweep_test_active = false;
                                zdt->sweep_fault = true;
                                zdt->sweep_fault_reason =
                                    ZDT_SWEEP_FAULT_COMMAND;
                            }
                        } else {
                            zdt->motion_result =
                                app_zdt_uart_stop(&zdt->uart_motor);
                            zdt->sweep_motion_active = false;
                            zdt->sweep_test_active = false;
                            zdt->sweep_fault = true;
                            zdt->sweep_fault_reason =
                                ZDT_SWEEP_FAULT_TIMEOUT;
                        }
                    } else if (zdt->sweep_motion_active &&
                               ((int32_t)(now - sweep_next_poll_tick) >= 0) &&
                               (zdt->uart_motor.angle_feedback_requests ==
                                zdt->uart_motor.angle_feedback_queries)) {
                        zdt->poll_result = app_zdt_uart_request_angle_once(
                            &zdt->uart_motor);
                        sweep_next_poll_tick = now +
                            pdMS_TO_TICKS(APP_ZDT_SWEEP_POLL_MS);
                    }
                } else if (zdt->sweep_waiting_settle_feedback) {
                    if (!sweep_settle_query_sent &&
                        ((int32_t)(now - sweep_next_action_tick) >= 0) &&
                        (zdt->uart_motor.angle_feedback_requests ==
                         zdt->uart_motor.angle_feedback_queries)) {
                        zdt->poll_result = app_zdt_uart_request_angle_once(
                            &zdt->uart_motor);
                        if (zdt->poll_result == ZDT_RESULT_OK) {
                            sweep_settle_query_sent = true;
                            sweep_last_angle_sample =
                                zdt->uart_motor.feedback.angle_sample_count;
                            sweep_deadline_tick = now +
                                pdMS_TO_TICKS(APP_ZDT_SWEEP_TIMEOUT_MS);
                            zdt->sweep_settle_queries++;
                        } else {
                            zdt->sweep_waiting_settle_feedback = false;
                            zdt->sweep_test_active = false;
                            zdt->sweep_fault = true;
                            zdt->sweep_fault_reason =
                                ZDT_SWEEP_FAULT_COMMAND;
                        }
                    } else if (sweep_settle_query_sent &&
                               (zdt->uart_motor.feedback.angle_sample_count !=
                                sweep_last_angle_sample)) {
                        float settled_error = current_degrees -
                                              zdt->sweep_target_degrees;

                        if (settled_error < 0.0f) {
                            settled_error = -settled_error;
                        }
                        sweep_settle_query_sent = false;
                        zdt->sweep_waiting_settle_feedback = false;

                        if (settled_error <=
                            APP_ZDT_SWEEP_ARRIVAL_EPSILON_DEG) {
                            zdt->sweep_correction_attempts = 0U;
                            zdt->sweep_moves_completed++;
                            zdt->sweep_target_degrees =
                                (zdt->sweep_target_degrees > midpoint_degrees)
                                    ? APP_ZDT_SWEEP_LOWER_TARGET_DEG
                                    : APP_ZDT_SWEEP_UPPER_TARGET_DEG;
                            sweep_next_action_tick = now;
                        } else if (zdt->sweep_correction_attempts <
                                   APP_ZDT_SWEEP_MAX_CORRECTIONS) {
                            zdt->sweep_correction_attempts++;
                            zdt->sweep_corrections++;
                            sweep_next_action_tick = now;
                        } else {
                            zdt->motion_result =
                                app_zdt_uart_stop(&zdt->uart_motor);
                            zdt->sweep_test_active = false;
                            zdt->sweep_fault = true;
                            zdt->sweep_fault_reason =
                                ZDT_SWEEP_FAULT_TIMEOUT;
                        }
                    } else if (sweep_settle_query_sent &&
                               ((int32_t)(now - sweep_deadline_tick) >= 0)) {
                        sweep_settle_query_sent = false;
                        zdt->sweep_waiting_settle_feedback = false;
                        if (zdt->sweep_correction_attempts <
                            APP_ZDT_SWEEP_MAX_CORRECTIONS) {
                            zdt->sweep_correction_attempts++;
                            zdt->sweep_corrections++;
                            sweep_next_action_tick = now;
                        } else {
                            zdt->motion_result =
                                app_zdt_uart_stop(&zdt->uart_motor);
                            zdt->sweep_test_active = false;
                            zdt->sweep_fault = true;
                            zdt->sweep_fault_reason =
                                ZDT_SWEEP_FAULT_TIMEOUT;
                        }
                    }
                } else if (sweep_initialized &&
                           zdt->sweep_test_active &&
                           ((int32_t)(now - sweep_next_action_tick) >= 0)) {
                    zdt->motion_result = app_zdt_uart_move_to_angle(
                        &zdt->uart_motor,
                        zdt->sweep_target_degrees,
                        APP_ZDT_SWEEP_SPEED_RPM,
                        APP_ZDT_SWEEP_ACCELERATION);

                    if (zdt->motion_result == ZDT_RESULT_OK) {
                        zdt->sweep_motion_active = true;
                        sweep_last_angle_sample =
                            zdt->uart_motor.feedback.angle_sample_count;
                        sweep_next_poll_tick = now +
                            pdMS_TO_TICKS(APP_ZDT_SWEEP_POLL_MS);
                        sweep_deadline_tick = now +
                            pdMS_TO_TICKS(APP_ZDT_SWEEP_TIMEOUT_MS);
                    } else if (zdt->motion_result !=
                               ZDT_RESULT_FEEDBACK_NOT_VALID) {
                        zdt->sweep_test_active = false;
                        zdt->sweep_fault = true;
                        zdt->sweep_fault_reason = ZDT_SWEEP_FAULT_COMMAND;
                    }
                }
            }
        }
#endif

        if (zdt->position_waiting_feedback &&
            (zdt->uart_motor.feedback.angle_sample_count !=
             position_feedback_sample)) {
            zdt->position_waiting_feedback = false;
        } else if (zdt->position_waiting_feedback &&
                   ((int32_t)(now - position_feedback_deadline_tick) >= 0) &&
                   (zdt->uart_motor.angle_feedback_requests ==
                    zdt->uart_motor.angle_feedback_queries)) {
            /* A lost post-motion response must not block every later target. */
            zdt->poll_result = app_zdt_uart_request_angle_once(
                &zdt->uart_motor);
            if (zdt->poll_result == ZDT_RESULT_OK) {
                zdt->position_feedback_retries++;
            }
            position_feedback_deadline_tick = now +
                pdMS_TO_TICKS(APP_ZDT_UART_FEEDBACK_TIMEOUT_MS);
        }

        if (zdt->position_control_enabled &&
            !zdt->position_waiting_feedback &&
            zdt->uart_motor.feedback.encoder_absolute_valid &&
            (zdt->position_command_sequence !=
             zdt->position_applied_sequence) &&
            (zdt->uart_motor.angle_feedback_requests ==
             zdt->uart_motor.angle_feedback_queries)) {
            uint32_t requested_sequence_before;
            uint32_t requested_sequence_after;
            float requested_target;

            requested_sequence_before = zdt->position_command_sequence;
            requested_target = zdt->position_target_degrees;
            requested_sequence_after = zdt->position_command_sequence;

            /* Retry next pass if the chassis task updated the pair while it
             * was being read. This keeps target and sequence coherent. */
            if (requested_sequence_before == requested_sequence_after) {
                zdt->motion_result = app_zdt_uart_move_to_angle(
                    &zdt->uart_motor,
                    requested_target,
                    APP_ZDT_SWEEP_SPEED_RPM,
                    APP_ZDT_SWEEP_ACCELERATION);

                if (zdt->motion_result == ZDT_RESULT_OK) {
                    zdt->position_applied_sequence =
                        requested_sequence_before;
                    if (zdt->uart_motor.angle_limit.last_command_pulses != 0U) {
                        /* Count only a real 0xFD motion frame, not a no-op. */
                        zdt->position_commands_sent++;
                        position_feedback_sample =
                            zdt->uart_motor.feedback.angle_sample_count;
                        zdt->position_waiting_feedback = true;
                        position_feedback_deadline_tick = now +
                            pdMS_TO_TICKS(APP_ZDT_UART_FEEDBACK_TIMEOUT_MS);
                    } else {
                        zdt->position_noop_updates++;
                    }
                } else if (zdt->motion_result !=
                           ZDT_RESULT_FEEDBACK_NOT_VALID) {
                    zdt->position_command_errors++;
                }
            }
        }

        /* Start a short delay after each newly queued control/request event. */
        if ((!feedback_delay_active) &&
            (zdt->uart_motor.angle_feedback_queries !=
             zdt->uart_motor.angle_feedback_requests)) {
            feedback_due_tick = now +
                pdMS_TO_TICKS(APP_ZDT_UART_FEEDBACK_DELAY_MS);
            feedback_delay_active = true;
        }

        /* Exactly one 0x31 angle query is issued for each queued event. */
        if (feedback_delay_active &&
            ((int32_t)(now - feedback_due_tick) >= 0)) {
            zdt->poll_result =
                bsp_zdt_uart_read_encoder_absolute(&zdt->uart_motor);
            if (zdt->poll_result == ZDT_RESULT_OK) {
                zdt->uart_motor.angle_feedback_queries++;
            }
            feedback_delay_active = false;
        }

        vTaskDelay(pdMS_TO_TICKS(APP_ZDT_UART_SERVICE_PERIOD_MS));
#endif
    }
}

zdt_result_t app_zdt_pulse_init(zdt_motor_t *motor)
{
    return bsp_zdt_init(motor, &g_zdt_config);
}

zdt_result_t app_zdt_enable(zdt_motor_t *motor, bool enable)
{
    return bsp_zdt_enable(motor, enable);
}

zdt_result_t app_zdt_move_pulses(zdt_motor_t *motor,
                                 zdt_direction_t direction,
                                 uint32_t frequency_hz,
                                 uint32_t pulses)
{
    zdt_result_t result;

    result = bsp_zdt_set_direction(motor, direction);
    if (result != ZDT_RESULT_OK) {
        return result;
    }

    return bsp_zdt_start_pulses(motor, frequency_hz, pulses);
}

zdt_result_t app_zdt_run_continuous(zdt_motor_t *motor,
                                    zdt_direction_t direction,
                                    uint32_t frequency_hz)
{
    zdt_result_t result;

    result = bsp_zdt_set_direction(motor, direction);
    if (result != ZDT_RESULT_OK) {
        return result;
    }

    return bsp_zdt_start_continuous(motor, frequency_hz);
}

void app_zdt_stop(zdt_motor_t *motor)
{
    bsp_zdt_stop(motor);
}

zdt_result_t app_zdt_uart_init(zdt_uart_t *motor)
{
    zdt_result_t result;

    /* SysConfig owns UART2/PB15/PB16; this enables its RX interrupt buffer. */
    bsp_uart_port2_init();
    result = bsp_zdt_uart_init(motor, &g_zdt_uart_config);
    if (result != ZDT_RESULT_OK) {
        return result;
    }

    motor->angle_limit.enabled = true;
    motor->angle_limit.lower_degrees = APP_ZDT_ANGLE_LOWER_DEG;
    motor->angle_limit.upper_degrees = APP_ZDT_ANGLE_UPPER_DEG;
    motor->angle_limit.pulses_per_revolution =
        APP_ZDT_UART_PULSES_PER_REVOLUTION;
    motor->angle_limit.requested_target_degrees = APP_ZDT_ANGLE_LOWER_DEG;
    motor->angle_limit.limited_target_degrees = APP_ZDT_ANGLE_LOWER_DEG;
    return ZDT_RESULT_OK;
}

zdt_result_t app_zdt_uart_enable(zdt_uart_t *motor, bool enable)
{
    zdt_result_t result = bsp_zdt_uart_enable(motor, enable, false);

    if (result == ZDT_RESULT_OK) {
        (void)app_zdt_queue_angle_feedback(motor);
    }
    return result;
}

zdt_result_t app_zdt_uart_move(zdt_uart_t *motor,
                               zdt_direction_t direction,
                               uint16_t speed_rpm,
                               uint8_t acceleration,
                               uint32_t pulses)
{
    float requested_target;
    float requested_delta;

    if (motor == NULL) {
        return ZDT_RESULT_NULL;
    }
    if (!motor->initialized) {
        return ZDT_RESULT_NOT_INITIALIZED;
    }
    if ((direction != ZDT_DIR_CW) && (direction != ZDT_DIR_CCW)) {
        return ZDT_RESULT_INVALID_CONFIG;
    }
    if (pulses == 0U) {
        return ZDT_RESULT_INVALID_PULSE_COUNT;
    }
    if (!app_zdt_uart_angle_limit_is_valid(motor)) {
        return ZDT_RESULT_INVALID_ANGLE_LIMIT;
    }
    if (!motor->feedback.encoder_absolute_valid) {
        (void)app_zdt_queue_angle_feedback(motor);
        return ZDT_RESULT_FEEDBACK_NOT_VALID;
    }

    requested_delta = (float)pulses * 360.0f /
                      (float)motor->angle_limit.pulses_per_revolution;
    if (direction != APP_ZDT_ENCODER_INCREASE_DIRECTION) {
        requested_delta = -requested_delta;
    }
    requested_target = motor->feedback.encoder_absolute_degrees +
                       requested_delta;

    return app_zdt_uart_move_to_angle(motor,
                                      requested_target,
                                      speed_rpm,
                                      acceleration);
}

zdt_result_t app_zdt_uart_move_to_angle(zdt_uart_t *motor,
                                        float target_degrees,
                                        uint16_t speed_rpm,
                                        uint8_t acceleration)
{
    zdt_angle_limit_t *limit;
    float current_degrees;
    float limited_target;
    float delta_degrees;
    float absolute_delta;
    uint32_t pulses;
    bool target_was_clamped = false;
    zdt_result_t result;

    if (motor == NULL) {
        return ZDT_RESULT_NULL;
    }
    if (!motor->initialized) {
        return ZDT_RESULT_NOT_INITIALIZED;
    }
    if (!app_zdt_uart_angle_limit_is_valid(motor)) {
        return ZDT_RESULT_INVALID_ANGLE_LIMIT;
    }
    if (!motor->feedback.encoder_absolute_valid) {
        (void)app_zdt_queue_angle_feedback(motor);
        return ZDT_RESULT_FEEDBACK_NOT_VALID;
    }

    limit = &motor->angle_limit;
    current_degrees = motor->feedback.encoder_absolute_degrees;
    limit->requested_target_degrees = target_degrees;
    limit->feedback_outside_range =
        (current_degrees <
         (limit->lower_degrees - APP_ZDT_FEEDBACK_TOLERANCE_DEG)) ||
        (current_degrees >
         (limit->upper_degrees + APP_ZDT_FEEDBACK_TOLERANCE_DEG));
    limit->lower_active =
        current_degrees <= (limit->lower_degrees + APP_ZDT_ANGLE_EPSILON_DEG);
    limit->upper_active =
        current_degrees >= (limit->upper_degrees - APP_ZDT_ANGLE_EPSILON_DEG);

    /* Never attempt an automatic recovery through a mechanically forbidden arc. */
    if (limit->feedback_outside_range) {
        limit->last_delta_degrees = 0.0f;
        limit->last_command_pulses = 0U;
        limit->blocked_events++;
        return ZDT_RESULT_ANGLE_LIMIT;
    }

    /* Remove only encoder quantization error; commanded limits stay exact. */
    if (current_degrees < limit->lower_degrees) {
        current_degrees = limit->lower_degrees;
    } else if (current_degrees > limit->upper_degrees) {
        current_degrees = limit->upper_degrees;
    }

    limited_target = target_degrees;
    if (limited_target < limit->lower_degrees) {
        limited_target = limit->lower_degrees;
        target_was_clamped = true;
    } else if (limited_target > limit->upper_degrees) {
        limited_target = limit->upper_degrees;
        target_was_clamped = true;
    }
    if (target_was_clamped) {
        limit->clamp_events++;
    }
    limit->limited_target_degrees = limited_target;

    /*
     * Both endpoints are inside one 95-degree safe interval. Subtracting
     * directly forces 240->335 to +95 and 335->240 to -95; it can never
     * choose the alternative 265-degree path through the forbidden region.
     */
    delta_degrees = limited_target - current_degrees;
    limit->last_delta_degrees = delta_degrees;
    absolute_delta = (delta_degrees < 0.0f)
                         ? -delta_degrees
                         : delta_degrees;

    if (absolute_delta <= APP_ZDT_ANGLE_EPSILON_DEG) {
        limit->last_command_pulses = 0U;
        if (target_was_clamped || limit->lower_active || limit->upper_active) {
            limit->blocked_events++;
            return ZDT_RESULT_ANGLE_LIMIT;
        }
        return ZDT_RESULT_OK;
    }

    pulses = (uint32_t)((absolute_delta *
                         (float)limit->pulses_per_revolution / 360.0f) +
                        0.5f);
    if (pulses == 0U) {
        limit->last_command_pulses = 0U;
        return ZDT_RESULT_OK;
    }
    limit->last_command_pulses = pulses;

    result = bsp_zdt_uart_position(
        motor,
        app_zdt_direction_from_delta(delta_degrees),
        speed_rpm,
        acceleration,
        pulses,
        ZDT_POSITION_RELATIVE_CURRENT,
        false);

    if (result == ZDT_RESULT_OK) {
        (void)app_zdt_queue_angle_feedback(motor);
    }
    return result;
}

zdt_result_t app_zdt_uart_run(zdt_uart_t *motor,
                              zdt_direction_t direction,
                              uint16_t speed_rpm,
                              uint8_t acceleration)
{
    zdt_result_t result;

    if (motor == NULL) {
        return ZDT_RESULT_NULL;
    }
    if (!motor->initialized) {
        return ZDT_RESULT_NOT_INITIALIZED;
    }
    if (motor->angle_limit.enabled) {
        motor->angle_limit.blocked_events++;
        return ZDT_RESULT_ANGLE_LIMIT;
    }

    result = bsp_zdt_uart_velocity(motor,
                                   direction,
                                   speed_rpm,
                                   acceleration,
                                   false);

    if (result == ZDT_RESULT_OK) {
        (void)app_zdt_queue_angle_feedback(motor);
    }
    return result;
}

zdt_result_t app_zdt_uart_stop(zdt_uart_t *motor)
{
    zdt_result_t result = bsp_zdt_uart_stop(motor, false);

    if (result == ZDT_RESULT_OK) {
        (void)app_zdt_queue_angle_feedback(motor);
    }
    return result;
}

zdt_result_t app_zdt_uart_request_angle_once(zdt_uart_t *motor)
{
    return app_zdt_queue_angle_feedback(motor);
}
