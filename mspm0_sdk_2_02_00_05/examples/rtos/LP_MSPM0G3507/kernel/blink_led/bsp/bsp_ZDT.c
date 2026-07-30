#include "bsp_ZDT.h"
#include "bsp_uart.h"

#include <limits.h>
#include <string.h>

/* TIMG12 is dedicated to one ZDT pulse output in this project. */
static zdt_motor_t *g_zdt_isr_motor = NULL;

static void zdt_write_level(GPIO_Regs *port, uint32_t pin, bool high)
{
    if (high) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
}

static void zdt_write_step_idle(zdt_motor_t *motor)
{
    zdt_write_level(motor->config.step_port,
                    motor->config.step_pin,
                    !motor->config.step_active_high);
    motor->step_active = false;
}

static void zdt_write_step_active(zdt_motor_t *motor)
{
    zdt_write_level(motor->config.step_port,
                    motor->config.step_pin,
                    motor->config.step_active_high);
    motor->step_active = true;
}

static bool zdt_config_is_valid(const zdt_config_t *config)
{
    if ((config == NULL) ||
        (config->step_port == NULL) || (config->step_pin == 0U) ||
        (config->dir_port == NULL) || (config->dir_pin == 0U) ||
        (config->enable_port == NULL) || (config->enable_pin == 0U) ||
        (config->timer == NULL) || (config->timer_clock_hz == 0U) ||
        (config->microsteps == 0U) ||
        (config->motor_step_angle_deg <= 0.0f)) {
        return false;
    }

    return true;
}

static zdt_result_t zdt_prepare_timer(zdt_motor_t *motor,
                                      uint32_t frequency_hz)
{
    uint64_t denominator;
    uint64_t half_period_ticks;

    if ((frequency_hz == 0U) ||
        (frequency_hz > BSP_ZDT_MAX_PULSE_FREQUENCY_HZ)) {
        return ZDT_RESULT_INVALID_FREQUENCY;
    }

    denominator = (uint64_t)frequency_hz * 2ULL;
    half_period_ticks =
        ((uint64_t)motor->config.timer_clock_hz + (denominator / 2ULL)) /
        denominator;

    if ((half_period_ticks == 0ULL) ||
        (half_period_ticks > ((uint64_t)UINT32_MAX + 1ULL))) {
        return ZDT_RESULT_INVALID_FREQUENCY;
    }

    DL_TimerG_setLoadValue(motor->config.timer,
                           (uint32_t)(half_period_ticks - 1ULL));
    DL_TimerG_setTimerCount(motor->config.timer,
                            (uint32_t)(half_period_ticks - 1ULL));

    return ZDT_RESULT_OK;
}

static zdt_result_t zdt_start(zdt_motor_t *motor,
                              uint32_t frequency_hz,
                              uint32_t pulses,
                              bool continuous)
{
    zdt_result_t result;

    if (motor == NULL) {
        return ZDT_RESULT_NULL;
    }
    if (!motor->initialized) {
        return ZDT_RESULT_NOT_INITIALIZED;
    }
    if (!motor->enabled) {
        return ZDT_RESULT_DISABLED;
    }
    if (motor->state == ZDT_STATE_RUNNING) {
        return ZDT_RESULT_BUSY;
    }
    if ((!continuous) && (pulses == 0U)) {
        return ZDT_RESULT_INVALID_PULSE_COUNT;
    }

    NVIC_DisableIRQ(motor->config.timer_irqn);
    DL_TimerG_stopCounter(motor->config.timer);
    zdt_write_step_idle(motor);

    result = zdt_prepare_timer(motor, frequency_hz);
    if (result != ZDT_RESULT_OK) {
        NVIC_ClearPendingIRQ(motor->config.timer_irqn);
        NVIC_EnableIRQ(motor->config.timer_irqn);
        return result;
    }

    motor->target_pulses = pulses;
    motor->emitted_pulses = 0U;
    motor->pulse_frequency_hz = frequency_hz;
    motor->continuous = continuous;
    motor->state = ZDT_STATE_RUNNING;

    DL_TimerG_clearInterruptStatus(motor->config.timer,
                                   DL_TIMERG_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(motor->config.timer_irqn);
    NVIC_EnableIRQ(motor->config.timer_irqn);
    DL_TimerG_startCounter(motor->config.timer);

    return ZDT_RESULT_OK;
}

zdt_result_t bsp_zdt_init(zdt_motor_t *motor, const zdt_config_t *config)
{
    if (motor == NULL) {
        return ZDT_RESULT_NULL;
    }

    memset(motor, 0, sizeof(*motor));

    if (!zdt_config_is_valid(config)) {
        motor->state = ZDT_STATE_ERROR;
        return ZDT_RESULT_INVALID_CONFIG;
    }

    motor->config = *config;
    motor->direction = ZDT_DIR_CW;
    motor->state = ZDT_STATE_DISABLED;

    NVIC_DisableIRQ(motor->config.timer_irqn);
    DL_TimerG_stopCounter(motor->config.timer);

    /* Set inactive levels before changing the three pins from Hi-Z to output. */
    zdt_write_step_idle(motor);
    zdt_write_level(motor->config.dir_port,
                    motor->config.dir_pin,
                    motor->config.dir_cw_high);
    zdt_write_level(motor->config.enable_port,
                    motor->config.enable_pin,
                    !motor->config.enable_active_high);

    DL_GPIO_enableOutput(motor->config.step_port, motor->config.step_pin);
    DL_GPIO_enableOutput(motor->config.dir_port, motor->config.dir_pin);
    DL_GPIO_enableOutput(motor->config.enable_port, motor->config.enable_pin);

    DL_TimerG_clearInterruptStatus(motor->config.timer,
                                   DL_TIMERG_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(motor->config.timer_irqn);

    g_zdt_isr_motor = motor;
    motor->initialized = true;
    NVIC_EnableIRQ(motor->config.timer_irqn);

    return ZDT_RESULT_OK;
}

zdt_result_t bsp_zdt_enable(zdt_motor_t *motor, bool enable)
{
    if (motor == NULL) {
        return ZDT_RESULT_NULL;
    }
    if (!motor->initialized) {
        return ZDT_RESULT_NOT_INITIALIZED;
    }

    if (!enable) {
        bsp_zdt_stop(motor);
    }

    zdt_write_level(motor->config.enable_port,
                    motor->config.enable_pin,
                    enable ? motor->config.enable_active_high
                           : !motor->config.enable_active_high);
    motor->enabled = enable;
    motor->state = enable ? ZDT_STATE_IDLE : ZDT_STATE_DISABLED;

    return ZDT_RESULT_OK;
}

zdt_result_t bsp_zdt_set_direction(zdt_motor_t *motor,
                                  zdt_direction_t direction)
{
    bool output_high;

    if (motor == NULL) {
        return ZDT_RESULT_NULL;
    }
    if (!motor->initialized) {
        return ZDT_RESULT_NOT_INITIALIZED;
    }
    if ((direction != ZDT_DIR_CW) && (direction != ZDT_DIR_CCW)) {
        return ZDT_RESULT_INVALID_CONFIG;
    }
    if (motor->state == ZDT_STATE_RUNNING) {
        return ZDT_RESULT_BUSY;
    }

    output_high = (direction == ZDT_DIR_CW)
                      ? motor->config.dir_cw_high
                      : !motor->config.dir_cw_high;
    zdt_write_level(motor->config.dir_port,
                    motor->config.dir_pin,
                    output_high);
    motor->direction = direction;

    return ZDT_RESULT_OK;
}

zdt_result_t bsp_zdt_start_pulses(zdt_motor_t *motor,
                                  uint32_t frequency_hz,
                                  uint32_t pulses)
{
    return zdt_start(motor, frequency_hz, pulses, false);
}

zdt_result_t bsp_zdt_start_continuous(zdt_motor_t *motor,
                                      uint32_t frequency_hz)
{
    return zdt_start(motor, frequency_hz, 0U, true);
}

void bsp_zdt_stop(zdt_motor_t *motor)
{
    if ((motor == NULL) || (!motor->initialized)) {
        return;
    }

    NVIC_DisableIRQ(motor->config.timer_irqn);
    DL_TimerG_stopCounter(motor->config.timer);
    zdt_write_step_idle(motor);
    DL_TimerG_clearInterruptStatus(motor->config.timer,
                                   DL_TIMERG_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(motor->config.timer_irqn);

    motor->pulse_frequency_hz = 0U;
    motor->continuous = false;
    motor->state = motor->enabled ? ZDT_STATE_IDLE : ZDT_STATE_DISABLED;
    NVIC_EnableIRQ(motor->config.timer_irqn);
}

bool bsp_zdt_is_busy(const zdt_motor_t *motor)
{
    return (motor != NULL) && (motor->state == ZDT_STATE_RUNNING);
}

uint32_t bsp_zdt_pulses_per_revolution(const zdt_motor_t *motor)
{
    float pulses;

    if ((motor == NULL) || (!motor->initialized) ||
        (motor->config.motor_step_angle_deg <= 0.0f)) {
        return 0U;
    }

    pulses = (360.0f / motor->config.motor_step_angle_deg) *
             (float)motor->config.microsteps;
    if (pulses >= (float)UINT32_MAX) {
        return UINT32_MAX;
    }

    return (uint32_t)(pulses + 0.5f);
}

uint32_t bsp_zdt_degrees_to_pulses(const zdt_motor_t *motor,
                                   float degrees)
{
    float pulses;
    uint32_t pulses_per_revolution;

    if (degrees < 0.0f) {
        degrees = -degrees;
    }

    pulses_per_revolution = bsp_zdt_pulses_per_revolution(motor);
    if (pulses_per_revolution == 0U) {
        return 0U;
    }

    pulses = (degrees / 360.0f) * (float)pulses_per_revolution;
    if (pulses >= (float)UINT32_MAX) {
        return UINT32_MAX;
    }

    return (uint32_t)(pulses + 0.5f);
}

uint32_t bsp_zdt_revolutions_to_pulses(const zdt_motor_t *motor,
                                       float revolutions)
{
    float pulses;
    uint32_t pulses_per_revolution;

    if (revolutions < 0.0f) {
        revolutions = -revolutions;
    }

    pulses_per_revolution = bsp_zdt_pulses_per_revolution(motor);
    if (pulses_per_revolution == 0U) {
        return 0U;
    }

    pulses = revolutions * (float)pulses_per_revolution;
    if (pulses >= (float)UINT32_MAX) {
        return UINT32_MAX;
    }

    return (uint32_t)(pulses + 0.5f);
}

uint32_t bsp_zdt_rpm_to_frequency(const zdt_motor_t *motor, float rpm)
{
    float frequency;
    uint32_t pulses_per_revolution;

    if (rpm < 0.0f) {
        rpm = -rpm;
    }

    pulses_per_revolution = bsp_zdt_pulses_per_revolution(motor);
    if (pulses_per_revolution == 0U) {
        return 0U;
    }

    frequency = (rpm * (float)pulses_per_revolution) / 60.0f;
    if (frequency >= (float)UINT32_MAX) {
        return UINT32_MAX;
    }

    return (uint32_t)(frequency + 0.5f);
}

static bool zdt_uart_config_is_valid(const zdt_uart_config_t *config)
{
    return (config != NULL) &&
           (config->uart != NULL) &&
           (config->address != 0U);
}

static bool zdt_direction_is_valid(zdt_direction_t direction)
{
    return (direction == ZDT_DIR_CW) || (direction == ZDT_DIR_CCW);
}

static uint8_t zdt_uart_response_length(uint8_t function)
{
    switch (function) {
        case 0x31U: /* Calibrated single-turn absolute encoder value. */
            return 5U;
        case 0x35U: /* Actual speed. */
            return 6U;
        case 0x36U: /* Actual position. */
            return 8U;
        case 0x3AU: /* Motor status flags. */
            return 4U;

        /* Control-command acknowledgement/action-complete frames. */
        case 0xF3U:
        case 0xF5U:
        case 0xF6U:
        case 0xFBU:
        case 0xFCU:
        case 0xFDU:
        case 0xFEU:
        case 0xFFU:
        case 0x9AU:
            return 4U;
        default:
            return 0U;
    }
}

static void zdt_uart_reset_parser(zdt_uart_t *motor)
{
    motor->rx_index = 0U;
    motor->rx_expected_length = 0U;
}

static bool zdt_uart_parse_frame(zdt_uart_t *motor)
{
    const uint8_t *frame = motor->rx_frame;
    zdt_uart_feedback_t *feedback = &motor->feedback;

    if ((frame[0] != motor->config.address) ||
        (frame[motor->rx_expected_length - 1U] != motor->config.checksum)) {
        return false;
    }

    switch (frame[1]) {
        case 0x31U: {
            uint16_t encoder_raw;
            float angle_degrees;

            encoder_raw = ((uint16_t)frame[2] << 8) | frame[3];
            angle_degrees = (float)encoder_raw * (360.0f / 65536.0f);
            feedback->encoder_absolute_raw = encoder_raw;
            feedback->encoder_absolute_degrees = angle_degrees;

            if (!feedback->observed_angle_range_valid) {
                feedback->observed_min_angle_degrees = angle_degrees;
                feedback->observed_max_angle_degrees = angle_degrees;
                feedback->observed_angle_range_valid = true;
            } else {
                if (angle_degrees < feedback->observed_min_angle_degrees) {
                    feedback->observed_min_angle_degrees = angle_degrees;
                }
                if (angle_degrees > feedback->observed_max_angle_degrees) {
                    feedback->observed_max_angle_degrees = angle_degrees;
                }
            }
            feedback->angle_sample_count++;
            feedback->encoder_absolute_valid = true;
            break;
        }

        case 0x35U: {
            uint16_t magnitude;

            if (frame[2] > 1U) {
                return false;
            }
            magnitude = ((uint16_t)frame[3] << 8) | frame[4];
            feedback->actual_speed_rpm = (frame[2] == 0U)
                                             ? (int16_t)magnitude
                                             : -(int16_t)magnitude;
            feedback->speed_valid = true;
            break;
        }

        case 0x36U: {
            uint32_t magnitude;
            int64_t signed_position;

            if (frame[2] > 1U) {
                return false;
            }
            magnitude = ((uint32_t)frame[3] << 24) |
                        ((uint32_t)frame[4] << 16) |
                        ((uint32_t)frame[5] << 8) |
                        (uint32_t)frame[6];
            signed_position = (frame[2] == 0U)
                                  ? (int64_t)magnitude
                                  : -(int64_t)magnitude;
            feedback->actual_position = signed_position;
            feedback->actual_position_degrees =
                (float)signed_position * (360.0f / 65536.0f);
            feedback->position_valid = true;
            break;
        }

        case 0x3AU: {
            uint8_t flags = frame[2];

            feedback->status_flags = flags;
            feedback->motor_enabled =
                (flags & BSP_ZDT_STATUS_ENABLED) != 0U;
            feedback->position_reached =
                (flags & BSP_ZDT_STATUS_POSITION_REACHED) != 0U;
            feedback->stall_detected =
                (flags & BSP_ZDT_STATUS_STALL) != 0U;
            feedback->stall_protection =
                (flags & BSP_ZDT_STATUS_STALL_PROTECTION) != 0U;
            feedback->left_limit_active =
                (flags & BSP_ZDT_STATUS_LEFT_LIMIT) != 0U;
            feedback->right_limit_active =
                (flags & BSP_ZDT_STATUS_RIGHT_LIMIT) != 0U;
            feedback->power_on_latch =
                (flags & BSP_ZDT_STATUS_POWER_ON_LATCH) != 0U;
            feedback->status_valid = true;
            break;
        }

        default:
            feedback->last_ack_function = frame[1];
            feedback->last_ack_status = frame[2];
            feedback->ack_valid = true;
            feedback->ack_frames++;
            break;
    }

    feedback->received_frames++;
    return true;
}

zdt_result_t bsp_zdt_uart_init(zdt_uart_t *motor,
                               const zdt_uart_config_t *config)
{
    if (motor == NULL) {
        return ZDT_RESULT_NULL;
    }

    memset(motor, 0, sizeof(*motor));
    if (!zdt_uart_config_is_valid(config)) {
        return ZDT_RESULT_INVALID_CONFIG;
    }

    motor->config = *config;
    motor->initialized = true;
    return ZDT_RESULT_OK;
}

zdt_result_t bsp_zdt_uart_send_raw(zdt_uart_t *motor,
                                   const uint8_t *command,
                                   uint16_t length)
{
    uint16_t i;

    if ((motor == NULL) || (command == NULL)) {
        return ZDT_RESULT_NULL;
    }
    if (!motor->initialized) {
        return ZDT_RESULT_NOT_INITIALIZED;
    }
    if ((length < 3U) || (length > BSP_ZDT_UART_MAX_COMMAND_LENGTH)) {
        return ZDT_RESULT_INVALID_COMMAND_LENGTH;
    }

    for (i = 0U; i < length; i++) {
        DL_UART_Main_transmitDataBlocking(motor->config.uart, command[i]);
    }
    while (DL_UART_isBusy(motor->config.uart)) {
    }

    motor->last_function = command[1];
    motor->transmitted_frames++;
    motor->transmitted_bytes += length;
    return ZDT_RESULT_OK;
}

zdt_result_t bsp_zdt_uart_enable(zdt_uart_t *motor,
                                 bool enable,
                                 bool synchronous)
{
    uint8_t command[6];

    if (motor == NULL) {
        return ZDT_RESULT_NULL;
    }
    if (!motor->initialized) {
        return ZDT_RESULT_NOT_INITIALIZED;
    }

    command[0] = motor->config.address;
    command[1] = 0xF3U;
    command[2] = 0xABU;
    command[3] = enable ? 1U : 0U;
    command[4] = synchronous ? 1U : 0U;
    command[5] = motor->config.checksum;
    return bsp_zdt_uart_send_raw(motor, command, sizeof(command));
}

zdt_result_t bsp_zdt_uart_velocity(zdt_uart_t *motor,
                                   zdt_direction_t direction,
                                   uint16_t speed_rpm,
                                   uint8_t acceleration,
                                   bool synchronous)
{
    uint8_t command[8];

    if (motor == NULL) {
        return ZDT_RESULT_NULL;
    }
    if (!motor->initialized) {
        return ZDT_RESULT_NOT_INITIALIZED;
    }
    if (!zdt_direction_is_valid(direction)) {
        return ZDT_RESULT_INVALID_CONFIG;
    }
    if (speed_rpm > BSP_ZDT_UART_MAX_SPEED_RPM) {
        return ZDT_RESULT_INVALID_SPEED;
    }

    command[0] = motor->config.address;
    command[1] = 0xF6U;
    command[2] = (direction == ZDT_DIR_CW) ? 0U : 1U;
    command[3] = (uint8_t)(speed_rpm >> 8);
    command[4] = (uint8_t)speed_rpm;
    command[5] = acceleration;
    command[6] = synchronous ? 1U : 0U;
    command[7] = motor->config.checksum;
    return bsp_zdt_uart_send_raw(motor, command, sizeof(command));
}

zdt_result_t bsp_zdt_uart_position(zdt_uart_t *motor,
                                   zdt_direction_t direction,
                                   uint16_t speed_rpm,
                                   uint8_t acceleration,
                                   uint32_t pulses,
                                   zdt_position_reference_t reference,
                                   bool synchronous)
{
    uint8_t command[13];

    if (motor == NULL) {
        return ZDT_RESULT_NULL;
    }
    if (!motor->initialized) {
        return ZDT_RESULT_NOT_INITIALIZED;
    }
    if (!zdt_direction_is_valid(direction)) {
        return ZDT_RESULT_INVALID_CONFIG;
    }
    if (speed_rpm > BSP_ZDT_UART_MAX_SPEED_RPM) {
        return ZDT_RESULT_INVALID_SPEED;
    }
    if ((reference != ZDT_POSITION_RELATIVE_TARGET) &&
        (reference != ZDT_POSITION_ABSOLUTE) &&
        (reference != ZDT_POSITION_RELATIVE_CURRENT)) {
        return ZDT_RESULT_INVALID_POSITION_REFERENCE;
    }
    if (pulses == 0U) {
        return ZDT_RESULT_INVALID_PULSE_COUNT;
    }

    command[0] = motor->config.address;
    command[1] = 0xFDU;
    command[2] = (direction == ZDT_DIR_CW) ? 0U : 1U;
    command[3] = (uint8_t)(speed_rpm >> 8);
    command[4] = (uint8_t)speed_rpm;
    command[5] = acceleration;
    command[6] = (uint8_t)(pulses >> 24);
    command[7] = (uint8_t)(pulses >> 16);
    command[8] = (uint8_t)(pulses >> 8);
    command[9] = (uint8_t)pulses;
    command[10] = (uint8_t)reference;
    command[11] = synchronous ? 1U : 0U;
    command[12] = motor->config.checksum;
    return bsp_zdt_uart_send_raw(motor, command, sizeof(command));
}

zdt_result_t bsp_zdt_uart_quick_position(zdt_uart_t *motor,
                                         int32_t signed_pulses)
{
    uint8_t command[7];
    uint32_t pulses = (uint32_t)signed_pulses;

    if (motor == NULL) {
        return ZDT_RESULT_NULL;
    }
    if (!motor->initialized) {
        return ZDT_RESULT_NOT_INITIALIZED;
    }
    if (signed_pulses == 0) {
        return ZDT_RESULT_INVALID_PULSE_COUNT;
    }

    command[0] = motor->config.address;
    command[1] = 0xFCU;
    command[2] = (uint8_t)(pulses >> 24);
    command[3] = (uint8_t)(pulses >> 16);
    command[4] = (uint8_t)(pulses >> 8);
    command[5] = (uint8_t)pulses;
    command[6] = motor->config.checksum;
    return bsp_zdt_uart_send_raw(motor, command, sizeof(command));
}

zdt_result_t bsp_zdt_uart_stop(zdt_uart_t *motor, bool synchronous)
{
    uint8_t command[5];

    if (motor == NULL) {
        return ZDT_RESULT_NULL;
    }
    if (!motor->initialized) {
        return ZDT_RESULT_NOT_INITIALIZED;
    }

    command[0] = motor->config.address;
    command[1] = 0xFEU;
    command[2] = 0x98U;
    command[3] = synchronous ? 1U : 0U;
    command[4] = motor->config.checksum;
    return bsp_zdt_uart_send_raw(motor, command, sizeof(command));
}

zdt_result_t bsp_zdt_uart_synchronous_start(zdt_uart_t *motor)
{
    uint8_t command[4];

    if (motor == NULL) {
        return ZDT_RESULT_NULL;
    }
    if (!motor->initialized) {
        return ZDT_RESULT_NOT_INITIALIZED;
    }

    command[0] = motor->config.address;
    command[1] = 0xFFU;
    command[2] = 0x66U;
    command[3] = motor->config.checksum;
    return bsp_zdt_uart_send_raw(motor, command, sizeof(command));
}

static zdt_result_t zdt_uart_read_parameter(zdt_uart_t *motor,
                                            uint8_t function)
{
    uint8_t command[3];

    if (motor == NULL) {
        return ZDT_RESULT_NULL;
    }
    if (!motor->initialized) {
        return ZDT_RESULT_NOT_INITIALIZED;
    }

    command[0] = motor->config.address;
    command[1] = function;
    command[2] = motor->config.checksum;
    return bsp_zdt_uart_send_raw(motor, command, sizeof(command));
}

zdt_result_t bsp_zdt_uart_read_speed(zdt_uart_t *motor)
{
    return zdt_uart_read_parameter(motor, 0x35U);
}

zdt_result_t bsp_zdt_uart_read_position(zdt_uart_t *motor)
{
    return zdt_uart_read_parameter(motor, 0x36U);
}

zdt_result_t bsp_zdt_uart_read_encoder_absolute(zdt_uart_t *motor)
{
    return zdt_uart_read_parameter(motor, 0x31U);
}

zdt_result_t bsp_zdt_uart_read_status(zdt_uart_t *motor)
{
    return zdt_uart_read_parameter(motor, 0x3AU);
}

bool bsp_zdt_uart_parse_byte(zdt_uart_t *motor, uint8_t byte)
{
    bool frame_valid;

    if ((motor == NULL) || (!motor->initialized)) {
        return false;
    }

    motor->feedback.received_bytes++;

    if (motor->rx_index == 0U) {
        if (byte == motor->config.address) {
            motor->rx_frame[0] = byte;
            motor->rx_index = 1U;
        }
        return false;
    }

    if (motor->rx_index == 1U) {
        motor->rx_frame[1] = byte;
        motor->rx_expected_length = zdt_uart_response_length(byte);
        if (motor->rx_expected_length == 0U) {
            motor->feedback.invalid_frames++;
            zdt_uart_reset_parser(motor);
            return false;
        }
        motor->rx_index = 2U;
        return false;
    }

    if (motor->rx_index >= BSP_ZDT_UART_RX_MAX_FRAME_LENGTH) {
        motor->feedback.invalid_frames++;
        zdt_uart_reset_parser(motor);
        return false;
    }

    motor->rx_frame[motor->rx_index++] = byte;
    if (motor->rx_index < motor->rx_expected_length) {
        return false;
    }

    frame_valid = zdt_uart_parse_frame(motor);
    if (!frame_valid) {
        motor->feedback.invalid_frames++;
    }
    zdt_uart_reset_parser(motor);
    return frame_valid;
}

uint32_t bsp_zdt_uart_process_rx(zdt_uart_t *motor)
{
    uint8_t byte;
    uint32_t parsed_frames = 0U;

    if ((motor == NULL) || (!motor->initialized)) {
        return 0U;
    }

    while (bsp_uart2_rxbuf_pop(&byte)) {
        if (bsp_zdt_uart_parse_byte(motor, byte)) {
            parsed_frames++;
        }
    }
    return parsed_frames;
}

void bsp_zdt_uart_reset_observed_angle_range(zdt_uart_t *motor)
{
    if (motor == NULL) {
        return;
    }

    motor->feedback.observed_min_angle_degrees = 0.0f;
    motor->feedback.observed_max_angle_degrees = 0.0f;
    motor->feedback.angle_sample_count = 0U;
    motor->feedback.observed_angle_range_valid = false;
}

void ZDT_PULSE_TIMER_INST_IRQHandler(void)
{
    zdt_motor_t *motor = g_zdt_isr_motor;

    if (DL_TimerG_getPendingInterrupt(ZDT_PULSE_TIMER_INST) !=
        DL_TIMER_IIDX_ZERO) {
        return;
    }

    if ((motor == NULL) || (motor->state != ZDT_STATE_RUNNING)) {
        DL_TimerG_stopCounter(ZDT_PULSE_TIMER_INST);
        return;
    }

    if (!motor->step_active) {
        zdt_write_step_active(motor);
        return;
    }

    zdt_write_step_idle(motor);
    motor->emitted_pulses++;

    if ((!motor->continuous) &&
        (motor->emitted_pulses >= motor->target_pulses)) {
        DL_TimerG_stopCounter(motor->config.timer);
        motor->pulse_frequency_hz = 0U;
        motor->state = motor->enabled ? ZDT_STATE_IDLE : ZDT_STATE_DISABLED;
    }
}
