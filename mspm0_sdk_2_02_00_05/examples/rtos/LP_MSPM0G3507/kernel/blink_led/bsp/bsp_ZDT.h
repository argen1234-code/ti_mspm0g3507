#ifndef BSP_ZDT_H
#define BSP_ZDT_H

#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

/*
 * ZDT_XS STEP/DIR/EN pulse-control and Emm UART-control driver.
 *
 * One timer interrupt represents half of a STEP period. The ISR asserts and
 * releases STEP alternately, so one complete active pulse is counted every two
 * timer interrupts. No FreeRTOS API is called from the timer ISR.
 */

#define BSP_ZDT_DEFAULT_MOTOR_STEP_ANGLE_DEG   (1.8f)
#define BSP_ZDT_DEFAULT_MICROSTEPS             (16U)
#define BSP_ZDT_MAX_PULSE_FREQUENCY_HZ         (20000U)
#define BSP_ZDT_UART_DEFAULT_ADDRESS            (1U)
#define BSP_ZDT_UART_DEFAULT_CHECKSUM           (0x6BU)
#define BSP_ZDT_UART_MAX_SPEED_RPM              (5000U)
#define BSP_ZDT_UART_MAX_COMMAND_LENGTH         (32U)
#define BSP_ZDT_UART_RX_MAX_FRAME_LENGTH         (8U)

/* X42S motor status flags returned by the 0x3A command. */
#define BSP_ZDT_STATUS_ENABLED                   (0x01U)
#define BSP_ZDT_STATUS_POSITION_REACHED          (0x02U)
#define BSP_ZDT_STATUS_STALL                     (0x04U)
#define BSP_ZDT_STATUS_STALL_PROTECTION          (0x08U)
#define BSP_ZDT_STATUS_LEFT_LIMIT                (0x10U)
#define BSP_ZDT_STATUS_RIGHT_LIMIT               (0x20U)
#define BSP_ZDT_STATUS_POWER_ON_LATCH            (0x80U)

/* Common X42S command response values. */
#define BSP_ZDT_ACK_OK                           (0x02U)
#define BSP_ZDT_ACK_HOME_ALREADY_REACHED         (0x12U)
#define BSP_ZDT_ACK_LIMIT_ALREADY_ACTIVE         (0x22U)
#define BSP_ZDT_ACK_PARAMETER_ERROR              (0xE2U)
#define BSP_ZDT_ACK_FORMAT_ERROR                 (0xEEU)
#define BSP_ZDT_ACK_ACTION_COMPLETE              (0x9FU)

typedef enum {
    ZDT_DIR_CW = 0,
    ZDT_DIR_CCW
} zdt_direction_t;

typedef enum {
    ZDT_STATE_DISABLED = 0,
    ZDT_STATE_IDLE,
    ZDT_STATE_RUNNING,
    ZDT_STATE_ERROR
} zdt_state_t;

typedef enum {
    ZDT_RESULT_OK = 0,
    ZDT_RESULT_NULL,
    ZDT_RESULT_INVALID_CONFIG,
    ZDT_RESULT_NOT_INITIALIZED,
    ZDT_RESULT_DISABLED,
    ZDT_RESULT_BUSY,
    ZDT_RESULT_INVALID_FREQUENCY,
    ZDT_RESULT_INVALID_PULSE_COUNT,
    ZDT_RESULT_INVALID_ADDRESS,
    ZDT_RESULT_INVALID_SPEED,
    ZDT_RESULT_INVALID_POSITION_REFERENCE,
    ZDT_RESULT_INVALID_COMMAND_LENGTH,
    ZDT_RESULT_FEEDBACK_NOT_VALID,
    ZDT_RESULT_INVALID_ANGLE_LIMIT,
    ZDT_RESULT_ANGLE_LIMIT
} zdt_result_t;

typedef enum {
    /* Relative to the previous input target position. */
    ZDT_POSITION_RELATIVE_TARGET = 0,
    ZDT_POSITION_ABSOLUTE = 1,
    /* Relative to the motor's current real-time position. */
    ZDT_POSITION_RELATIVE_CURRENT = 2
} zdt_position_reference_t;

typedef struct {
    GPIO_Regs *step_port;
    uint32_t step_pin;
    GPIO_Regs *dir_port;
    uint32_t dir_pin;
    GPIO_Regs *enable_port;
    uint32_t enable_pin;

    GPTIMER_Regs *timer;
    IRQn_Type timer_irqn;
    uint32_t timer_clock_hz;

    /* true means the corresponding MCU output is active/high for that action. */
    bool step_active_high;
    bool dir_cw_high;
    bool enable_active_high;

    uint16_t microsteps;
    float motor_step_angle_deg;
} zdt_config_t;

typedef struct {
    zdt_config_t config;

    volatile zdt_state_t state;
    volatile uint32_t target_pulses;
    volatile uint32_t emitted_pulses;
    volatile uint32_t pulse_frequency_hz;
    volatile bool step_active;
    volatile bool continuous;
    volatile bool enabled;

    zdt_direction_t direction;
    bool initialized;
} zdt_motor_t;

/*
 * X42S Emm-firmware serial interface. The UART peripheral and pins are owned
 * by SysConfig; this structure only owns the motor address and command state.
 */
typedef struct {
    UART_Regs *uart;
    uint8_t address;
    uint8_t checksum;
} zdt_uart_config_t;

typedef struct {
    /* Emm firmware: speed is RPM; position unit is 1/65536 revolution. */
    volatile int16_t actual_speed_rpm;
    volatile int64_t actual_position;
    volatile float actual_position_degrees;
    /* Calibrated single-turn encoder value: 0..65535 maps to 0..<360 deg. */
    volatile uint16_t encoder_absolute_raw;
    volatile float encoder_absolute_degrees;
    volatile float observed_min_angle_degrees;
    volatile float observed_max_angle_degrees;
    volatile uint32_t angle_sample_count;
    volatile uint8_t status_flags;

    volatile bool motor_enabled;
    volatile bool position_reached;
    volatile bool stall_detected;
    volatile bool stall_protection;
    volatile bool left_limit_active;
    volatile bool right_limit_active;
    volatile bool power_on_latch;

    volatile uint8_t last_ack_function;
    volatile uint8_t last_ack_status;
    volatile bool speed_valid;
    volatile bool position_valid;
    volatile bool encoder_absolute_valid;
    volatile bool observed_angle_range_valid;
    volatile bool status_valid;
    volatile bool ack_valid;

    volatile uint32_t received_frames;
    volatile uint32_t received_bytes;
    volatile uint32_t invalid_frames;
    volatile uint32_t ack_frames;
} zdt_uart_feedback_t;

typedef struct {
    bool enabled;
    float lower_degrees;
    float upper_degrees;
    uint32_t pulses_per_revolution;

    volatile float requested_target_degrees;
    volatile float limited_target_degrees;
    volatile float last_delta_degrees;
    volatile uint32_t last_command_pulses;
    volatile bool lower_active;
    volatile bool upper_active;
    volatile bool feedback_outside_range;
    volatile bool emergency_stop_latched;
    volatile uint32_t clamp_events;
    volatile uint32_t blocked_events;
    volatile uint32_t emergency_stop_events;
} zdt_angle_limit_t;

typedef struct {
    zdt_uart_config_t config;
    uint32_t transmitted_frames;
    uint32_t transmitted_bytes;
    uint8_t last_function;
    zdt_uart_feedback_t feedback;
    zdt_angle_limit_t angle_limit;

    /* One angle query is issued for each queued control/request event. */
    volatile uint32_t angle_feedback_requests;
    volatile uint32_t angle_feedback_queries;

    /* Protocol parser state. Frames are assembled by function-code length. */
    uint8_t rx_frame[BSP_ZDT_UART_RX_MAX_FRAME_LENGTH];
    uint8_t rx_index;
    uint8_t rx_expected_length;
    bool initialized;
} zdt_uart_t;

zdt_result_t bsp_zdt_init(zdt_motor_t *motor, const zdt_config_t *config);
zdt_result_t bsp_zdt_enable(zdt_motor_t *motor, bool enable);
zdt_result_t bsp_zdt_set_direction(zdt_motor_t *motor,
                                  zdt_direction_t direction);
zdt_result_t bsp_zdt_start_pulses(zdt_motor_t *motor,
                                  uint32_t frequency_hz,
                                  uint32_t pulses);
zdt_result_t bsp_zdt_start_continuous(zdt_motor_t *motor,
                                      uint32_t frequency_hz);
void bsp_zdt_stop(zdt_motor_t *motor);

bool bsp_zdt_is_busy(const zdt_motor_t *motor);
uint32_t bsp_zdt_pulses_per_revolution(const zdt_motor_t *motor);
uint32_t bsp_zdt_degrees_to_pulses(const zdt_motor_t *motor,
                                   float degrees);
uint32_t bsp_zdt_revolutions_to_pulses(const zdt_motor_t *motor,
                                       float revolutions);
uint32_t bsp_zdt_rpm_to_frequency(const zdt_motor_t *motor, float rpm);

/* X42S Emm UART protocol (default 115200-8-N-1). */
zdt_result_t bsp_zdt_uart_init(zdt_uart_t *motor,
                               const zdt_uart_config_t *config);
zdt_result_t bsp_zdt_uart_send_raw(zdt_uart_t *motor,
                                   const uint8_t *command,
                                   uint16_t length);
zdt_result_t bsp_zdt_uart_enable(zdt_uart_t *motor,
                                 bool enable,
                                 bool synchronous);
zdt_result_t bsp_zdt_uart_velocity(zdt_uart_t *motor,
                                   zdt_direction_t direction,
                                   uint16_t speed_rpm,
                                   uint8_t acceleration,
                                   bool synchronous);
zdt_result_t bsp_zdt_uart_position(zdt_uart_t *motor,
                                   zdt_direction_t direction,
                                   uint16_t speed_rpm,
                                   uint8_t acceleration,
                                   uint32_t pulses,
                                   zdt_position_reference_t reference,
                                   bool synchronous);
zdt_result_t bsp_zdt_uart_quick_position(zdt_uart_t *motor,
                                         int32_t signed_pulses);
zdt_result_t bsp_zdt_uart_stop(zdt_uart_t *motor, bool synchronous);
zdt_result_t bsp_zdt_uart_synchronous_start(zdt_uart_t *motor);
zdt_result_t bsp_zdt_uart_read_speed(zdt_uart_t *motor);
zdt_result_t bsp_zdt_uart_read_position(zdt_uart_t *motor);
zdt_result_t bsp_zdt_uart_read_encoder_absolute(zdt_uart_t *motor);
zdt_result_t bsp_zdt_uart_read_status(zdt_uart_t *motor);

/* UART2 RX service. Call process_rx() periodically from the owning task. */
bool bsp_zdt_uart_parse_byte(zdt_uart_t *motor, uint8_t byte);
uint32_t bsp_zdt_uart_process_rx(zdt_uart_t *motor);
void bsp_zdt_uart_reset_observed_angle_range(zdt_uart_t *motor);

#endif
