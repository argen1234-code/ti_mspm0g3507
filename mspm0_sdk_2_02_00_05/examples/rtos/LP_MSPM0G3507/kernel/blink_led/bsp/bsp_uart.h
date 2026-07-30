#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stdint.h>

/* Schematic mapping, all ports use 115200-8-N-1:
 * UART0 PA10/PA11 (camera), UART1 PA8/PA9, UART2 PB15/PB16,
 * UART3 RX-only PB13 (JY61P/JY901S). */
#define BSP_CAMERA_FRAME_MAX_LENGTH     (48U)
#define BSP_CAMERA_POSITION_MIN_CM      (-12.5f)
#define BSP_CAMERA_POSITION_MAX_CM      (12.5f)

typedef struct {
    volatile float position_cm;
    volatile float velocity_cm_s;
    volatile uint8_t received_checksum;
    volatile uint8_t calculated_checksum;
    volatile bool valid;

    /* Raw UART0 diagnostics, updated even when no valid frame is formed. */
    volatile uint32_t raw_bytes;
    volatile uint32_t rx_irq_count;
    volatile uint8_t last_raw_byte;
    volatile bool uart_enabled;
    volatile bool uart_loopback_enabled;
    volatile uint16_t uart_integer_divisor;
    volatile uint8_t uart_fractional_divisor;

    volatile uint32_t sequence;
    volatile uint32_t received_frames;
    volatile uint32_t valid_frames;
    volatile uint32_t checksum_errors;
    volatile uint32_t format_errors;
    volatile uint32_t range_errors;
    volatile uint32_t frame_overflow_errors;
    volatile uint32_t dropped_bytes;

    char rx_frame[BSP_CAMERA_FRAME_MAX_LENGTH];
    uint8_t rx_length;
    bool receiving;
} bsp_camera_data_t;

void bsp_uart_init(void);
void bsp_uart_port1_init(void);
void bsp_uart_port2_init(void);
void bsp_uart_jy901s_init(void);
void bsp_uart_imu_init(void);

bool uart_rxbuf_pop(uint8_t *byte);
bool bsp_uart1_rxbuf_pop(uint8_t *byte);
bool bsp_uart2_rxbuf_pop(uint8_t *byte);
bool bsp_uart_jy901s_rxbuf_pop(uint8_t *byte);
bool bsp_uart_imu_rxbuf_pop(uint8_t *byte);

/* Legacy Yabo IMU input on UART1, retained for source compatibility. */
bool uart_imu_rxbuf_pop(uint8_t *byte);

/* UART0 camera protocol: (<x_cm>,<v_cm_s>,@<CHK>)\r\n */
void bsp_camera_init(bsp_camera_data_t *camera);
bool bsp_camera_parse_byte(bsp_camera_data_t *camera, uint8_t byte);
uint32_t bsp_camera_process(bsp_camera_data_t *camera);

/* Explicit UART0 TX functions, reserved for future camera commands. */
void bsp_uart_send_byte(uint8_t ch);
void bsp_uart_send_str(const char *str);

/* Legacy debug output is retained as a no-op so UART0 stays camera-only. */
void debug_print(const char *fmt, ...);

#endif
