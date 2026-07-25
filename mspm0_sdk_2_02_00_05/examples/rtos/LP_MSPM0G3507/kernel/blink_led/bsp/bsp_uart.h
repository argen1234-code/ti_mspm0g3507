#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stdint.h>

/* Schematic mapping, all ports use 115200-8-N-1:
 * UART0 PA10/PA11, UART1 PA8/PA9, UART2 PB15/PB16,
 * UART3 PA26/PB13 (JY61P/JY901S). */
void bsp_uart_init(void);
void bsp_uart_port1_init(void);
void bsp_uart_port2_init(void);
void bsp_uart_jy901s_init(void);

bool uart_rxbuf_pop(uint8_t *byte);
bool bsp_uart1_rxbuf_pop(uint8_t *byte);
bool bsp_uart2_rxbuf_pop(uint8_t *byte);
bool bsp_uart_jy901s_rxbuf_pop(uint8_t *byte);

void bsp_uart_send_byte(uint8_t ch);
void bsp_uart_send_str(const char *str);
void debug_print(const char *fmt, ...);

#endif
