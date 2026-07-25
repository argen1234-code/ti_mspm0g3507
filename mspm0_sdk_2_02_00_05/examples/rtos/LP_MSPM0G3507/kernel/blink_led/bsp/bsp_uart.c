#include "bsp_uart.h"
#include "ti_msp_dl_config.h"

#include <stdarg.h>
#include <stdio.h>

#define UART_RX_BUFFER_SIZE 256U

typedef struct {
    volatile uint8_t data[UART_RX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} uart_ring_t;

static uart_ring_t uart0_rx;
static uart_ring_t uart1_rx;
static uart_ring_t uart2_rx;
static uart_ring_t uart3_rx;

static void ring_push(uart_ring_t *ring, uint8_t byte)
{
    uint16_t next = (uint16_t) ((ring->head + 1U) % UART_RX_BUFFER_SIZE);

    if (next != ring->tail) {
        ring->data[ring->head] = byte;
        ring->head = next;
    }
}

static bool ring_pop(uart_ring_t *ring, uint8_t *byte)
{
    if ((byte == NULL) || (ring->head == ring->tail)) {
        return false;
    }

    *byte = ring->data[ring->tail];
    ring->tail = (uint16_t) ((ring->tail + 1U) % UART_RX_BUFFER_SIZE);
    return true;
}

static void uart_enable_rx(UART_Regs *uart, IRQn_Type irqn)
{
    DL_UART_Main_changeConfig(uart);
    DL_UART_Main_disableLoopbackMode(uart);
    DL_UART_Main_enableFIFOs(uart);
    DL_UART_Main_setRXFIFOThreshold(uart, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setTXFIFOThreshold(uart, DL_UART_TX_FIFO_LEVEL_1_2_EMPTY);
    DL_UART_Main_enable(uart);
    DL_UART_Main_enableInterrupt(uart, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(irqn);
}

static void uart0_putc(char c)
{
    while (DL_UART_isBusy(UART_0_INST)) {
    }
    DL_UART_Main_transmitData(UART_0_INST, (uint8_t) c);
}

void UART0_IRQHandler(void)
{
    if ((DL_UART_Main_getEnabledInterruptStatus(
            UART_0_INST, DL_UART_MAIN_INTERRUPT_RX) &
         DL_UART_MAIN_INTERRUPT_RX) != 0U) {
        while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST)) {
            ring_push(&uart0_rx,
                (uint8_t) DL_UART_Main_receiveData(UART_0_INST));
        }
    }
}

void UART1_IRQHandler(void)
{
    if ((DL_UART_Main_getEnabledInterruptStatus(
            UART_1_INST, DL_UART_MAIN_INTERRUPT_RX) &
         DL_UART_MAIN_INTERRUPT_RX) != 0U) {
        while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
            ring_push(&uart1_rx,
                (uint8_t) DL_UART_Main_receiveData(UART_1_INST));
        }
    }
}

void UART2_IRQHandler(void)
{
    if ((DL_UART_Main_getEnabledInterruptStatus(
            UART_2_INST, DL_UART_MAIN_INTERRUPT_RX) &
         DL_UART_MAIN_INTERRUPT_RX) != 0U) {
        while (!DL_UART_Main_isRXFIFOEmpty(UART_2_INST)) {
            ring_push(&uart2_rx,
                (uint8_t) DL_UART_Main_receiveData(UART_2_INST));
        }
    }
}

void UART3_IRQHandler(void)
{
    if ((DL_UART_Main_getEnabledInterruptStatus(
            UART_3_INST, DL_UART_MAIN_INTERRUPT_RX) &
         DL_UART_MAIN_INTERRUPT_RX) != 0U) {
        while (!DL_UART_Main_isRXFIFOEmpty(UART_3_INST)) {
            ring_push(&uart3_rx,
                (uint8_t) DL_UART_Main_receiveData(UART_3_INST));
        }
    }
}

void bsp_uart_init(void)
{
    uart_enable_rx(UART_0_INST, UART_0_INST_INT_IRQN);
}

void bsp_uart_port1_init(void)
{
    uart_enable_rx(UART_1_INST, UART_1_INST_INT_IRQN);
}

void bsp_uart_port2_init(void)
{
    uart_enable_rx(UART_2_INST, UART_2_INST_INT_IRQN);
}

void bsp_uart_jy901s_init(void)
{
    uart_enable_rx(UART_3_INST, UART_3_INST_INT_IRQN);
}

bool uart_rxbuf_pop(uint8_t *byte)
{
    return ring_pop(&uart0_rx, byte);
}

bool bsp_uart1_rxbuf_pop(uint8_t *byte)
{
    return ring_pop(&uart1_rx, byte);
}

bool bsp_uart2_rxbuf_pop(uint8_t *byte)
{
    return ring_pop(&uart2_rx, byte);
}

bool bsp_uart_jy901s_rxbuf_pop(uint8_t *byte)
{
    return ring_pop(&uart3_rx, byte);
}

int fputc(int ch, FILE *stream)
{
    (void) stream;
    uart0_putc((char) ch);
    return ch;
}

void bsp_uart_send_byte(uint8_t ch)
{
    uart0_putc((char) ch);
}

void bsp_uart_send_str(const char *str)
{
    if (str != NULL) {
        while (*str != '\0') {
            uart0_putc(*str++);
        }
    }
}

void debug_print(const char *fmt, ...)
{
    char buffer[192];
    va_list args;
    int length;
    int i;

    if (fmt == NULL) {
        return;
    }

    va_start(args, fmt);
    length = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (length < 0) {
        return;
    }
    if (length > (int) (sizeof(buffer) - 1U)) {
        length = (int) (sizeof(buffer) - 1U);
    }

    for (i = 0; i < length; i++) {
        uart0_putc(buffer[i]);
    }
}
