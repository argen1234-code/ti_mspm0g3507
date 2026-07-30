#include "bsp_uart.h"
#include "ti_msp_dl_config.h"

#include <stdio.h>
#include <string.h>

#define UART_RX_BUFFER_SIZE 256U
#define BSP_UART3_INST UART3
#define BSP_UART3_IRQN UART3_INT_IRQn
#define BSP_UART3_RX_IOMUX IOMUX_PINCM30
#define BSP_UART3_RX_FUNC IOMUX_PINCM30_PF_UART3_RX
#define BSP_UART3_IBRD_40MHZ_115200 (21U)
#define BSP_UART3_FBRD_40MHZ_115200 (45U)

typedef struct {
    volatile uint8_t data[UART_RX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint32_t dropped_bytes;
} uart_ring_t;

static uart_ring_t uart0_rx;
static uart_ring_t uart1_rx;
static uart_ring_t uart2_rx;
static uart_ring_t uart3_rx;
static bool uart3_rx_initialized;
static volatile uint32_t uart0_raw_bytes;
static volatile uint32_t uart0_rx_irq_count;
static volatile uint8_t uart0_last_raw_byte;

static const DL_UART_Main_ClockConfig g_bsp_uart3_clock_config = {
    .clockSel = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1,
};

static const DL_UART_Main_Config g_bsp_uart3_config = {
    .mode = DL_UART_MAIN_MODE_NORMAL,
    .direction = DL_UART_MAIN_DIRECTION_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity = DL_UART_MAIN_PARITY_NONE,
    .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits = DL_UART_MAIN_STOP_BITS_ONE,
};

static void ring_push(uart_ring_t *ring, uint8_t byte)
{
    uint16_t next = (uint16_t) ((ring->head + 1U) % UART_RX_BUFFER_SIZE);

    if (next != ring->tail) {
        ring->data[ring->head] = byte;
        ring->head = next;
    } else {
        ring->dropped_bytes++;
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
    NVIC_ClearPendingIRQ(irqn);
    DL_UART_Main_enableInterrupt(uart, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(irqn);
}

static void uart0_putc(char c)
{
    while (DL_UART_isBusy(UART_0_INST)) {
    }
    DL_UART_Main_transmitData(UART_0_INST, (uint8_t) c);
}

static bool camera_hex_to_nibble(char ch, uint8_t *nibble)
{
    if (nibble == NULL) {
        return false;
    }
    if ((ch >= '0') && (ch <= '9')) {
        *nibble = (uint8_t)(ch - '0');
        return true;
    }
    if ((ch >= 'A') && (ch <= 'F')) {
        *nibble = (uint8_t)(ch - 'A' + 10);
        return true;
    }
    if ((ch >= 'a') && (ch <= 'f')) {
        *nibble = (uint8_t)(ch - 'a' + 10);
        return true;
    }
    return false;
}

static bool camera_parse_decimal(const char *text, float *value)
{
    const char *cursor;
    float result = 0.0f;
    float fraction_scale = 0.1f;
    bool negative;
    bool integer_digit = false;
    bool fraction_digit = false;

    if ((text == NULL) || (value == NULL)) {
        return false;
    }

    cursor = text;
    if ((*cursor != '+') && (*cursor != '-')) {
        return false;
    }
    negative = (*cursor == '-');
    cursor++;

    while ((*cursor >= '0') && (*cursor <= '9')) {
        integer_digit = true;
        result = result * 10.0f + (float)(*cursor - '0');
        cursor++;
    }
    if (!integer_digit || (*cursor != '.')) {
        return false;
    }
    cursor++;

    while ((*cursor >= '0') && (*cursor <= '9')) {
        fraction_digit = true;
        result += (float)(*cursor - '0') * fraction_scale;
        fraction_scale *= 0.1f;
        cursor++;
    }
    if (!fraction_digit || (*cursor != '\0')) {
        return false;
    }

    *value = negative ? -result : result;
    return true;
}

static bool camera_parse_frame(bsp_camera_data_t *camera)
{
    char *checksum_marker = NULL;
    uint8_t checksum_marker_index = 0U;
    char *field_separator;
    uint8_t checksum = 0U;
    uint8_t checksum_high;
    uint8_t checksum_low;
    uint8_t received_checksum;
    float position_cm;
    float velocity_cm_s;
    uint8_t i;

    camera->received_frames++;
    camera->valid = false;

    for (i = 0U; (uint16_t)i + 1U < camera->rx_length; i++) {
        if ((camera->rx_frame[i] == ',') &&
            (camera->rx_frame[i + 1U] == '@')) {
            checksum_marker = &camera->rx_frame[i];
            checksum_marker_index = i;
        }
    }

    if ((checksum_marker == NULL) ||
        ((uint16_t)checksum_marker_index + 4U != camera->rx_length) ||
        !camera_hex_to_nibble(checksum_marker[2], &checksum_high) ||
        !camera_hex_to_nibble(checksum_marker[3], &checksum_low)) {
        camera->format_errors++;
        return false;
    }

    for (i = 0U; &camera->rx_frame[i] < checksum_marker; i++) {
        checksum = (uint8_t)(checksum + (uint8_t)camera->rx_frame[i]);
    }
    received_checksum = (uint8_t)((checksum_high << 4) | checksum_low);
    camera->calculated_checksum = checksum;
    camera->received_checksum = received_checksum;
    if (checksum != received_checksum) {
        camera->checksum_errors++;
        return false;
    }

    *checksum_marker = '\0';
    field_separator = strchr(camera->rx_frame, ',');
    if ((field_separator == NULL) ||
        (strchr(field_separator + 1, ',') != NULL)) {
        camera->format_errors++;
        return false;
    }
    *field_separator = '\0';

    if (!camera_parse_decimal(camera->rx_frame, &position_cm) ||
        !camera_parse_decimal(field_separator + 1, &velocity_cm_s)) {
        camera->format_errors++;
        return false;
    }
    if ((position_cm < BSP_CAMERA_POSITION_MIN_CM) ||
        (position_cm > BSP_CAMERA_POSITION_MAX_CM)) {
        camera->range_errors++;
        return false;
    }

    camera->position_cm = position_cm;
    camera->velocity_cm_s = velocity_cm_s;
    camera->valid_frames++;
    camera->sequence++;
    camera->valid = true;
    return true;
}

void UART0_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART_0_INST) !=
        DL_UART_MAIN_IIDX_RX) {
        return;
    }

    uart0_rx_irq_count++;
    while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST)) {
        uint8_t byte =
            (uint8_t) DL_UART_Main_receiveData(UART_0_INST);

        uart0_last_raw_byte = byte;
        uart0_raw_bytes++;
        ring_push(&uart0_rx, byte);
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
            BSP_UART3_INST, DL_UART_MAIN_INTERRUPT_RX) &
         DL_UART_MAIN_INTERRUPT_RX) != 0U) {
        while (!DL_UART_Main_isRXFIFOEmpty(BSP_UART3_INST)) {
            ring_push(&uart3_rx,
                (uint8_t) DL_UART_Main_receiveData(BSP_UART3_INST));
        }
    }
}

void bsp_uart_init(void)
{
    /* Reassert PA11 as the external UART0 RX pin before enabling RX IRQ. */
    DL_GPIO_initPeripheralInputFunction(GPIO_UART_0_IOMUX_RX,
                                        GPIO_UART_0_IOMUX_RX_FUNC);
    NVIC_DisableIRQ(UART_0_INST_INT_IRQN);
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
    if (!uart3_rx_initialized) {
        DL_UART_Main_reset(BSP_UART3_INST);
        DL_UART_Main_enablePower(BSP_UART3_INST);
        delay_cycles(POWER_STARTUP_DELAY);

        DL_GPIO_initPeripheralInputFunction(BSP_UART3_RX_IOMUX,
                                            BSP_UART3_RX_FUNC);
        DL_UART_Main_setClockConfig(
            BSP_UART3_INST,
            (DL_UART_Main_ClockConfig *)&g_bsp_uart3_clock_config);
        DL_UART_Main_init(BSP_UART3_INST,
                          (DL_UART_Main_Config *)&g_bsp_uart3_config);
        DL_UART_Main_setOversampling(BSP_UART3_INST,
                                     DL_UART_OVERSAMPLING_RATE_16X);
        /* UART Main BUSCLK follows the 40 MHz ULPCLK in this clock tree. */
        DL_UART_Main_setBaudRateDivisor(BSP_UART3_INST,
                                        BSP_UART3_IBRD_40MHZ_115200,
                                        BSP_UART3_FBRD_40MHZ_115200);
        DL_UART_Main_enableFIFOs(BSP_UART3_INST);
        DL_UART_Main_setRXFIFOThreshold(BSP_UART3_INST,
                                        DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
        DL_UART_Main_disableLoopbackMode(BSP_UART3_INST);
        DL_UART_Main_enable(BSP_UART3_INST);
        uart3_rx_initialized = true;
    }

    DL_UART_Main_enableInterrupt(BSP_UART3_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(BSP_UART3_IRQN);
}

void bsp_uart_imu_init(void)
{
    bsp_uart_jy901s_init();
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

bool bsp_uart_imu_rxbuf_pop(uint8_t *byte)
{
    return ring_pop(&uart3_rx, byte);
}

bool uart_imu_rxbuf_pop(uint8_t *byte)
{
    return ring_pop(&uart1_rx, byte);
}

void bsp_camera_init(bsp_camera_data_t *camera)
{
    if (camera != NULL) {
        memset(camera, 0, sizeof(*camera));
        uart0_raw_bytes = 0U;
        uart0_rx_irq_count = 0U;
        uart0_last_raw_byte = 0U;
    }
}

bool bsp_camera_parse_byte(bsp_camera_data_t *camera, uint8_t byte)
{
    bool frame_valid;

    if (camera == NULL) {
        return false;
    }

    if (byte == (uint8_t)'(') {
        if (camera->receiving && (camera->rx_length != 0U)) {
            camera->format_errors++;
        }
        camera->rx_length = 0U;
        camera->receiving = true;
        return false;
    }
    if (!camera->receiving) {
        return false;
    }
    if (byte == (uint8_t)')') {
        if (camera->rx_length == 0U) {
            camera->format_errors++;
            camera->valid = false;
            camera->receiving = false;
            return false;
        }

        camera->rx_frame[camera->rx_length] = '\0';
        frame_valid = camera_parse_frame(camera);
        camera->rx_length = 0U;
        camera->receiving = false;
        return frame_valid;
    }
    if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n')) {
        camera->format_errors++;
        camera->valid = false;
        camera->rx_length = 0U;
        camera->receiving = false;
        return false;
    }
    if ((byte < 0x20U) || (byte > 0x7EU)) {
        camera->format_errors++;
        camera->valid = false;
        camera->rx_length = 0U;
        camera->receiving = false;
        return false;
    }
    if (camera->rx_length >= (BSP_CAMERA_FRAME_MAX_LENGTH - 1U)) {
        camera->frame_overflow_errors++;
        camera->valid = false;
        camera->rx_length = 0U;
        camera->receiving = false;
        return false;
    }

    camera->rx_frame[camera->rx_length++] = (char)byte;
    return false;
}

uint32_t bsp_camera_process(bsp_camera_data_t *camera)
{
    uint8_t byte;
    uint32_t valid_frames = 0U;

    if (camera == NULL) {
        return 0U;
    }

    while (uart_rxbuf_pop(&byte)) {
        if (bsp_camera_parse_byte(camera, byte)) {
            valid_frames++;
        }
    }
    camera->raw_bytes = uart0_raw_bytes;
    camera->rx_irq_count = uart0_rx_irq_count;
    camera->last_raw_byte = uart0_last_raw_byte;
    camera->uart_enabled = DL_UART_Main_isEnabled(UART_0_INST);
    camera->uart_loopback_enabled =
        DL_UART_Main_isLoopbackModeEnabled(UART_0_INST);
    camera->uart_integer_divisor = (uint16_t)
        DL_UART_Main_getIntegerBaudRateDivisor(UART_0_INST);
    camera->uart_fractional_divisor = (uint8_t)
        DL_UART_Main_getFractionalBaudRateDivisor(UART_0_INST);
    camera->dropped_bytes = uart0_rx.dropped_bytes;
    return valid_frames;
}

int fputc(int ch, FILE *stream)
{
    /* printf/debug output is disabled because UART0 belongs to the camera. */
    (void) ch;
    (void) stream;
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
    /* Kept for source compatibility; deliberately sends nothing on UART0. */
    (void) fmt;
}
