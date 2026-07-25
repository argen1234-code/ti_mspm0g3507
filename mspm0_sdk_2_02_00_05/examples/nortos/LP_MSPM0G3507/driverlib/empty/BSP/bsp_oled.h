#ifndef BSP_OLED_H
#define BSP_OLED_H

#include <stdint.h>

/*
 * 引脚使用:
 *   PA.28 — OLED_SCL (SPI 时钟)
 *   PA.31 — OLED_SDA (SPI 数据 / MOSI)
 *   PB.14 — OLED_RST (硬件复位, 低有效)
 *   PB.15 — OLED_DC  (数据/命令选择: 0=命令, 1=数据)
 *
 * 驱动芯片: SSD1306, 128x64 点阵, 4线 SPI 模式
 */

#define OLED_PRINTF 1

/* ---- 引脚 IOMUX ---- */
#define OLED_SCL_IOMUX   ((uint32_t)IOMUX_PINCM3)
#define OLED_SDA_IOMUX   ((uint32_t)IOMUX_PINCM6)
#define OLED_RST_IOMUX   ((uint32_t)IOMUX_PINCM31)
#define OLED_DC_IOMUX    ((uint32_t)IOMUX_PINCM32)

/* ---- 引脚端口与位 ---- */
#define OLED_PORT_SCL    GPIOA
#define OLED_PIN_SCL     DL_GPIO_PIN_28
#define OLED_PORT_SDA    GPIOA
#define OLED_PIN_SDA     DL_GPIO_PIN_31
#define OLED_PORT_RST    GPIOB
#define OLED_PIN_RST     DL_GPIO_PIN_14
#define OLED_PORT_DC     GPIOB
#define OLED_PIN_DC      DL_GPIO_PIN_15

/* ---- 引脚操作宏 ---- */
#define OLED_W_SCL(x)    DL_GPIO_writePinsVal(OLED_PORT_SCL, OLED_PIN_SCL, (x) ? OLED_PIN_SCL : 0)
#define OLED_W_SDA(x)    DL_GPIO_writePinsVal(OLED_PORT_SDA, OLED_PIN_SDA, (x) ? OLED_PIN_SDA : 0)
#define OLED_W_RST(x)    DL_GPIO_writePinsVal(OLED_PORT_RST, OLED_PIN_RST, (x) ? OLED_PIN_RST : 0)
#define OLED_W_DC(x)     DL_GPIO_writePinsVal(OLED_PORT_DC, OLED_PIN_DC, (x) ? OLED_PIN_DC : 0)

/* ============================================================
 *  外部接口
 * ============================================================ */

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

int  oled_printf(uint8_t Line, uint8_t Column, const char *format, ...);
void oled_clear_line_from(uint8_t Line, uint8_t Column);
void oled_clear_from(uint8_t Line, uint8_t Column);

#endif
