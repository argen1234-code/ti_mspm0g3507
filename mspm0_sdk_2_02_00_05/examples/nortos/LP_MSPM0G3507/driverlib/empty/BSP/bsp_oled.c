/*
 * 引脚使用:
 *   PA.28 — OLED_SCL (SPI 时钟)
 *   PA.31 — OLED_SDA (SPI 数据 / MOSI)
 *   PB.14 — OLED_RST (硬件复位, 低有效)
 *   PB.15 — OLED_DC  (数据/命令选择: 0=命令, 1=数据)
 *
 * 驱动芯片: SSD1306, 128x64 点阵, 4线 SPI 模式
 * 显示范围: 4行 x 16列 (8x16 字符)
 */

#include "../ti_msp_dl_config.h"
#include "bsp_oled.h"
#include "bsp_oled_font.h"

#include <stdio.h>
#include <stdarg.h>

/* ============================================================
 *  内部: 毫秒延时 (基于 CPU 周期, SysTick 不可用时回退)
 * ============================================================ */
static void oled_delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        delay_cycles(32000); /* 32MHz * 1ms = 32000 cycles */
    }
}

/* ============================================================
 *  内部: SPI 发送一个字节 (软件模拟, MSB 先发)
 * ============================================================ */
static void oled_spi_write_byte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++) {
        OLED_W_SDA(byte & 0x80 ? 1 : 0);
        OLED_W_SCL(1);
        OLED_W_SCL(0);
        byte <<= 1;
    }
}

/* ============================================================
 *  内部: 写命令 (DC = 0)
 * ============================================================ */
static void oled_write_cmd(uint8_t cmd)
{
    OLED_W_DC(0);
    oled_spi_write_byte(cmd);
    OLED_W_DC(1); /* 回到数据模式, 方便后续连续写数据 */
}

/* ============================================================
 *  内部: 写数据 (DC = 1)
 * ============================================================ */
static void oled_write_data(uint8_t data)
{
    OLED_W_DC(1);
    oled_spi_write_byte(data);
}

/* ============================================================
 *  内部: 设置光标位置
 * ============================================================ */
static void oled_set_cursor(uint8_t page, uint8_t col)
{
    oled_write_cmd(0xB0 | page);
    oled_write_cmd(0x10 | ((col & 0xF0) >> 4));
    oled_write_cmd(0x00 | (col & 0x0F));
}

/* ============================================================
 *  OLED 初始化
 * ============================================================ */
void OLED_Init(void)
{
    /* 初始化 GPIO 引脚为数字输出 */
    DL_GPIO_initDigitalOutput(OLED_SCL_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_SDA_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_RST_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_DC_IOMUX);

    /* 上电延时 */
    oled_delay_ms(100);

    /* 硬件复位 */
    OLED_W_RST(0);
    oled_delay_ms(10);
    OLED_W_RST(1);
    oled_delay_ms(10);

    /* SSD1306 初始化序列 */
    oled_write_cmd(0xAE);   /* 关闭显示 */

    oled_write_cmd(0xD5);   /* 设置显示时钟分频比/振荡器频率 */
    oled_write_cmd(0x80);

    oled_write_cmd(0xA8);   /* 设置多路复用比 */
    oled_write_cmd(0x3F);   /* 64 */

    oled_write_cmd(0xD3);   /* 设置显示偏移 */
    oled_write_cmd(0x00);

    oled_write_cmd(0x40);   /* 设置显示起始行 */

    oled_write_cmd(0x8D);   /* 启用电荷泵 */
    oled_write_cmd(0x14);

    oled_write_cmd(0xA1);   /* 段重映射: 左右镜像 (0xA0=正常, 0xA1=镜像) */

    oled_write_cmd(0xC8);   /* COM 扫描方向: 上下镜像 (0xC0=正常, 0xC8=镜像) */

    oled_write_cmd(0xDA);   /* 设置 COM 引脚硬件配置 */
    oled_write_cmd(0x12);

    oled_write_cmd(0x81);   /* 设置对比度 */
    oled_write_cmd(0xCF);

    oled_write_cmd(0xD9);   /* 设置预充电周期 */
    oled_write_cmd(0xF1);

    oled_write_cmd(0xDB);   /* 设置 VCOMH 取消选择级别 */
    oled_write_cmd(0x30);

    oled_write_cmd(0xA4);   /* 全局显示: 跟随 RAM 内容 */

    oled_write_cmd(0xA6);   /* 正常显示 (非反转) */

    oled_write_cmd(0xAF);   /* 开启显示 */

    OLED_Clear();
}

/* ============================================================
 *  OLED 清屏
 * ============================================================ */
void OLED_Clear(void)
{
    for (uint8_t page = 0; page < 8; page++) {
        oled_set_cursor(page, 0);
        for (uint8_t col = 0; col < 128; col++) {
            oled_write_data(0x00);
        }
    }
}

/* ============================================================
 *  OLED 显示一个字符 (8x16)
 * ============================================================ */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t page = (Line - 1) * 2;

    /* 上半部分 (8 像素) */
    oled_set_cursor(page, (Column - 1) * 8);
    for (uint8_t i = 0; i < 8; i++) {
        oled_write_data(OLED_F8x16[Char - ' '][i]);
    }

    /* 下半部分 (8 像素) */
    oled_set_cursor(page + 1, (Column - 1) * 8);
    for (uint8_t i = 0; i < 8; i++) {
        oled_write_data(OLED_F8x16[Char - ' '][i + 8]);
    }
}

/* ============================================================
 *  OLED 显示字符串
 * ============================================================ */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    for (uint8_t i = 0; String[i] != '\0'; i++) {
        OLED_ShowChar(Line, Column + i, String[i]);
    }
}

/* ============================================================
 *  内部: 求 X 的 Y 次方
 * ============================================================ */
static uint32_t oled_pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) {
        Result *= X;
    }
    return Result;
}

/* ============================================================
 *  OLED 显示十进制无符号数
 * ============================================================ */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    for (uint8_t i = 0; i < Length; i++) {
        OLED_ShowChar(Line, Column + i,
            Number / oled_pow(10, Length - i - 1) % 10 + '0');
    }
}

/* ============================================================
 *  OLED 显示十进制有符号数
 * ============================================================ */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint32_t num_abs;
    if (Number >= 0) {
        OLED_ShowChar(Line, Column, '+');
        num_abs = Number;
    } else {
        OLED_ShowChar(Line, Column, '-');
        num_abs = -Number;
    }
    for (uint8_t i = 0; i < Length; i++) {
        OLED_ShowChar(Line, Column + i + 1,
            num_abs / oled_pow(10, Length - i - 1) % 10 + '0');
    }
}

/* ============================================================
 *  OLED 显示十六进制数
 * ============================================================ */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    for (uint8_t i = 0; i < Length; i++) {
        uint8_t digit = Number / oled_pow(16, Length - i - 1) % 16;
        if (digit < 10) {
            OLED_ShowChar(Line, Column + i, digit + '0');
        } else {
            OLED_ShowChar(Line, Column + i, digit - 10 + 'A');
        }
    }
}

/* ============================================================
 *  OLED 显示二进制数
 * ============================================================ */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    for (uint8_t i = 0; i < Length; i++) {
        OLED_ShowChar(Line, Column + i,
            Number / oled_pow(2, Length - i - 1) % 2 + '0');
    }
}

/* ============================================================
 *  OLED 格式化打印 (支持自动换行, \r \n \t)
 * ============================================================ */
int oled_printf(uint8_t Line, uint8_t Column, const char *format, ...)
{
    char buffer[256];
    va_list args;
    uint8_t line = Line;
    uint8_t col  = Column;
    int char_count = 0;

    if (Line < 1 || Line > 4 || Column < 1 || Column > 16) {
        return -1;
    }

    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len < 0) {
        return -1;
    }

    uint16_t i = 0;
    while (buffer[i] != '\0' && line <= 4) {
        /* \r\n */
        if (buffer[i] == '\r' && buffer[i + 1] == '\n') {
            col = 1;
            line++;
            i += 2;
            continue;
        }
        /* \n */
        if (buffer[i] == '\n') {
            col = 1;
            line++;
            i++;
            continue;
        }
        /* \r */
        if (buffer[i] == '\r') {
            col = 1;
            i++;
            continue;
        }
        /* \t → 4 空格 */
        if (buffer[i] == '\t') {
            uint8_t spaces = 4 - ((col - 1) % 4);
            for (uint8_t s = 0; s < spaces && col <= 16; s++) {
                OLED_ShowChar(line, col, ' ');
                col++;
                char_count++;
            }
            if (col > 16) {
                col = 1;
                line++;
            }
            i++;
            continue;
        }
        /* 普通字符 */
        OLED_ShowChar(line, col, buffer[i]);
        char_count++;

        col++;
        if (col > 16) {
            col = 1;
            line++;
        }
        i++;
    }

    return char_count;
}

/* ============================================================
 *  从指定列开始清空当前行余下部分
 * ============================================================ */
void oled_clear_line_from(uint8_t Line, uint8_t Column)
{
    if (Line < 1 || Line > 4 || Column < 1 || Column > 16) {
        return;
    }
    for (uint8_t col = Column; col <= 16; col++) {
        OLED_ShowChar(Line, col, ' ');
    }
}

/* ============================================================
 *  从指定位置开始清空屏幕余下部分
 * ============================================================ */
void oled_clear_from(uint8_t Line, uint8_t Column)
{
    if (Line < 1 || Line > 4 || Column < 1 || Column > 16) {
        return;
    }
    oled_clear_line_from(Line, Column);
    for (uint8_t l = Line + 1; l <= 4; l++) {
        oled_clear_line_from(l, 1);
    }
}
