#ifndef BSP_OLED_H
#define BSP_OLED_H

#include <stdint.h>

/* Schematic OLED: SSD1306-compatible I2C, SCL=PB2, SDA=PB3. */
#define OLED_PRINTF 1

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
int oled_printf(uint8_t Line, uint8_t Column, const char *format, ...);
void oled_clear_line_from(uint8_t Line, uint8_t Column);
void oled_clear_from(uint8_t Line, uint8_t Column);

#endif
