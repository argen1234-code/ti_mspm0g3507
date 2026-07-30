#ifndef BSP_TRACK_H
#define BSP_TRACK_H

#include <stdbool.h>
#include <stdint.h>

extern volatile bool g_track_c1_ready;
extern volatile uint8_t g_track_c1_raw_level;

void bsp_track_init(void);
void bsp_track_enable_c1_pullup(void);
uint8_t bsp_track_read(void);
bool bsp_track_get_channel(uint8_t ch);
int16_t bsp_track_line_position(bool *valid);

#endif
