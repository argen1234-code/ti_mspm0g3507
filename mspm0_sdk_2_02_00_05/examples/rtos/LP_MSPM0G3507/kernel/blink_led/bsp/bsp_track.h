#ifndef BSP_TRACK_H
#define BSP_TRACK_H

#include <stdbool.h>
#include <stdint.h>

void bsp_track_init(void);
uint8_t bsp_track_read(void);
bool bsp_track_get_channel(uint8_t ch);
int16_t bsp_track_line_position(bool *valid);

#endif
