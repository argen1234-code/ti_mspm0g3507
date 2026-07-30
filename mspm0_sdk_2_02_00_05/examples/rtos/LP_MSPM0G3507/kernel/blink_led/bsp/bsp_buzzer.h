#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include <stdbool.h>

/* Schematic active-high buzzer module signal on PA21. */
void bsp_buzzer_init(void);
void bsp_buzzer_set(bool on);
void bsp_buzzer_on(void);
void bsp_buzzer_off(void);
bool bsp_buzzer_is_on(void);

#endif
