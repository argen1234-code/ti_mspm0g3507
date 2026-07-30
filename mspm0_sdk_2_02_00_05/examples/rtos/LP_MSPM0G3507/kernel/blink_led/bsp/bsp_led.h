#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BSP_LED_1 = 0,
    BSP_LED_2,
    BSP_LED_3,
    BSP_LED_COUNT
} bsp_led_id_t;

/* Schematic LEDs are active-low: LED1=PB14, LED2=PB18, LED3=PA22. */
void bsp_led_init(void);
void bsp_led_set(bsp_led_id_t led, bool on);
void bsp_led_on(bsp_led_id_t led);
void bsp_led_off(bsp_led_id_t led);
void bsp_led_toggle(bsp_led_id_t led);
bool bsp_led_is_on(bsp_led_id_t led);
void bsp_led_all_off(void);

#endif
