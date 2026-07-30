#ifndef BSP_KEY_H
#define BSP_KEY_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BSP_KEY_UP = 0,
    BSP_KEY_RIGHT,
    BSP_KEY_CENTER,
    BSP_KEY_LEFT,
    BSP_KEY_DOWN,
    BSP_KEY_COUNT
} bsp_key_id_t;

/* PCB physical orientation verified by the OLED key test. */
#define BSP_KEY_FRONT  BSP_KEY_DOWN
#define BSP_KEY_BACK   BSP_KEY_UP
#define BSP_KEY_LEFT_PHYSICAL  BSP_KEY_RIGHT
#define BSP_KEY_RIGHT_PHYSICAL BSP_KEY_LEFT
#define BSP_KEY_MIDDLE BSP_KEY_CENTER

void bsp_key_init(void);
void bsp_key_update(void);
bool bsp_key_is_pressed(bsp_key_id_t key);

/* Compatibility aliases: KEY1=UP, KEY2=DOWN. */
bool bsp_key1_is_pressed(void);
bool bsp_key2_is_pressed(void);
uint8_t bsp_key1_get_state(void);
uint8_t bsp_key2_get_state(void);

#endif
