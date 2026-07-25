#include "bsp_key.h"
#include "ti_msp_dl_config.h"

#define KEY_DEBOUNCE_SAMPLES 3U

static uint8_t key_state[BSP_KEY_COUNT];
static uint8_t key_count[BSP_KEY_COUNT];

static bool key_read_raw(bsp_key_id_t key)
{
    switch (key) {
    case BSP_KEY_UP:
        return DL_GPIO_readPins(KEYS_KEY_UP_PORT, KEYS_KEY_UP_PIN) == 0U;
    case BSP_KEY_RIGHT:
        return DL_GPIO_readPins(KEYS_KEY_RIGHT_PORT, KEYS_KEY_RIGHT_PIN) == 0U;
    case BSP_KEY_CENTER:
        return DL_GPIO_readPins(KEYS_KEY_CENTER_PORT, KEYS_KEY_CENTER_PIN) == 0U;
    case BSP_KEY_LEFT:
        return DL_GPIO_readPins(KEYS_KEY_LEFT_PORT, KEYS_KEY_LEFT_PIN) == 0U;
    case BSP_KEY_DOWN:
        return DL_GPIO_readPins(KEYS_KEY_DOWN_PORT, KEYS_KEY_DOWN_PIN) == 0U;
    default:
        return false;
    }
}

void bsp_key_init(void)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t) BSP_KEY_COUNT; i++) {
        key_state[i] = key_read_raw((bsp_key_id_t) i) ? 1U : 0U;
        key_count[i] = 0U;
    }
}

void bsp_key_update(void)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t) BSP_KEY_COUNT; i++) {
        uint8_t raw = key_read_raw((bsp_key_id_t) i) ? 1U : 0U;

        if (raw == key_state[i]) {
            key_count[i] = 0U;
        } else if (++key_count[i] >= KEY_DEBOUNCE_SAMPLES) {
            key_state[i] = raw;
            key_count[i] = 0U;
        }
    }
}

bool bsp_key_is_pressed(bsp_key_id_t key)
{
    return ((uint32_t) key < (uint32_t) BSP_KEY_COUNT) &&
           (key_state[(uint8_t) key] != 0U);
}

bool bsp_key1_is_pressed(void)
{
    return bsp_key_is_pressed(BSP_KEY_UP);
}

bool bsp_key2_is_pressed(void)
{
    return bsp_key_is_pressed(BSP_KEY_DOWN);
}

uint8_t bsp_key1_get_state(void)
{
    return bsp_key1_is_pressed() ? 1U : 0U;
}

uint8_t bsp_key2_get_state(void)
{
    return bsp_key2_is_pressed() ? 1U : 0U;
}
