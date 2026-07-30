#include "bsp_led.h"
#include "ti_msp_dl_config.h"
#include <stddef.h>

static bool led_is_on[BSP_LED_COUNT];

static bool bsp_led_get_gpio(bsp_led_id_t led,
                             GPIO_Regs **port,
                             uint32_t *pin)
{
    if ((port == NULL) || (pin == NULL)) return false;

    switch (led) {
    case BSP_LED_1:
        *port = LEDS_LED1_PORT;
        *pin = LEDS_LED1_PIN;
        return true;
    case BSP_LED_2:
        *port = LEDS_LED2_PORT;
        *pin = LEDS_LED2_PIN;
        return true;
    case BSP_LED_3:
        *port = LEDS_LED3_PORT;
        *pin = LEDS_LED3_PIN;
        return true;
    default:
        return false;
    }
}

void bsp_led_init(void)
{
    bsp_led_all_off();
}

void bsp_led_set(bsp_led_id_t led, bool on)
{
    GPIO_Regs *port;
    uint32_t pin;

    if (!bsp_led_get_gpio(led, &port, &pin)) return;

    if (on) {
        DL_GPIO_clearPins(port, pin);
    } else {
        DL_GPIO_setPins(port, pin);
    }
    led_is_on[(uint8_t) led] = on;
}

void bsp_led_on(bsp_led_id_t led)
{
    bsp_led_set(led, true);
}

void bsp_led_off(bsp_led_id_t led)
{
    bsp_led_set(led, false);
}

void bsp_led_toggle(bsp_led_id_t led)
{
    if ((uint32_t) led >= (uint32_t) BSP_LED_COUNT) return;
    bsp_led_set(led, !led_is_on[(uint8_t) led]);
}

bool bsp_led_is_on(bsp_led_id_t led)
{
    return ((uint32_t) led < (uint32_t) BSP_LED_COUNT) &&
           led_is_on[(uint8_t) led];
}

void bsp_led_all_off(void)
{
    uint8_t led;

    for (led = 0U; led < (uint8_t) BSP_LED_COUNT; led++) {
        bsp_led_set((bsp_led_id_t) led, false);
    }
}
