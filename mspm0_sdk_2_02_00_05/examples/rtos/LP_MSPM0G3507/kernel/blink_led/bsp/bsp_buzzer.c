#include "bsp_buzzer.h"
#include "ti_msp_dl_config.h"

static bool buzzer_is_on = false;

void bsp_buzzer_init(void)
{
    DL_GPIO_clearPins(BUZZER_GPIO_PORT, BUZZER_GPIO_BUZZER_PIN);
    buzzer_is_on = false;
}

void bsp_buzzer_set(bool on)
{
    if (on) {
        DL_GPIO_setPins(BUZZER_GPIO_PORT, BUZZER_GPIO_BUZZER_PIN);
    } else {
        DL_GPIO_clearPins(BUZZER_GPIO_PORT, BUZZER_GPIO_BUZZER_PIN);
    }
    buzzer_is_on = on;
}

void bsp_buzzer_on(void)
{
    bsp_buzzer_set(true);
}

void bsp_buzzer_off(void)
{
    bsp_buzzer_set(false);
}

bool bsp_buzzer_is_on(void)
{
    return buzzer_is_on;
}
