#include "bsp_track.h"
#include "ti_msp_dl_config.h"

/*
 * PA31/C1 is intentionally not declared in SysConfig's GPIO instance.
 * This prevents SYSCFG_DL_init() from connecting it before the tracking
 * module has completed its power-on sequence.
 */
#define TRACK_C1_DELAYED_PORT   (GPIOA)
#define TRACK_C1_DELAYED_PIN    (DL_GPIO_PIN_31)
#define TRACK_C1_DELAYED_IOMUX  (IOMUX_PINCM6)

volatile bool g_track_c1_ready = false;
volatile uint8_t g_track_c1_raw_level = 0U;

static bool track_pin_active(uint8_t ch)
{
    switch (ch) {
    case 0: {
        uint32_t raw = DL_GPIO_readPins(TRACK_C1_DELAYED_PORT,
                                       TRACK_C1_DELAYED_PIN);
        g_track_c1_raw_level = (raw == 0U) ? 0U : 1U;
        return g_track_c1_ready && (raw == 0U);
    }
    case 1: return DL_GPIO_readPins(TRACK_C2_PORT, TRACK_C2_PIN) == 0U;
    case 2: return DL_GPIO_readPins(TRACK_C3_PORT, TRACK_C3_PIN) == 0U;
    case 3: return DL_GPIO_readPins(TRACK_C4_PORT, TRACK_C4_PIN) == 0U;
    case 4: return DL_GPIO_readPins(TRACK_C5_PORT, TRACK_C5_PIN) == 0U;
    case 5: return DL_GPIO_readPins(TRACK_C6_PORT, TRACK_C6_PIN) == 0U;
    case 6: return DL_GPIO_readPins(TRACK_C7_PORT, TRACK_C7_PIN) == 0U;
    case 7: return DL_GPIO_readPins(TRACK_C8_PORT, TRACK_C8_PIN) == 0U;
    default: return false;
    }
}

void bsp_track_init(void)
{
    /* Explicitly retain PA31/C1 in the electrically disconnected state. */
    g_track_c1_ready = false;
    DL_GPIO_setAnalogInternalResistor(TRACK_C1_DELAYED_IOMUX,
                                      DL_GPIO_RESISTOR_NONE);
}

void bsp_track_enable_c1_pullup(void)
{
    DL_GPIO_initDigitalInputFeatures(
        TRACK_C1_DELAYED_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    g_track_c1_ready = true;
}

uint8_t bsp_track_read(void)
{
    uint8_t ch;
    uint8_t value = 0U;

    for (ch = 0U; ch < 8U; ch++) {
        if (track_pin_active(ch)) {
            value |= (uint8_t) (1U << ch);
        }
    }
    return value;
}

bool bsp_track_get_channel(uint8_t ch)
{
    return track_pin_active(ch);
}

int16_t bsp_track_line_position(bool *valid)
{
    static const int16_t weights[8] = {-350, -250, -150, -50, 50, 150, 250, 350};
    uint8_t data = bsp_track_read();
    int32_t sum = 0;
    uint8_t count = 0U;
    uint8_t ch;

    for (ch = 0U; ch < 8U; ch++) {
        if ((data & (uint8_t) (1U << ch)) != 0U) {
            sum += weights[ch];
            count++;
        }
    }

    if (valid != 0) {
        *valid = count != 0U;
    }
    return (count == 0U) ? 0 : (int16_t) (sum / count);
}
