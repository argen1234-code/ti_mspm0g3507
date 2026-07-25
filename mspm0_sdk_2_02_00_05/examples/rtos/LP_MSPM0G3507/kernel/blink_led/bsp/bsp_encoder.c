#include "bsp_encoder.h"
#include "ti_msp_dl_config.h"

volatile int32_t g_encoderA_cnt = 0;
volatile int32_t g_encoderB_cnt = 0;

void bsp_encoder_init(void)
{
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

void GROUP1_IRQHandler(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(
        GPIOB,
        ENCODERA_E1A_PIN | ENCODERA_E1B_PIN |
        ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);

    if ((status & ENCODERA_E1A_PIN) != 0U) {
        g_encoderA_cnt +=
            (DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1B_PIN) != 0U) ? -1 : 1;
    }
    if ((status & ENCODERA_E1B_PIN) != 0U) {
        g_encoderA_cnt +=
            (DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1A_PIN) != 0U) ? 1 : -1;
    }
    if ((status & ENCODERB_E2A_PIN) != 0U) {
        g_encoderB_cnt +=
            (DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2B_PIN) != 0U) ? -1 : 1;
    }
    if ((status & ENCODERB_E2B_PIN) != 0U) {
        g_encoderB_cnt +=
            (DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2A_PIN) != 0U) ? 1 : -1;
    }

    DL_GPIO_clearInterruptStatus(
        GPIOB,
        status & (ENCODERA_E1A_PIN | ENCODERA_E1B_PIN |
                  ENCODERB_E2A_PIN | ENCODERB_E2B_PIN));
}

float bsp_encoder_calc_rpm(int32_t encoder_count, uint32_t sample_time_ms)
{
    float pulses_per_rev;

    if (sample_time_ms == 0U) {
        return 0.0f;
    }

    pulses_per_rev = (float) (ENCODER_LINES * MULTIPLY_FACTOR * GEAR_RATIO);
    return ((float) encoder_count * 60000.0f) /
           (pulses_per_rev * (float) sample_time_ms);
}
