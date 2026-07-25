#include "bsp_motor.h"
#include "ti_msp_dl_config.h"

#define MOTOR_PWM_MAX 7999

static int32_t clamp_pwm(int32_t value)
{
    if (value > MOTOR_PWM_MAX) {
        return MOTOR_PWM_MAX;
    }
    if (value < -MOTOR_PWM_MAX) {
        return -MOTOR_PWM_MAX;
    }
    return value;
}

void bsp_motor_set_pwm(int32_t pwm_a, int32_t pwm_b)
{
    uint32_t duty_a;
    uint32_t duty_b;

    pwm_a = clamp_pwm(pwm_a);
    pwm_b = clamp_pwm(pwm_b);

    if (pwm_a > 0) {
        DL_GPIO_clearPins(AIN_AIN1_PORT, AIN_AIN1_PIN);
        DL_GPIO_setPins(AIN_AIN2_PORT, AIN_AIN2_PIN);
        duty_a = (uint32_t) pwm_a;
    } else if (pwm_a < 0) {
        DL_GPIO_setPins(AIN_AIN1_PORT, AIN_AIN1_PIN);
        DL_GPIO_clearPins(AIN_AIN2_PORT, AIN_AIN2_PIN);
        duty_a = (uint32_t) (-pwm_a);
    } else {
        DL_GPIO_clearPins(AIN_AIN1_PORT, AIN_AIN1_PIN);
        DL_GPIO_clearPins(AIN_AIN2_PORT, AIN_AIN2_PIN);
        duty_a = 0U;
    }

    if (pwm_b > 0) {
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN);
        DL_GPIO_setPins(BIN_PORT, BIN_BIN2_PIN);
        duty_b = (uint32_t) pwm_b;
    } else if (pwm_b < 0) {
        DL_GPIO_setPins(BIN_PORT, BIN_BIN1_PIN);
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN2_PIN);
        duty_b = (uint32_t) (-pwm_b);
    } else {
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN | BIN_BIN2_PIN);
        duty_b = 0U;
    }

    DL_Timer_setCaptureCompareValue(PWM_0_INST, duty_a, GPIO_PWM_0_C3_IDX);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, duty_b, GPIO_PWM_0_C2_IDX);
}
