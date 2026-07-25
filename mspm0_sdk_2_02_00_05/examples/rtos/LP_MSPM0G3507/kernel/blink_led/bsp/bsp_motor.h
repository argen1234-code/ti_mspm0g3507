#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>

/* TB6612: PWMA=PA28, PWMB=PB20, AIN1=PA13, AIN2=PB26,
 *         BIN1=PB9, BIN2=PB7. STBY is tied to +5V on the schematic. */
void bsp_motor_set_pwm(int32_t pwma, int32_t pwmb);

#endif
