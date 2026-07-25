#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

void Motor_Init(void);
void Motor_SetPWM_Left(int16_t Speed);
void Motor_SetPWM_Right(int16_t Speed);

#endif
