#ifndef __INFRARED_H
#define __INFRARED_H
#include "stdint.h"

int Position_PID(int current_error);
void Motor_Output(void);
void Line_Logic(void);

extern uint8_t a;

#endif
