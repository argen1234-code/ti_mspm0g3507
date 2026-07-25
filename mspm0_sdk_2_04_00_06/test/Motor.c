#include "Motor.h"
#include "math.h"


void Motor_Init(void)
{
	DL_TimerA_startCounter(PWM_MOTOR_INST);
}



void Motor_SetPWM_Left(int16_t PWM)
{
	if (PWM >= 0)
	{
		DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_26);
		DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_27);
		if (PWM > 99) {PWM = 99;}        
		DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, PWM  * 10, DL_TIMER_CC_0_INDEX);
	}
	else
	{
		DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_26);
		DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_27);
		if (PWM < -99) {PWM = -99;}
		DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, -PWM  * 10, DL_TIMER_CC_0_INDEX);
	}
}

void Motor_SetPWM_Right(int16_t PWM)
{
	if (PWM >= 0)
	{
		DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_24);
		DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_25);
		if (PWM > 99) {PWM = 99;}         
		DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, PWM * 10, DL_TIMER_CC_1_INDEX);   //x/800
	}
	else
	{
		DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_24);
		DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_25);
		if (PWM < -99) {PWM = -99;}
		DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, -PWM * 10, DL_TIMER_CC_1_INDEX);  
	}
}
