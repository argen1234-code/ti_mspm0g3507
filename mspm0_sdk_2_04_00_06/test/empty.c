#include "ti_msp_dl_config.h"
#include "Motor.h"
#include "timer.h"
#include "delay.h"
#include "Track.h"

int main(void)
{
  SYSCFG_DL_init(); //系统初始化
	
	Delay_Init();
	

	
	
    
  while(1)
	{
		if(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_6) == 0)
	   {
		  Delay_ms(20);
		  while(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_6) == 0);
		  Delay_ms(20);

			 a += 4;
			 DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_2);
			 Delay_ms(200);
			 DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_2);
		 }
		 
		 	if(DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_2) == 0)
	   {
		  Delay_ms(20);
		  while(DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_2) == 0);
		  Delay_ms(20);
		  Motor_Init(); //电机初始化
			tima_1_init(); //1Ms 定时中断
			 
		 }
	
  }
}




