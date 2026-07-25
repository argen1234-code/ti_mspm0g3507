#include "ti_msp_dl_config.h"
#include "Motor.h"
#include "delay.h"
#include "math.h"

// 红外传感器引脚定义
#define D1 DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_15)  // 最左侧
#define D2 DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_16)
#define D3 DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_21)  
#define D4 DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_22)
#define D5 DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_23) 
#define D6 DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_7) 
#define D7 DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_8) 
#define D8 DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_9) 
#define D9 DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_18) 
#define D10 DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_19) 
#define D11 DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_20) 
#define D12 DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_24)  // 最右侧

// 控制参数

float error = 0;
float OUTPUT_L = 0, OUTPUT_R = 0; 
float PID_out = 0;  
uint8_t turn_flag = 0;
__IO uint16_t count = 0;
__IO uint8_t angle_90_num = 0;
 
// PID参数（调整后）
#define KP          0.15   
#define KI          0.0001    
#define KD          3.2  
#define MAX_PWM     99      // PWM上限
#define MIN_PWM     -99       // PWM下限
#define BASE_SPEED 24  // 基础PWM值

__IO uint8_t a = 0;


void Line_Logic(void) 
{
  float d1 = D1; // 读取传感器值（0或1）
  float d2 = D2;
  float d3 = D3;
  float d4 = D4;
  float d5 = D5;
  float d6 = D6;
  float d7 = D7;
  float d8 = D8;
  float d9 = D9;
  float d10 = D10;
  float d11 = D11;
  float d12 = D12;
	

  // 计算加权和及总数
  float weights[] = {6.8, 3.0, 1.5, 1.0, 0.0, -0.5, -1.5, -3.0, -6.8}; // 加强边缘权重
  float sum = weights[0]*d3 + weights[1]*d4 + weights[2]*d5 + weights[3]*d6 + weights[5]*d7 + weights[6]*d8 + weights[7]*d9 + weights[8]*d10;
  float total = d4 + d5 + d6 + d7 + d8 + d9;                                                                          

  static float last_valid_error = 0; // 保存上一次有效误差

	
  if (total > 0) 
	{
    error = (sum / total) * 10;
    last_valid_error = error;
		if ((d1 > 0.1) || (d2 > 0.1))
		{
			turn_flag = 1;
		}
		else if ((d12 > 0.1) || (d11 > 0.1))
		{
			turn_flag = 2;
		}
	
   } 
	
	else 
	{
      // 无传感器检测到时
			error = last_valid_error;
  }

  // 限制误差范围
  if (error > 40) error = 40;
  else if (error < -40) error = -40;
	
}

int Position_PID(int current_error) 
{
  static float integral = 0, last_error = 0;
  float error = (float)current_error;
  
  // PID计算（KP已缩小10倍）
  integral += error * 0.01;
	if (integral > 5) {integral = 5;}
	else if (integral < -5) {integral = -5;}
	
  float derivative = (error - last_error) / 0.01;
  float output = KP * error + KI * integral + KD * derivative;
  
  last_error = error;
  return (int)output;
}

void Motor_Output(void) 
{
	if (turn_flag == 1)
	{
		Motor_SetPWM_Left(3);
	  Motor_SetPWM_Right(20);
		count ++;
		if (count > 92)
		{
			angle_90_num ++;
			count = 0;
			turn_flag = 0;
		}
	}
	else if (turn_flag == 2)
	{
	  Motor_SetPWM_Left(20);
	  Motor_SetPWM_Right(3);
		count ++;
		if (count > 92)
		{
			angle_90_num ++;
			count = 0;
			turn_flag = 0;
		}
	}
	else if (angle_90_num == a)
	{
		count ++;
		if (count > 20)
		{
			Motor_SetPWM_Left(0);
	    Motor_SetPWM_Right(0);
		}

	}
	else
	{
    // 差速分配
    OUTPUT_L = BASE_SPEED - PID_out;
    OUTPUT_R = BASE_SPEED + PID_out+3;
		

		
		  // PWM限幅
    OUTPUT_L = OUTPUT_L > MAX_PWM ? MAX_PWM : (OUTPUT_L < MIN_PWM ? MIN_PWM : OUTPUT_L);
    OUTPUT_R = OUTPUT_R > MAX_PWM ? MAX_PWM : (OUTPUT_R < MIN_PWM ? MIN_PWM : OUTPUT_R);
	  
    // 设置电机PWM，未使用编码器PID
	  Motor_SetPWM_Left(OUTPUT_L);
	  Motor_SetPWM_Right(OUTPUT_R);
	}
	
}

void TIMER_10ms_INST_IRQHandler(void)
{
		
		    //如果产生了定时器中断
    switch( DL_TimerG_getPendingInterrupt(TIMER_10ms_INST) )
    {
        case DL_TIMER_IIDX_ZERO://如果是0溢出中断
					
                  Line_Logic();           // 计算误差
                  PID_out = Position_PID(error); // PID计算
                  Motor_Output();         // 更新电机PWM
            break;

            default://其他的定时器中断
            break;
    }
}

