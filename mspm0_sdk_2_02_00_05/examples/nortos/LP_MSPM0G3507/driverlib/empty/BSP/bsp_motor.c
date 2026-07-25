#include "bsp_motor.h"
#include <stdlib.h>

/***************************************************************************************************
*   函 数 名: Motor_Init
*   入口参数: 无
*   返 回 值: 无
*   函数功能: 配置PWM引脚 + 方向引脚, 启动TIMA0
*   说    明: 引脚 IOMUX 手动配置 (PA8/PA9/PA15/PB13 → TIMA0 CCP0~3)
****************************************************************************************************/
void Motor_Init(void)
{
    /* ---- TIMA0 时钟配置 (32MHz / 16 = 2MHz timer clock) ---- */
    DL_Timer_ClockConfig clkCfg = {
        .clockSel    = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale    = 15
    };
    DL_TimerA_setClockConfig(MOTOR_FL_PWM_INST, &clkCfg);

    /* ---- TIMA0 PWM 模式配置 ---- */
    DL_Timer_PWMConfig pwmCfg = {
        .period             = 1000,
        .pwmMode            = DL_TIMER_PWM_MODE_EDGE_ALIGN,
        .isTimerWithFourCC  = true,
        .startTimer         = DL_TIMER_STOP
    };
    DL_TimerA_initPWMMode(MOTOR_FL_PWM_INST, &pwmCfg);

    /* ---- PWM 引脚配置为 TIMA0 外设输出 ---- */
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM19,
        IOMUX_PINCM19_PF_TIMA0_CCP0);
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM20,
        IOMUX_PINCM20_PF_TIMA0_CCP1);
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM37,
        IOMUX_PINCM37_PF_TIMA0_CCP2);
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM30,
        IOMUX_PINCM30_PF_TIMA0_CCP3);

    /* ---- 方向引脚配置为数字输出 ---- */
    DL_GPIO_initDigitalOutput(MOTOR_FL_IN1_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_FL_IN2_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_FR_IN1_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_FR_IN2_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_RL_IN1_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_RL_IN2_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_RR_IN1_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_RR_IN2_IOMUX);

    /* 所有电机初始化为刹车状态 (IN1=H, IN2=H) */
    DL_GPIO_setPins(MOTOR_FL_IN1_PORT, MOTOR_FL_IN1_PIN);
    DL_GPIO_setPins(MOTOR_FL_IN2_PORT, MOTOR_FL_IN2_PIN);
    DL_GPIO_setPins(MOTOR_FR_IN1_PORT, MOTOR_FR_IN1_PIN);
    DL_GPIO_setPins(MOTOR_FR_IN2_PORT, MOTOR_FR_IN2_PIN);
    DL_GPIO_setPins(MOTOR_RL_IN1_PORT, MOTOR_RL_IN1_PIN);
    DL_GPIO_setPins(MOTOR_RL_IN2_PORT, MOTOR_RL_IN2_PIN);
    DL_GPIO_setPins(MOTOR_RR_IN1_PORT, MOTOR_RR_IN1_PIN);
    DL_GPIO_setPins(MOTOR_RR_IN2_PORT, MOTOR_RR_IN2_PIN);

    DL_TimerA_startCounter(MOTOR_FL_PWM_INST);
}

/***************************************************************************************************
*   函 数 名: PWM_SetCompare
*   入口参数: value — 比较值(0~PWM_MAX)
*             motor_position — 电机位置
*   返 回 值: 无
*   函数功能: 设置指定电机的PWM占空比
****************************************************************************************************/
static void PWM_SetCompare(uint16_t value, uint8_t motor_position)
{
    switch (motor_position) {
    case MOTOR_FRONT_LEFT:
        DL_TimerA_setCaptureCompareValue(MOTOR_FL_PWM_INST,
            value, MOTOR_FL_PWM_CH);
        break;
    case MOTOR_FRONT_RIGHT:
        DL_TimerA_setCaptureCompareValue(MOTOR_FR_PWM_INST,
            value, MOTOR_FR_PWM_CH);
        break;
    case MOTOR_REAR_LEFT:
        DL_TimerA_setCaptureCompareValue(MOTOR_RL_PWM_INST,
            value, MOTOR_RL_PWM_CH);
        break;
    case MOTOR_REAR_RIGHT:
        DL_TimerA_setCaptureCompareValue(MOTOR_RR_PWM_INST,
            value, MOTOR_RR_PWM_CH);
        break;
    default:
        break;
    }
}

/***************************************************************************************************
*   函 数 名: Motor_SetDirection
*   入口参数: PWM — 带符号速度值(正=正转,负=反转,0=刹车)
*             motor_position — 电机位置
*   返 回 值: 无
*   函数功能: 根据PWM符号设置TB6612方向引脚
*   说    明: IN1=H/IN2=L → 正转, IN1=L/IN2=H → 反转, IN1=H/IN2=H → 刹车
****************************************************************************************************/
static void Motor_SetDirection(int16_t PWM, uint8_t motor_position)
{
    switch (motor_position) {
    case MOTOR_FRONT_LEFT:
        if (PWM > 0) {
            DL_GPIO_setPins(MOTOR_FL_IN1_PORT, MOTOR_FL_IN1_PIN);
            DL_GPIO_clearPins(MOTOR_FL_IN2_PORT, MOTOR_FL_IN2_PIN);
        } else if (PWM < 0) {
            DL_GPIO_clearPins(MOTOR_FL_IN1_PORT, MOTOR_FL_IN1_PIN);
            DL_GPIO_setPins(MOTOR_FL_IN2_PORT, MOTOR_FL_IN2_PIN);
        } else {
            DL_GPIO_setPins(MOTOR_FL_IN1_PORT, MOTOR_FL_IN1_PIN);
            DL_GPIO_setPins(MOTOR_FL_IN2_PORT, MOTOR_FL_IN2_PIN);
        }
        break;
    case MOTOR_FRONT_RIGHT:
        if (PWM > 0) {
            DL_GPIO_setPins(MOTOR_FR_IN1_PORT, MOTOR_FR_IN1_PIN);
            DL_GPIO_clearPins(MOTOR_FR_IN2_PORT, MOTOR_FR_IN2_PIN);
        } else if (PWM < 0) {
            DL_GPIO_clearPins(MOTOR_FR_IN1_PORT, MOTOR_FR_IN1_PIN);
            DL_GPIO_setPins(MOTOR_FR_IN2_PORT, MOTOR_FR_IN2_PIN);
        } else {
            DL_GPIO_setPins(MOTOR_FR_IN1_PORT, MOTOR_FR_IN1_PIN);
            DL_GPIO_setPins(MOTOR_FR_IN2_PORT, MOTOR_FR_IN2_PIN);
        }
        break;
    case MOTOR_REAR_LEFT:
        if (PWM > 0) {
            DL_GPIO_setPins(MOTOR_RL_IN1_PORT, MOTOR_RL_IN1_PIN);
            DL_GPIO_clearPins(MOTOR_RL_IN2_PORT, MOTOR_RL_IN2_PIN);
        } else if (PWM < 0) {
            DL_GPIO_clearPins(MOTOR_RL_IN1_PORT, MOTOR_RL_IN1_PIN);
            DL_GPIO_setPins(MOTOR_RL_IN2_PORT, MOTOR_RL_IN2_PIN);
        } else {
            DL_GPIO_setPins(MOTOR_RL_IN1_PORT, MOTOR_RL_IN1_PIN);
            DL_GPIO_setPins(MOTOR_RL_IN2_PORT, MOTOR_RL_IN2_PIN);
        }
        break;
    case MOTOR_REAR_RIGHT:
        if (PWM > 0) {
            DL_GPIO_setPins(MOTOR_RR_IN1_PORT, MOTOR_RR_IN1_PIN);
            DL_GPIO_clearPins(MOTOR_RR_IN2_PORT, MOTOR_RR_IN2_PIN);
        } else if (PWM < 0) {
            DL_GPIO_clearPins(MOTOR_RR_IN1_PORT, MOTOR_RR_IN1_PIN);
            DL_GPIO_setPins(MOTOR_RR_IN2_PORT, MOTOR_RR_IN2_PIN);
        } else {
            DL_GPIO_setPins(MOTOR_RR_IN1_PORT, MOTOR_RR_IN1_PIN);
            DL_GPIO_setPins(MOTOR_RR_IN2_PORT, MOTOR_RR_IN2_PIN);
        }
        break;
    default:
        break;
    }
}

/***************************************************************************************************
*   函 数 名: Motor_SetPWM
*   入口参数: PWM — 带符号速度值, motor_position — 电机位置
*   返 回 值: 无
*   函数功能: 设置单个电机的速度和方向(PWM>0正转,<0反转,=0刹车)
****************************************************************************************************/
void Motor_SetPWM(int16_t PWM, uint8_t motor_position)
{
    if (PWM > PWM_MAX)  PWM = PWM_MAX;
    if (PWM < -PWM_MAX) PWM = -PWM_MAX;

    Motor_SetDirection(PWM, motor_position);
    PWM_SetCompare(abs(PWM), motor_position);
}

/***************************************************************************************************
*   函 数 名: Motor_SetAllPWM
*   入口参数: 四个电机的速度值
*   返 回 值: 无
*   函数功能: 统一设置四个电机的PWM
****************************************************************************************************/
void Motor_SetAllPWM(int16_t front_left, int16_t front_right,
    int16_t rear_left, int16_t rear_right)
{
    Motor_SetPWM(front_left, MOTOR_FRONT_LEFT);
    Motor_SetPWM(front_right, MOTOR_FRONT_RIGHT);
    Motor_SetPWM(rear_left, MOTOR_REAR_LEFT);
    Motor_SetPWM(rear_right, MOTOR_REAR_RIGHT);
}
