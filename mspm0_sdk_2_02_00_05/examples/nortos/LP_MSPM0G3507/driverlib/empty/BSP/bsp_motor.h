#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include "../ti_msp_dl_config.h"

/* PWM 最大占空比 */
#define PWM_MAX 999

/* 电机位置索引 */
enum {
    MOTOR_FRONT_LEFT = 0,
    MOTOR_FRONT_RIGHT,
    MOTOR_REAR_LEFT,
    MOTOR_REAR_RIGHT
};

/* ---- PWM 引脚 (TIMA0 CCP0~3) ---- */
#define MOTOR_FL_PWM_INST  TIMA0
#define MOTOR_FL_PWM_CH    DL_TIMER_CC_0_INDEX
#define MOTOR_FL_PWM_PORT  GPIOA
#define MOTOR_FL_PWM_PIN   DL_GPIO_PIN_8

#define MOTOR_FR_PWM_INST  TIMA0
#define MOTOR_FR_PWM_CH    DL_TIMER_CC_1_INDEX
#define MOTOR_FR_PWM_PORT  GPIOA
#define MOTOR_FR_PWM_PIN   DL_GPIO_PIN_9

#define MOTOR_RL_PWM_INST  TIMA0
#define MOTOR_RL_PWM_CH    DL_TIMER_CC_2_INDEX
#define MOTOR_RL_PWM_PORT  GPIOA
#define MOTOR_RL_PWM_PIN   DL_GPIO_PIN_15

#define MOTOR_RR_PWM_INST  TIMA0
#define MOTOR_RR_PWM_CH    DL_TIMER_CC_3_INDEX
#define MOTOR_RR_PWM_PORT  GPIOB
#define MOTOR_RR_PWM_PIN   DL_GPIO_PIN_13

/* ---- 方向引脚 (TB6612 IN1/IN2) ---- */
#define MOTOR_FL_IN1_PORT  GPIOA
#define MOTOR_FL_IN1_PIN   DL_GPIO_PIN_12
#define MOTOR_FL_IN1_IOMUX ((uint32_t)IOMUX_PINCM34)
#define MOTOR_FL_IN2_PORT  GPIOA
#define MOTOR_FL_IN2_PIN   DL_GPIO_PIN_13
#define MOTOR_FL_IN2_IOMUX ((uint32_t)IOMUX_PINCM35)

#define MOTOR_FR_IN1_PORT  GPIOA
#define MOTOR_FR_IN1_PIN   DL_GPIO_PIN_27
#define MOTOR_FR_IN1_IOMUX ((uint32_t)IOMUX_PINCM60)
#define MOTOR_FR_IN2_PORT  GPIOB
#define MOTOR_FR_IN2_PIN   DL_GPIO_PIN_6
#define MOTOR_FR_IN2_IOMUX ((uint32_t)IOMUX_PINCM23)

#define MOTOR_RL_IN1_PORT  GPIOA
#define MOTOR_RL_IN1_PIN   DL_GPIO_PIN_2
#define MOTOR_RL_IN1_IOMUX ((uint32_t)IOMUX_PINCM7)
#define MOTOR_RL_IN2_PORT  GPIOB
#define MOTOR_RL_IN2_PIN   DL_GPIO_PIN_7
#define MOTOR_RL_IN2_IOMUX ((uint32_t)IOMUX_PINCM24)

#define MOTOR_RR_IN1_PORT  GPIOA
#define MOTOR_RR_IN1_PIN   DL_GPIO_PIN_14
#define MOTOR_RR_IN1_IOMUX ((uint32_t)IOMUX_PINCM36)
#define MOTOR_RR_IN2_PORT  GPIOB
#define MOTOR_RR_IN2_PIN   DL_GPIO_PIN_17
#define MOTOR_RR_IN2_IOMUX ((uint32_t)IOMUX_PINCM43)

extern void Motor_Init(void);
extern void Motor_SetPWM(int16_t PWM, uint8_t motor_position);
extern void Motor_SetAllPWM(int16_t front_left, int16_t front_right,
    int16_t rear_left, int16_t rear_right);

#endif
