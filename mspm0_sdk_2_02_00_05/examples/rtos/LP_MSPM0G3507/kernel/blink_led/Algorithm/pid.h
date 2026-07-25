#ifndef PID_H
#define PID_H

#include <stddef.h>
#include <stdint.h>

/* 限幅宏: 将 input 钳位在 [-max, max] 之间 */
#define LimitMax(input, max)    \
{                               \
    if (input > max)            \
    {                           \
        input = max;            \
    }                           \
    else if (input < -max)      \
    {                           \
        input = -max;           \
    }                           \
}

/* PID 模式 */
enum PID_MODE
{
    PID_POSITION = 0    /* 位置式 PID */
};

/* PID 控制器 — 位置式速度环
 *
 * 数据流:
 *   输入: measured (当前转速, RPM), target (目标转速, RPM)
 *   计算: Error0 = target - measured
 *         ErrorInt += Error0 (积分), D_error = Error0 - Error1 (微分)
 *         Out = Kp*Error0 + Ki*ErrorInt + Kd*D_error
 *   输出: Out → motor[].pwm_out → bsp_motor_set_pwm() 占空比
 */
typedef struct {
    uint8_t mode;       /* PID_POSITION */
    float   Target;     /* 目标值 (RPM) */
    float   Actual;     /* 实测反馈值 (RPM) */
    float   Out;        /* 控制输出 (PWM 占空比) */
    float   Kp;         /* 比例增益 */
    float   Ki;         /* 积分增益 */
    float   Kd;         /* 微分增益 */
    float   Error0;     /* e(k): 当前误差 */
    float   Error1;     /* e(k-1): 上一次误差 */
    float   ErrorInt;   /* Σe: 误差积分累加 */
    float   D_error;    /* e(k)-e(k-1): 误差微分 */
    float   OutMax;     /* 输出限幅 (±) */
    float   max_iout;   /* 积分限幅 (抗饱和) */
} PID_t;

extern void  PID_init(PID_t *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);
extern float PID_Calculate(PID_t *pid, float measured, float target);

#endif
