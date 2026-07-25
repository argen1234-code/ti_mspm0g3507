/*
 * 位置式 PID 控制器 — 电机速度环
 *
 * 数据流 (每 10ms 调用一次):
 *   PID_Calculate(speed_measured, speed_target)
 *     │ Error0 = Target - Actual
 *     │ ErrorInt += Error0  (积分累加, 抗饱和限幅 max_iout)
 *     │ D_error = Error0 - Error1  (微分)
 *     │ Out = Kp*Error0 + Ki*ErrorInt + Kd*D_error  (限幅 OutMax)
 *     │ Error1 = Error0  (保存本次误差供下次微分)
 *     └─→ motor[].pwm_out → bsp_motor_set_pwm()
 *
 * 公式: Out = Kp*e(k) + Ki*Σe + Kd*(e(k)-e(k-1))
 */

#include "pid.h"

/* PID 结构体初始化 */
void PID_init(PID_t *pid, uint8_t mode, const float PID[3], float max_out, float max_iout)
{
    if (pid == NULL || PID == NULL) return;
    pid->mode     = mode;
    pid->Kp       = PID[0];
    pid->Ki       = PID[1];
    pid->Kd       = PID[2];
    pid->OutMax   = max_out;
    pid->max_iout = max_iout;
    pid->D_error  = 0.0f;
    pid->Error0 = pid->Error1 = pid->Out = pid->ErrorInt = 0.0f;
}

/* PID 计算: 输入实测值和目标值, 返回控制量 */
float PID_Calculate(PID_t *pid, float measured, float target)
{
    if (pid->mode == PID_POSITION) {
        pid->Target  = target;
        pid->Actual  = measured;
        pid->Error0  = pid->Target - pid->Actual;       /* 当前误差 */
        pid->ErrorInt += pid->Error0;                    /* 积分累加 */
        LimitMax(pid->ErrorInt, pid->max_iout);          /* 积分限幅 */
        pid->D_error = pid->Error0 - pid->Error1;        /* 误差微分 */
        pid->Out = pid->Kp * pid->Error0
                 + pid->Ki * pid->ErrorInt
                 + pid->Kd * pid->D_error;               /* PID 公式 */
        LimitMax(pid->Out, pid->OutMax);                 /* 输出限幅 */
        pid->Error1 = pid->Error0;                       /* 保存本次误差 */
    }
    return pid->Out;
}
