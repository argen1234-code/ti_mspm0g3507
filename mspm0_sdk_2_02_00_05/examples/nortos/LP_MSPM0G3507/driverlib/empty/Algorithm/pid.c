#include "pid.h"

/***************************************************************************************************
*   函 数 名: PID_init
*   入口参数: pid — PID结构体指针
*             mode — PID模式 (PID_POSITION)
*             PID[3] — Kp/Ki/Kd 参数数组
*             max_out — 输出限幅
*             max_iout — 积分限幅
*   返 回 值: 无
*   函数功能: 初始化PID控制器参数
****************************************************************************************************/
void PID_init(PID_t *pid, uint8_t mode, const double PID[3], double max_out, double max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->OutMax = max_out;
    pid->max_iout = max_iout;
    pid->D_error = 0.0;
    pid->Error0 = pid->Error1 = pid->Out = pid->ErrorInt = 0.0;
}

/***************************************************************************************************
*   函 数 名: PID_Calculate
*   入口参数: pid — PID结构体指针
*             ref — 反馈值 (当前实际值)
*             set — 设定值 (目标值)
*   返 回 值: PID输出值
*   函数功能: 位置式PID计算
****************************************************************************************************/
double PID_Calculate(PID_t *pid, double ref, double set)
{
    if (pid->mode == PID_POSITION)
    {
        pid->Target = set;
        pid->Actual = ref;

        pid->Error0 = pid->Target - pid->Actual;

        pid->ErrorInt += pid->Error0;
        LimitMax(pid->ErrorInt, pid->max_iout);

        pid->D_error = pid->Error0 - pid->Error1;

        pid->Out = pid->Kp * pid->Error0 +
                   pid->Ki * pid->ErrorInt +
                   pid->Kd * pid->D_error;

        LimitMax(pid->Out, pid->OutMax);

        pid->Error1 = pid->Error0;
    }

    return pid->Out;
}
