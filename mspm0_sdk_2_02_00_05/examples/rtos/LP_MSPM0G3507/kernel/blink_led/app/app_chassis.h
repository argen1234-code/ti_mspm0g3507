#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "bsp_board.h"
#include "bsp_jy901s.h"
#include "pid.h"

#define chassis_board_task 1

/* ---- 底盘电机速度PID参数 ---- */
#define MOTOR_SPEED_PID_KP       200.0f
#define MOTOR_SPEED_PID_KI       3.0f
#define MOTOR_SPEED_PID_KD       10.0f
#define MOTOR_SPEED_PID_MAX_OUT  7999.0f
#define MOTOR_SPEED_PID_MAX_IOUT 1500.0f

/* ============================================================
 *  底盘电机 (内嵌速度 PID 控制器)
 * ============================================================ */

typedef struct {
    float   speed;       /* 当前转速 (rpm, 编码器反馈) */
    float   speed_set;   /* 目标转速 (rpm) */
    PID_t   speed_pid;   /* 速度环 PID 控制器 */
    int32_t pwm_out;     /* 最终 PWM 输出 (保留兼容) */
} chassis_motor_t;

/* ============================================================
 *  车辆工作模式
 * ============================================================ */

typedef enum {
    CAR_MODE_STOP = 0,
    CAR_MODE_RUN,
} CarMode_t;

/* ============================================================
 *  底盘总控制结构体 — 全局唯一实例 chassis_move
 *
 *  数据流 (100Hz 循环):
 *    [输入]  encoder_cnt[2] → 编码器计数 (ISR 更新)
 *            imu            → IMU 传感器数据 (UART3 ISR → 环形缓冲 → 解析)
 *    [计算]  误差 = speed_set - speed
 *            PID_Calculate() → pwm_out
 *    [输出]  pwm_out → bsp_motor_set_pwm() → TIMA0 CCP + GPIO 方向
 * ============================================================ */

typedef struct {
    CarMode_t        mode;
    chassis_motor_t  motor[2];       /* [0]:左轮A, [1]:右轮B */

    jy901s_data_t    imu;            /* JY61P/JY901S 标准帧, UART3 接收 */

    volatile int32_t *encoder_cnt[2]; /* 指向 g_encoderA_cnt / g_encoderB_cnt */
    int32_t           encoder_last[2]; /* 上一周期计数 (计算增量 δ) */
    uint32_t          sample_time_ms;  /* 控制周期 10ms */

    int *flag_stop;                   /* 指向全局 Flag_Stop */
} chassis_move_t;

/* ============================================================
 *  外部接口
 * ============================================================ */

extern void chassis_task(void *pvParameters);

extern void chassis_mode_change(chassis_move_t *chassis);
extern void chassis_feedback_update(chassis_move_t *chassis);
extern void chassis_set_control(chassis_move_t *chassis);
extern void chassis_control_loop(chassis_move_t *chassis);
extern void chassis_send_cmd(chassis_move_t *chassis);

#endif
