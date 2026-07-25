#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "../ti_msp_dl_config.h"
#include "pid.h"

#define CHASSIS_BOARD_TASK 1

/* ---- 速度PID参数 ---- */
#define MOTOR_SPEED_PID_KP       1.22f
#define MOTOR_SPEED_PID_KI       0.09f
#define MOTOR_SPEED_PID_KD       0.08f
#define MOTOR_SPEED_PID_MAX_OUT  120.0f
#define MOTOR_SPEED_PID_MAX_IOUT 40.0f

/* ============================================================
 *  底盘电机 + 速度PID
 * ============================================================ */

typedef struct {
    double speed;              /* 当前转速 (rpm, 编码器反馈) */
    double speed_set;          /* 目标转速 (rpm) */
    double angle;              /* 当前角度 */
    double angle_set;          /* 目标角度 */
    PID_t  speed_pid;          /* 速度环 PID 控制器 */
} chassis_motor_t;

/* ============================================================
 *  车辆工作模式
 * ============================================================ */

typedef enum {
    CAR_MODE_STOP = 0,         /* 停车 */
    CAR_MODE_REMOTE,           /* 遥控模式 (UART指令) */
    CAR_MODE_AUTO              /* 自主导航 (预留) */
} CarMode_t;

/* ============================================================
 *  Jetson/上位机下发的控制帧 (预留)
 * ============================================================ */

typedef struct {
    uint8_t mode;              /* 模式 */
    float   vx;                /* 线速度 X (m/s) */
    float   vy;                /* 线速度 Y (m/s) */
    float   vz;                /* 角速度 Z (rad/s) */
} cmd_vel_t;

/* ============================================================
 *  USB 回传数据 (预留)
 * ============================================================ */

typedef struct {
    float heading_to_target_deg;
    float current_lat;
    float current_lon;
} date_to_usb_t;

/* ============================================================
 *  底盘全向移动总控制结构体
 * ============================================================ */

typedef struct chassis_move_s {
    /* ---- 工作模式 ---- */
    CarMode_t           mode;

    /* ---- 上位机下发的 cmd_vel ---- */
    cmd_vel_t           cmd_vel;
    uint32_t            cmd_last_tick;

    /* ---- USB 回传数据 ---- */
    date_to_usb_t       date_to_usb;

    /* ---- 全向移动目标速度 (运动学分解前的合速度) ---- */
    float Vx_set;
    float Vy_set;
    float Wz_set;

    /* ---- 4 路电机 [FL:前左, FR:前右, RL:后左, RR:后右] ---- */
    chassis_motor_t     motor[4];

} chassis_move_t;

/* ============================================================
 *  外部接口
 * ============================================================ */

extern void chassis_task(void *pvParameters);
extern void Chassis_SetMode(chassis_move_t *chassis, CarMode_t mode);

/* 主循环 5 步骤 */
extern void chassis_mode_change(chassis_move_t *chassis);
extern void chassis_feedback_update(chassis_move_t *chassis);
extern void chassis_set_control(chassis_move_t *chassis);
extern void chassis_control_loop(chassis_move_t *chassis);
extern void chassis_send_cmd(chassis_move_t *chassis);

#endif
