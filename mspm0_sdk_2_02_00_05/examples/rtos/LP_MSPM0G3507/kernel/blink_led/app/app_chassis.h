#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "bsp_board.h"
#include "bsp_jy61s.h"
#include "bsp_uart.h"
#include "bsp_ZDT.h"
#include "pid.h"

#define chassis_board_task 1

/* ---- 底盘电机速度PID参数 ---- */
#define MOTOR_SPEED_PID_KP       200.0f
#define MOTOR_SPEED_PID_KI       3.0f
#define MOTOR_SPEED_PID_KD       10.0f
#define MOTOR_SPEED_PID_MAX_OUT  7999.0f
#define MOTOR_SPEED_PID_MAX_IOUT 1500.0f

/* ---- 八路数字循迹参数 ---- */
#define TRACK_BASE_SPEED_RPM          20.0f
#define TRACK_MIN_BASE_SPEED_RPM      12.0f
#define TRACK_MAX_WHEEL_SPEED_RPM     35.0f
#define TRACK_STEER_KP                0.035f
#define TRACK_STEER_KD                0.010f
#define TRACK_MAX_CORRECTION_RPM      18.0f
#define TRACK_RECOVERY_SPEED_RPM      10.0f
#define TRACK_LOST_PROTECT_MS         6000U
#define TRACK_DIRECTION_DEADBAND      25

/* ---- Camera ball-position loop (UART0, -12.5 cm .. +12.5 cm) ---- */
/* The only ball-balance center is the camera's 0.0 cm position. */
#define BALL_POSITION_TARGET_CM                (0.0f)
#define BALL_POSITION_PID_KP                   (2.20f)
#define BALL_POSITION_PID_KI                   (0.00f)
#define BALL_POSITION_PID_KD                   (2.80f)
#define BALL_POSITION_PID_MAX_OUT_DEG          \
    (APP_ZDT_ANGLE_UPPER_DEG - APP_ZDT_ANGLE_LOWER_DEG)
#define BALL_POSITION_PID_MAX_IOUT             (100.0f)

/* Position/velocity coupled recovery trajectory. */
#define BALL_VELOCITY_PID_KP                    (0.175f)
#define BALL_VELOCITY_PID_KI                    (0.0f)
#define BALL_VELOCITY_PID_KD                    (0.0f)
#define BALL_VELOCITY_PID_MAX_OUT_DEG           (12.0f)
#define BALL_VELOCITY_PID_MAX_IOUT              (0.0f)
#define BALL_TRAJECTORY_MAX_SPEED_CM_S          (28.0f)
#define BALL_TRAJECTORY_ACCEL_CM_S2             (80.0f)
#define BALL_TRAJECTORY_BRAKE_ACCEL_CM_S2       (100.0f)
#define BALL_OVERLIMIT_POSITION_CM              (4.0f)
#define BALL_OVERLIMIT_ACCEL_CM_S2              (120.0f)
#define BALL_OUTER_BRAKE_START_POSITION_CM      (6.0f)
#define BALL_OUTER_BRAKE_TARGET_SPEED_CM_S      (20.0f)
#define BALL_OUTER_BRAKE_ACCEL_CM_S2            (200.0f)
#define BALL_OUTER_BRAKE_POSITION_PID_SCALE     (0.15f)
#define BALL_POSITION_PREDICTION_TIME_S         (0.025f)
#define BALL_VELOCITY_FILTER_ALPHA              (0.60f)
#define BALL_POSITION_DEADBAND_CM               (0.20f)
#define BALL_VELOCITY_DEADBAND_CM_S             (0.50f)
#define BALL_ACCELERATION_FEEDFORWARD_DEG_PER_CM_S2 (0.01f)
/* Added on the PID side; -2.0 deg produces the verified +2.0 deg motor trim. */
#define BALL_MECHANICAL_COMPENSATION_DEG        (-3.1f)
#define BALL_CAMERA_TIMEOUT_MS                 (300U)

/* Reverse this sign if the first hardware test drives the ball away. */
#define BALL_CONTROL_DIRECTION_SIGN            (1.0f)

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

typedef enum {
    ZDT_SWEEP_FAULT_NONE = 0,
    ZDT_SWEEP_FAULT_OUTSIDE_RANGE,
    ZDT_SWEEP_FAULT_TIMEOUT,
    ZDT_SWEEP_FAULT_COMMAND
} zdt_sweep_fault_t;

typedef enum {
    BALL_CONTROL_PHASE_HOLD = 0,
    BALL_CONTROL_PHASE_RECOVER,
    BALL_CONTROL_PHASE_BRAKE,
    BALL_CONTROL_PHASE_OVERLIMIT
} ball_control_phase_t;

typedef struct {
    zdt_motor_t pulse_motor;
    zdt_uart_t uart_motor;
    volatile bool task_running;
    volatile uint32_t task_heartbeat;
    volatile zdt_result_t init_result;
    volatile zdt_result_t motion_result;
    volatile zdt_result_t poll_result;
    volatile float sweep_target_degrees;
    volatile bool sweep_test_active;
    volatile bool sweep_motion_active;
    volatile bool sweep_waiting_settle_feedback;
    volatile bool sweep_fault;
    volatile zdt_sweep_fault_t sweep_fault_reason;
    volatile uint8_t sweep_correction_attempts;
    volatile uint32_t sweep_moves_completed;
    volatile uint32_t sweep_corrections;
    volatile uint32_t sweep_settle_queries;
    volatile bool position_control_enabled;
    volatile float position_target_degrees;
    volatile uint32_t position_command_sequence;
    volatile uint32_t position_applied_sequence;
    volatile bool position_waiting_feedback;
    volatile uint32_t position_commands_sent;
    volatile uint32_t position_noop_updates;
    volatile uint32_t position_command_errors;
    volatile uint32_t position_feedback_retries;
    volatile uint32_t bootstrap_feedback_requests;
} chassis_zdt_t;

typedef struct {
    PID_t position_pid;
    PID_t velocity_pid;
    float target_position_cm;
    float position_error_cm;
    float measured_position_cm;
    float measured_velocity_cm_s;
    float filtered_velocity_cm_s;
    float predicted_position_cm;
    float target_velocity_cm_s;
    float velocity_error_cm_s;
    float stopping_distance_cm;
    float desired_acceleration_cm_s2;
    float pid_output_degrees;
    float position_pid_applied_degrees;
    float velocity_pid_output_degrees;
    float acceleration_feedforward_degrees;
    float mechanical_compensation_degrees;
    /* Actuator neutral reference; never derived from the safety-range midpoint. */
    float motor_neutral_degrees;
    float motor_target_degrees;
    uint32_t last_camera_sequence;
    uint32_t camera_stale_time_ms;
    uint32_t control_updates;
    bool enabled;
    bool camera_online;
    bool motor_neutral_captured;
    bool velocity_filter_initialized;
    bool moving_away_from_center;
    bool outer_fast_brake_active;
    ball_control_phase_t control_phase;
} chassis_ball_control_t;

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
    bsp_camera_data_t camera;        /* UART0 camera ball data */
    chassis_ball_control_t ball;     /* Camera position PID -> ZDT angle */
    chassis_motor_t  motor[2];       /* [0]:左轮B/E2, [1]:右轮A/E1 */

    jy61s_data_t     imu;            /* JY61S/JY61P 标准帧, UART3 接收 */
    chassis_zdt_t    zdt;            /* X42S脉冲/串口对象及运行状态 */

    volatile int32_t *encoder_cnt[2]; /* 指向 g_encoderA_cnt / g_encoderB_cnt */
    int32_t           encoder_last[2]; /* 上一周期计数 (计算增量 δ) */
    uint32_t          sample_time_ms;  /* 控制周期 10ms */

    int *flag_stop;                   /* 指向全局 Flag_Stop */
} chassis_move_t;

/* ============================================================
 *  外部接口
 * ============================================================ */

extern chassis_move_t chassis_move;
extern void chassis_task(void *pvParameters);

extern void chassis_mode_change(chassis_move_t *chassis);
extern void chassis_feedback_update(chassis_move_t *chassis);
extern void chassis_ball_position_control(chassis_move_t *chassis);
extern void chassis_set_control(chassis_move_t *chassis);
extern void chassis_control_loop(chassis_move_t *chassis);
extern void chassis_send_cmd(chassis_move_t *chassis);

#endif
