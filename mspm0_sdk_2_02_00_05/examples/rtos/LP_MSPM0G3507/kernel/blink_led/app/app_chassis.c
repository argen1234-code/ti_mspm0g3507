/*
 * 底盘控制任务 — 100Hz 主控制回路
 *
 * === 数据流 (每 10ms) ===
 *
 *   编码器A/B ISR (GROUP1_IRQHandler)
 *     │ g_encoderA_cnt, g_encoderB_cnt ← GPIO 四倍频计数
 *     ▼
 *   chassis_feedback_update()
 *     │ δ = cnt - encoder_last → bsp_encoder_calc_rpm() → motor[].speed
 *     │
 *     │ UART3 ISR → JY61S RX buffer
 *     │ bsp_jy61s_process() → chassis_move.imu
 *     ▼
 *   chassis_set_control()
 *     │ 8路循迹位置 → 左右轮目标转速 / 脱线搜索 / 全0保护
 *     ▼
 *   chassis_control_loop()
 *     │ PID_Calculate(speed, speed_set) → motor[].pwm_out
 *     ▼
 *   chassis_send_cmd()
 *     │ bsp_motor_set_pwm() → TIMA0 CCP3/CCP2 占空比 + GPIO 方向
 *     ▼
 *   vTaskDelay(10ms) → 下一周期
 *
 * 引脚以 2026-07-25 原理图为准。
 */

#include "app_chassis.h"
#include "app_ZDT_task.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_uart.h"
#include "bsp_jy61s.h"
#include "bsp_buzzer.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "bsp_track.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

/* 全局唯一的底盘实例, 所有任务通过 extern 引用此实例读写状态 */
chassis_move_t chassis_move = {0};

volatile uint32_t chassis_heartbeat = 0;   /* 主循环迭代计数, 用于 alive 心跳 */
volatile uint32_t chassis_step     = 0;    /* 启动阶段标记: 1=进入init, 2=init完成, 3=主循环 */

/* ============================================================
 *  chassis_stop — 紧急停车
 *
 *  功能: 将双电机目标速度归零, 同时清零 PID 历史状态,
 *        使电机立即停止且不产生积分残留导致的抖动。
 *  实现:
 *    1. 两路 speed_set = 0 (目标速度清零)
 *    2. PID 积分累加 ErrorInt → 0 (清除历史积分)
 *    3. PID 上次误差 Error1  → 0 (下次微分从零开始)
 *    4. PID 输出 Out         → 0 (归零)
 *  调用方: 循迹全0保护或其他安全停车逻辑
 * ============================================================ */
static void chassis_stop(chassis_move_t *chassis)
{
    chassis->motor[0].speed_set = 0;
    chassis->motor[1].speed_set = 0;

    for (uint8_t i = 0; i < 2; i++) {
        chassis->motor[i].speed_pid.ErrorInt = 0.0f;
        chassis->motor[i].speed_pid.Error0   = 0.0f;
        chassis->motor[i].speed_pid.Error1   = 0.0f;
        chassis->motor[i].speed_pid.D_error  = 0.0f;
        chassis->motor[i].speed_pid.Target   = 0.0f;
        chassis->motor[i].speed_pid.Out      = 0.0f;
        chassis->motor[i].pwm_out            = 0;
    }
}

static float chassis_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void chassis_pid_reset(PID_t *pid)
{
    if (pid == NULL) return;

    pid->Target = 0.0f;
    pid->Actual = 0.0f;
    pid->Out = 0.0f;
    pid->Error0 = 0.0f;
    pid->Error1 = 0.0f;
    pid->ErrorInt = 0.0f;
    pid->D_error = 0.0f;
}

static void chassis_ball_dynamic_reset(chassis_ball_control_t *ball)
{
    if (ball == NULL) return;

    chassis_pid_reset(&ball->position_pid);
    chassis_pid_reset(&ball->velocity_pid);
    ball->filtered_velocity_cm_s = 0.0f;
    ball->predicted_position_cm = 0.0f;
    ball->target_velocity_cm_s = 0.0f;
    ball->velocity_error_cm_s = 0.0f;
    ball->stopping_distance_cm = 0.0f;
    ball->desired_acceleration_cm_s2 = 0.0f;
    ball->pid_output_degrees = 0.0f;
    ball->position_pid_applied_degrees = 0.0f;
    ball->velocity_pid_output_degrees = 0.0f;
    ball->acceleration_feedforward_degrees = 0.0f;
    ball->velocity_filter_initialized = false;
    ball->moving_away_from_center = false;
    ball->outer_fast_brake_active = false;
    ball->control_phase = BALL_CONTROL_PHASE_HOLD;
}

static void chassis_zdt_publish_position_target(chassis_move_t *chassis,
                                                float target_degrees)
{
    if (chassis == NULL) return;

    target_degrees = chassis_clampf(target_degrees,
                                    APP_ZDT_ANGLE_LOWER_DEG,
                                    APP_ZDT_ANGLE_UPPER_DEG);
    chassis->zdt.position_target_degrees = target_degrees;
    chassis->zdt.position_command_sequence++;
}

/* ============================================================
 *  chassis_init — 底盘硬件和算法初始化 (chassis_task 启动时调用一次)
 *
 *  功能: 按依赖顺序初始化所有外设和控制结构。
 *
 *  初始化顺序:
 *    1. 编码器中断: bsp_encoder_init()
 *    2. 编码器指针绑定 + PID 初始化 (两路速度环)
 *    3. 四串口接口，其中 UART3 连接 JY61S/JY61P
 *    4. 五向按键与 8 路数字循迹接口；PA31/C1 延迟 3 秒接入上拉
 *    5. IMU 数据结构清零
 * ============================================================ */
static void chassis_init(chassis_move_t *chassis)
{
    const static float ball_position_pid_param[3] = {
        BALL_POSITION_PID_KP,
        BALL_POSITION_PID_KI,
        BALL_POSITION_PID_KD
    };
    const static float ball_velocity_pid_param[3] = {
        BALL_VELOCITY_PID_KP,
        BALL_VELOCITY_PID_KI,
        BALL_VELOCITY_PID_KD
    };
    const static float speed_pid_param[3] = {
        MOTOR_SPEED_PID_KP,   /* 200.0f 比例增益 */
        MOTOR_SPEED_PID_KI,   /*   3.0f 积分增益 */
        MOTOR_SPEED_PID_KD    /*  10.0f 微分增益 */
    };

    chassis->sample_time_ms = 10;   /* 100Hz */
    chassis->mode = CAR_MODE_STOP;
    app_task_state_machine_init(&chassis->task_state);

    /* 编码器: 使能 GPIO 中断 → 指针绑定 → 历史清零 */
    bsp_encoder_init();

    chassis->encoder_cnt[0] = &g_encoderB_cnt;  /* 实际左轮: 电机B / E2 */
    chassis->encoder_cnt[1] = &g_encoderA_cnt;  /* 实际右轮: 电机A / E1 */

    chassis->encoder_last[0] = 0;               /* 首周期 δ 从零开始 */
    chassis->encoder_last[1] = 0;

    Flag_Stop = 1;
    chassis->flag_stop = &Flag_Stop;            /* 1=停止, 0=允许循迹 */

    /* 两路速度环 PID 初始化, 参数对称 */
    for (uint8_t i = 0; i < 2; i++) {
        PID_init(&chassis->motor[i].speed_pid,
                 PID_POSITION,
                 speed_pid_param,
                 MOTOR_SPEED_PID_MAX_OUT,  /* 输出限幅: ±7999 */
                 MOTOR_SPEED_PID_MAX_IOUT); /* 积分限幅: ±1500 */
    }

    /* 上电默认两轮停止，等待后续逻辑显式将 Flag_Stop 清零。 */
    PID_init(&chassis->ball.position_pid,
             PID_POSITION,
             ball_position_pid_param,
             BALL_POSITION_PID_MAX_OUT_DEG,
             BALL_POSITION_PID_MAX_IOUT);
    PID_init(&chassis->ball.velocity_pid,
             PID_POSITION,
             ball_velocity_pid_param,
             BALL_VELOCITY_PID_MAX_OUT_DEG,
             BALL_VELOCITY_PID_MAX_IOUT);
    chassis->ball.target_position_cm = BALL_POSITION_TARGET_CM;
    chassis->ball.position_error_cm = 0.0f;
    chassis->ball.measured_position_cm = 0.0f;
    chassis->ball.measured_velocity_cm_s = 0.0f;
    chassis->ball.mechanical_compensation_degrees =
        BALL_MECHANICAL_COMPENSATION_DEG;
    chassis->ball.motor_neutral_degrees = 0.0f;
    chassis->ball.motor_target_degrees = 0.0f;
    chassis->ball.last_camera_sequence = chassis->camera.sequence;
    chassis->ball.camera_stale_time_ms = BALL_CAMERA_TIMEOUT_MS;
    chassis->ball.control_updates = 0U;
    chassis->ball.enabled = true;
    chassis->ball.camera_online = false;
    chassis->ball.motor_neutral_captured = false;
    chassis_ball_dynamic_reset(&chassis->ball);

    chassis->zdt.position_control_enabled = false;
    chassis->zdt.position_waiting_feedback = false;

    chassis_stop(chassis);
    bsp_motor_set_pwm(0, 0);

    /* 四个原理图串口接口 */
    bsp_camera_init(&chassis->camera);
    bsp_uart_init();
    /* UART1 is currently unused; leave its RX interrupt disabled. */
    bsp_uart_port2_init();
    bsp_uart_imu_init();
    bsp_jy61s_init(&chassis->imu);
    bsp_buzzer_init();
    bsp_led_init();
    bsp_key_init();
    /* UART0 is camera-only; no startup/debug text is transmitted. */
}

/* ============================================================
 *  chassis_task_change - non-blocking key scan and task selection
 *
 *  功能: 响应上位机指令或按键, 在 STOP/RUN 等模式间切换。
 *  Flag_Stop=1 时强制停车；Flag_Stop=0 时允许循迹逻辑决定运行状态。
 * ============================================================ */
void chassis_task_change(chassis_move_t *chassis)
{
    if ((chassis == NULL) || (chassis->flag_stop == NULL)) {
        return;
    }

    bsp_key_update();
    if (app_task_state_machine_update(
            &chassis->task_state,
            bsp_key_is_pressed(BSP_KEY_FRONT),
            chassis->sample_time_ms)) {
        *(chassis->flag_stop) = 0;
    }

    if ((*(chassis->flag_stop) != 0) ||
        (chassis->task_state.state != APP_TASK_STATE_TASK1_TRACKING)) {
        chassis->mode = CAR_MODE_STOP;
        chassis_stop(chassis);
    }
}

/* ============================================================
 *  chassis_feedback_update — 传感器数据采集 (每周期调用)
 *
 *  功能: 读取编码器和 IMU 的实时数据, 更新底盘状态。
 *
 *  Part A — 编码器 → 转速 (两路对称):
 *    1. 读取 g_encoderA/B_cnt (原子读取 32-bit volatile, ISR 实时更新)
 *    2. 计算增量 δ = (cnt - encoder_last) * encoder_feedback_sign
 *       - 实际左轮 B/E2: sign=-1
 *       - 实际右轮 A/E1: sign=+1
 *       使两路车轮物理向前时反馈速度均为正
 *    3. 保存 encoder_last = cnt (为本周期基准)
 *    4. bsp_encoder_calc_rpm(δ, 10ms):
 *       RPM = δ * 60000 / (编码器线数 * 倍频因子 * 减速比 * 采样时间)
 *           = δ * 60000 / (13 * 2 * 30 * 10) = δ * 7.692...
 *    5. 低通滤波: 如果 |δ|≤2 (几乎静止), 当前转速衰减为 50%
 *       避免编码器微小抖动被 PID 放大为 PWM 输出
 *
 *  Part B — IMU 数据解析:
 *    bsp_jy61s_process() 从 UART3 环形缓冲取字节并解析标准 0x55 帧,
 *    直接填充 chassis->imu 的欧拉角/加速度/陀螺等字段。
 *    环形缓冲在 ISR 中填充, 此处为消费者。
 * ============================================================ */
void chassis_feedback_update(chassis_move_t *chassis)
{
    if (chassis == NULL) return;

    /* [0]=实际左轮B/E2, [1]=实际右轮A/E1 */
    static const int8_t encoder_feedback_sign[2] = {-1, 1};

    for (uint8_t i = 0; i < 2; i++) {
        /* 读取 ISR 更新的编码器计数 → 计算本周期增量 */
        int32_t cnt   = *(chassis->encoder_cnt[i]);
        int32_t delta = (cnt - chassis->encoder_last[i]) *
                        encoder_feedback_sign[i];
        chassis->encoder_last[i] = cnt;

        /* 增量 → RPM 转换 (含减速比和倍频) */
        float rpm = bsp_encoder_calc_rpm(delta, chassis->sample_time_ms);

        /* 低通滤波: 微小增量(≤2 pulse) 时衰减, 抑制静止抖动 */
        if (ABS(delta) <= 2) {
            chassis->motor[i].speed *= 0.5f;
        } else {
            chassis->motor[i].speed = rpm;
        }
    }

    /* JY61S IMU 串口数据解析 (消费 UART3 环形缓冲) */
    bsp_jy61s_process(&chassis->imu);
    (void)bsp_camera_process(&chassis->camera);
    chassis_ball_position_control(chassis);
}

void chassis_task1_finish(chassis_move_t *chassis)
{
    if ((chassis == NULL) ||
        !app_task_state_machine_finish_task1(&chassis->task_state)) {
        return;
    }

    if (chassis->flag_stop != NULL) {
        *(chassis->flag_stop) = 1;
    }
    chassis->mode = CAR_MODE_STOP;
    chassis_stop(chassis);
    bsp_motor_set_pwm(0, 0);
}

/* ============================================================
 *  chassis_set_control — 八路数字循迹外环
 *
 *  黑线为1、白底为0，位置误差范围约为 -350~+350：
 *    error < 0: 黑线位于车体左侧，降低左轮、提高右轮
 *    error > 0: 黑线位于车体右侧，提高左轮、降低右轮
 *
 *  脱线处理：
 *    1. 短时全0：按最后一次偏离方向原地搜索
 *    2. 连续全0达到 TRACK_LOST_PROTECT_MS：强制停止并清空速度PID
 *    3. 再次检测到黑线：自动退出保护并恢复循迹
 * ============================================================ */
/*
 * Camera position outer loop. The 100 Hz chassis task consumes UART0 data,
 * but recalculates only when a new valid camera frame changes sequence.
 * Camera velocity adds physical damping without differentiating repeated
 * position samples. The ZDT task consumes only the latest angle target.
 */
void chassis_ball_position_control(chassis_move_t *chassis)
{
    chassis_ball_control_t *ball;
    uint32_t camera_sequence;

    if (chassis == NULL) return;
    ball = &chassis->ball;

    if (!ball->enabled) {
        ball->camera_online = false;
        chassis_ball_dynamic_reset(ball);
        return;
    }

    if (!ball->motor_neutral_captured) {
        if (chassis->zdt.uart_motor.feedback.encoder_absolute_valid) {
            float current_degrees =
                chassis->zdt.uart_motor.feedback.encoder_absolute_degrees;

            /* The mechanism is assumed to be placed approximately level
             * before power-on. Capture, but never move to, this live angle. */
            if ((current_degrees >= APP_ZDT_ANGLE_LOWER_DEG) &&
                (current_degrees <= APP_ZDT_ANGLE_UPPER_DEG)) {
                ball->motor_neutral_degrees = current_degrees;
                ball->motor_target_degrees = current_degrees;
                ball->motor_neutral_captured = true;
                chassis->zdt.position_control_enabled = true;
                chassis_zdt_publish_position_target(chassis,
                                                    current_degrees);
            }
        }
        return;
    }

    camera_sequence = chassis->camera.sequence;
    if (camera_sequence != ball->last_camera_sequence) {
        float distance_cm;
        float target_speed_magnitude_cm_s;
        float position_sign;
        float correction_degrees;
        float correction_min_degrees;
        float correction_max_degrees;

        ball->last_camera_sequence = camera_sequence;
        ball->camera_stale_time_ms = 0U;
        ball->camera_online = true;
        ball->measured_position_cm = chassis_clampf(
            chassis->camera.position_cm,
            BSP_CAMERA_POSITION_MIN_CM,
            BSP_CAMERA_POSITION_MAX_CM);
        ball->measured_velocity_cm_s = chassis->camera.velocity_cm_s;
        ball->position_error_cm = ball->target_position_cm -
                                  ball->measured_position_cm;

        if (!ball->velocity_filter_initialized) {
            ball->filtered_velocity_cm_s = ball->measured_velocity_cm_s;
            ball->velocity_filter_initialized = true;
        } else {
            ball->filtered_velocity_cm_s =
                BALL_VELOCITY_FILTER_ALPHA * ball->filtered_velocity_cm_s +
                (1.0f - BALL_VELOCITY_FILTER_ALPHA) *
                    ball->measured_velocity_cm_s;
        }

        ball->predicted_position_cm = chassis_clampf(
            ball->measured_position_cm +
                ball->filtered_velocity_cm_s *
                    BALL_POSITION_PREDICTION_TIME_S,
            BSP_CAMERA_POSITION_MIN_CM,
            BSP_CAMERA_POSITION_MAX_CM);
        distance_cm = fabsf(ball->predicted_position_cm -
                            ball->target_position_cm);
        position_sign = (ball->predicted_position_cm >=
                         ball->target_position_cm) ? 1.0f : -1.0f;
        ball->moving_away_from_center =
            ((ball->predicted_position_cm - ball->target_position_cm) *
             ball->filtered_velocity_cm_s) > 0.0f;
        ball->stopping_distance_cm =
            (ball->filtered_velocity_cm_s *
             ball->filtered_velocity_cm_s) /
            (2.0f * BALL_TRAJECTORY_BRAKE_ACCEL_CM_S2);

        target_speed_magnitude_cm_s =
            sqrtf(2.0f * BALL_TRAJECTORY_BRAKE_ACCEL_CM_S2 * distance_cm);
        target_speed_magnitude_cm_s = chassis_clampf(
            target_speed_magnitude_cm_s,
            0.0f,
            BALL_TRAJECTORY_MAX_SPEED_CM_S);
        ball->target_velocity_cm_s =
            -position_sign * target_speed_magnitude_cm_s;
        ball->outer_fast_brake_active = false;

        if ((distance_cm <= BALL_POSITION_DEADBAND_CM) &&
            (fabsf(ball->filtered_velocity_cm_s) <=
             BALL_VELOCITY_DEADBAND_CM_S)) {
            ball->control_phase = BALL_CONTROL_PHASE_HOLD;
            ball->target_velocity_cm_s = 0.0f;
            ball->desired_acceleration_cm_s2 = 0.0f;
        } else if ((!ball->moving_away_from_center) &&
                   (distance_cm >= BALL_OVERLIMIT_POSITION_CM) &&
                   (distance_cm <= BALL_OUTER_BRAKE_START_POSITION_CM) &&
                   (fabsf(ball->filtered_velocity_cm_s) >
                    BALL_OUTER_BRAKE_TARGET_SPEED_CM_S)) {
            ball->control_phase = BALL_CONTROL_PHASE_BRAKE;
            ball->outer_fast_brake_active = true;
            ball->target_velocity_cm_s =
                -position_sign * BALL_OUTER_BRAKE_TARGET_SPEED_CM_S;
            ball->desired_acceleration_cm_s2 =
                (ball->filtered_velocity_cm_s >= 0.0f) ?
                    -BALL_OUTER_BRAKE_ACCEL_CM_S2 :
                     BALL_OUTER_BRAKE_ACCEL_CM_S2;
        } else if ((!ball->moving_away_from_center) &&
                   (ball->stopping_distance_cm >= distance_cm)) {
            ball->control_phase = BALL_CONTROL_PHASE_BRAKE;
            ball->desired_acceleration_cm_s2 =
                (ball->filtered_velocity_cm_s >= 0.0f) ?
                    -BALL_TRAJECTORY_BRAKE_ACCEL_CM_S2 :
                     BALL_TRAJECTORY_BRAKE_ACCEL_CM_S2;
        } else if (distance_cm >= BALL_OVERLIMIT_POSITION_CM) {
            ball->control_phase = BALL_CONTROL_PHASE_OVERLIMIT;
            ball->desired_acceleration_cm_s2 =
                -position_sign * BALL_OVERLIMIT_ACCEL_CM_S2;
        } else {
            ball->control_phase = BALL_CONTROL_PHASE_RECOVER;
            ball->desired_acceleration_cm_s2 =
                -position_sign * BALL_TRAJECTORY_ACCEL_CM_S2;
        }

        ball->velocity_error_cm_s = ball->target_velocity_cm_s -
                                    ball->filtered_velocity_cm_s;
        ball->pid_output_degrees = PID_Calculate(
            &ball->position_pid,
            ball->predicted_position_cm,
            ball->target_position_cm);
        ball->position_pid_applied_degrees = ball->pid_output_degrees;
        if (ball->outer_fast_brake_active) {
            ball->position_pid_applied_degrees *=
                BALL_OUTER_BRAKE_POSITION_PID_SCALE;
        }
        ball->velocity_pid_output_degrees = PID_Calculate(
            &ball->velocity_pid,
            ball->filtered_velocity_cm_s,
            ball->target_velocity_cm_s);
        ball->acceleration_feedforward_degrees =
            BALL_ACCELERATION_FEEDFORWARD_DEG_PER_CM_S2 *
            ball->desired_acceleration_cm_s2;

        /* motor_neutral_degrees is deliberately excluded from both PIDs.
         * It is used only after the coupled correction has been calculated. */
        correction_degrees = BALL_CONTROL_DIRECTION_SIGN *
            (-(ball->position_pid_applied_degrees +
               ball->mechanical_compensation_degrees) -
             ball->velocity_pid_output_degrees -
             ball->acceleration_feedforward_degrees);

        /* Camera 0.0 cm is the only balance center. These two values merely
         * convert the mechanical safety endpoints into allowable actuator
         * corrections around the captured neutral reference. */
        correction_min_degrees = APP_ZDT_ANGLE_LOWER_DEG -
                                 ball->motor_neutral_degrees;
        correction_max_degrees = APP_ZDT_ANGLE_UPPER_DEG -
                                 ball->motor_neutral_degrees;
        correction_degrees = chassis_clampf(
            correction_degrees,
            correction_min_degrees,
            correction_max_degrees);

        ball->motor_target_degrees = chassis_clampf(
            ball->motor_neutral_degrees + correction_degrees,
            APP_ZDT_ANGLE_LOWER_DEG,
            APP_ZDT_ANGLE_UPPER_DEG);
        ball->control_updates++;
        chassis_zdt_publish_position_target(
            chassis, ball->motor_target_degrees);
        return;
    }

    if (ball->camera_stale_time_ms < BALL_CAMERA_TIMEOUT_MS) {
        ball->camera_stale_time_ms += chassis->sample_time_ms;
    }
    if ((ball->camera_stale_time_ms >= BALL_CAMERA_TIMEOUT_MS) &&
        ball->camera_online) {
        /* Vision loss fails safe to the mechanical neutral angle. */
        ball->camera_online = false;
        ball->motor_target_degrees = ball->motor_neutral_degrees;
        chassis_ball_dynamic_reset(ball);
        chassis_zdt_publish_position_target(
            chassis, ball->motor_target_degrees);
    }
}

void chassis_set_control(chassis_move_t *chassis)
{
    static int16_t last_error = 0;
    static int8_t last_direction = 0;
    static uint32_t lost_time_ms = 0U;
    static bool line_was_visible = false;
    static bool line_ever_seen = false;
    bool valid = false;
    int16_t error;

    if (chassis == NULL) return;

    switch (chassis->task_state.state) {
    case APP_TASK_STATE_TASK1_TRACKING:
        /* Task 1 uses the existing eight-channel line-tracking controller. */
        break;
    case APP_TASK_STATE_IDLE:
    case APP_TASK_STATE_TASK1_FINISHED:
    default:
        chassis->mode = CAR_MODE_STOP;
        chassis_stop(chassis);
        return;
    }

    if ((chassis->flag_stop != NULL) &&
        (*(chassis->flag_stop) != 0)) {
        chassis->mode = CAR_MODE_STOP;
        chassis_stop(chassis);
        return;
    }

    error = bsp_track_line_position(&valid);

    if (valid) {
        float abs_error = (error < 0) ? (float) (-error) : (float) error;
        float base_speed = TRACK_BASE_SPEED_RPM - abs_error * 0.02f;
        float derivative = line_was_visible ? (float) (error - last_error) : 0.0f;
        float correction = TRACK_STEER_KP * (float) error +
                           TRACK_STEER_KD * derivative;

        base_speed = chassis_clampf(base_speed,
                                    TRACK_MIN_BASE_SPEED_RPM,
                                    TRACK_BASE_SPEED_RPM);
        correction = chassis_clampf(correction,
                                    -TRACK_MAX_CORRECTION_RPM,
                                     TRACK_MAX_CORRECTION_RPM);

        chassis->motor[0].speed_set = chassis_clampf(
            base_speed + correction,
            -TRACK_MAX_WHEEL_SPEED_RPM,
             TRACK_MAX_WHEEL_SPEED_RPM);
        chassis->motor[1].speed_set = chassis_clampf(
            base_speed - correction,
            -TRACK_MAX_WHEEL_SPEED_RPM,
             TRACK_MAX_WHEEL_SPEED_RPM);

        if (error < -TRACK_DIRECTION_DEADBAND) {
            last_direction = -1;
        } else if (error > TRACK_DIRECTION_DEADBAND) {
            last_direction = 1;
        }

        last_error = error;
        lost_time_ms = 0U;
        line_was_visible = true;
        line_ever_seen = true;
        chassis->mode = CAR_MODE_RUN;
        return;
    }

    line_was_visible = false;
    if (lost_time_ms < TRACK_LOST_PROTECT_MS) {
        lost_time_ms += chassis->sample_time_ms;
    }

    if (line_ever_seen && (lost_time_ms < TRACK_LOST_PROTECT_MS)) {
        int8_t recovery_direction = last_direction;

        if (recovery_direction == 0) {
            recovery_direction = (last_error < 0) ? -1 : 1;
        }

        chassis->mode = CAR_MODE_RUN;
        if (recovery_direction < 0) {
            chassis->motor[0].speed_set = -TRACK_RECOVERY_SPEED_RPM;
            chassis->motor[1].speed_set =  TRACK_RECOVERY_SPEED_RPM;
        } else {
            chassis->motor[0].speed_set =  TRACK_RECOVERY_SPEED_RPM;
            chassis->motor[1].speed_set = -TRACK_RECOVERY_SPEED_RPM;
        }
        return;
    }

    chassis->mode = CAR_MODE_STOP;
    chassis_stop(chassis);
}

/* ============================================================
 *  chassis_control_loop — PID 速度环计算 (每周期调用)
 *
 *  功能: 对左右两轮分别执行 PID 计算, 将计算出的控制量写入 pwm_out。
 *
 *  实现:
 *    PID_Calculate(speed_measured, speed_target):
 *      输入: speed (编码器反馈的当前转速), speed_set (目标转速)
 *      计算: Error0 = target - measured
 *            ErrorInt += Error0 (积分累加)
 *            D_error = Error0 - Error1 (微分)
 *            Out = Kp*Error0 + Ki*ErrorInt + Kd*D_error
 *      输出: 有符号 PWM 占空比 (-7999 ~ +7999)
 *      内部: 自动限幅积分 (max_iout) 和输出 (OutMax), 抗饱和
 *    PID_Calculate 返回 float, 强制转换为 int32_t 存储到 pwm_out
 * ============================================================ */
void chassis_control_loop(chassis_move_t *chassis)
{
    if (chassis == NULL) return;

    if (chassis->mode != CAR_MODE_RUN) {
        chassis->motor[0].pwm_out = 0;
        chassis->motor[1].pwm_out = 0;
        return;
    }

    for (uint8_t i = 0; i < 2; i++) {
        chassis->motor[i].pwm_out = (int32_t)PID_Calculate(
            &chassis->motor[i].speed_pid,
            chassis->motor[i].speed,     /* measured: 编码器反馈 RPM */
            chassis->motor[i].speed_set); /* target: 循迹外环给出的目标 RPM */
    }
}

/* ============================================================
 *  chassis_send_cmd — 电机控制指令输出 (每周期调用)
 *
 *  功能: 按左右轮实际安装极性修正 PID 占空比后下发给电机驱动层。
 *
 *  实现:
 *    bsp_motor_set_pwm(pwmA, pwmB):
 *      1. pwm > 0 → 正转方向 (AIN2/BIN2 HIGH, AIN1/BIN1 LOW)
 *         pwm < 0 → 反转方向 (AIN1/BIN1 HIGH, AIN2/BIN2 LOW)
 *      2. TIMA0 CCP3/CCP2 设置占空比 = |pwm|
 *         频率固定 10kHz, 分辨率 7999 (TIMA1 ARR)
 * ============================================================ */
void chassis_send_cmd(chassis_move_t *chassis)
{
    if (chassis == NULL) return;

    if (chassis->mode != CAR_MODE_RUN) {
        bsp_motor_set_pwm(0, 0);
        return;
    }

    /* 电机A是实际右轮(正PWM前进)，电机B是实际左轮(负PWM前进) */
    bsp_motor_set_pwm( chassis->motor[1].pwm_out,  /* 电机A / 实际右轮 */
                     -chassis->motor[0].pwm_out); /* 电机B / 实际左轮 */
}

/* ============================================================
 *  chassis_task — FreeRTOS 底盘控制任务入口 (100Hz)
 *
 *  功能: 系统上电后, ChassisBoardTask 唯一执行的任务函数。
 *
 *  生命周期:
 *    1. chassis_init() — 一次性初始化 (编码器/PID/串口/IMU)
 *    2. UART0持续接收摄像头位置、速度和校验数据
 *    3. 主循环 (永久):
 *       按固定顺序执行 5 个步骤:
 *         task_change   → 非阻塞按键扫描与任务切换
 *         feedback      → 传感器采集 (编码器+IMU)
 *         set_control   → 目标速度设定 (预留)
 *         control_loop  → PID 计算 (速度环)
 *         send_cmd      → PWM 输出 (驱动电机)
 *       vTaskDelay(10ms) → 控制频率 100Hz
 *
 *  时序保证: 5 个步骤串行执行, 每次完成后 vTaskDelay 10ms,
 *   保证相邻两次 PID 计算的间隔严格为 10ms (= sample_time_ms)。
 * ============================================================ */
void chassis_task(void *pvParameters)
{
    chassis_move_t *chassis = (chassis_move_t *)pvParameters;

    if (chassis == NULL) {
        vTaskDelete(NULL);
        return;
    }

    chassis_step = 1;
    chassis_init(chassis);
    chassis_step = 2;

    while (1) {
        chassis_heartbeat++;
        app_watchdog_task_heartbeat(&chassis->watchdog,
                                    APP_WATCHDOG_TASK_CHASSIS);
        chassis_step = 3;

        /*
         * 控制流水线 5 步 (严格按序):
         *   传感器采集 → 目标设定 → PID计算 → 指令输出
         */
        chassis_task_change(chassis);     /* 步骤1: 按键扫描与任务切换 */
        chassis_feedback_update(chassis); /* 步骤2: 传感器 → RPM + IMU */
        chassis_set_control(chassis);     /* 步骤3: 目标速度设定 */
        chassis_control_loop(chassis);    /* 步骤4: PID 速度环计算 */
        chassis_send_cmd(chassis);        /* 步骤5: PWM 输出到电机 */

        vTaskDelay(pdMS_TO_TICKS(10));          /* 精确 10ms 周期 */
    }
}
