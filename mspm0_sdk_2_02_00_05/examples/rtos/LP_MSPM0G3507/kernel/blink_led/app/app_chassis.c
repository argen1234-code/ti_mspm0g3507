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
 *     │ UART3 ISR → JY901S RX buffer
 *     │ bsp_jy901s_process() → chassis_move.imu
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
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_uart.h"
#include "bsp_jy901s.h"
#include "bsp_key.h"
#include "bsp_track.h"
#include "FreeRTOS.h"
#include "task.h"

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
 *  调用方: 当前未调用, 预留供上位机 STOP 指令或安全逻辑使用
 * ============================================================ */
static void chassis_stop(chassis_move_t *chassis)
{
    chassis->motor[0].speed_set = 0;
    chassis->motor[1].speed_set = 0;

    for (uint8_t i = 0; i < 2; i++) {
        chassis->motor[i].speed_pid.ErrorInt = 0.0f;
        chassis->motor[i].speed_pid.Error1   = 0.0f;
        chassis->motor[i].speed_pid.Out      = 0.0f;
    }
}

/* ============================================================
 *  chassis_init — 底盘硬件和算法初始化 (chassis_task 启动时调用一次)
 *
 *  功能: 按依赖顺序初始化所有外设和控制结构。
 *
 *  初始化顺序:
 *    1. 编码器中断: bsp_encoder_init()
 *    2. 编码器指针绑定 + PID 初始化 (两路速度环)
 *    3. 四串口接口，其中 UART3 连接 JY61P/JY901S
 *    4. 五向按键与 8 路数字循迹接口
 *    5. IMU 数据结构清零
 * ============================================================ */
static void chassis_init(chassis_move_t *chassis)
{
    const static float speed_pid_param[3] = {
        MOTOR_SPEED_PID_KP,   /* 200.0f 比例增益 */
        MOTOR_SPEED_PID_KI,   /*   3.0f 积分增益 */
        MOTOR_SPEED_PID_KD    /*  10.0f 微分增益 */
    };

    chassis->sample_time_ms = 10;   /* 100Hz */
    chassis->mode = CAR_MODE_RUN;

    /* 编码器: 使能 GPIO 中断 → 指针绑定 → 历史清零 */
    bsp_encoder_init();

    chassis->encoder_cnt[0] = &g_encoderA_cnt;  /* 左轮编码器 */
    chassis->encoder_cnt[1] = &g_encoderB_cnt;  /* 右轮编码器 */

    chassis->encoder_last[0] = 0;               /* 首周期 δ 从零开始 */
    chassis->encoder_last[1] = 0;

    chassis->flag_stop = &Flag_Stop;            /* 全局启停标志 */

    /* 两路速度环 PID 初始化, 参数对称 */
    for (uint8_t i = 0; i < 2; i++) {
        PID_init(&chassis->motor[i].speed_pid,
                 PID_POSITION,
                 speed_pid_param,
                 MOTOR_SPEED_PID_MAX_OUT,  /* 输出限幅: ±7999 */
                 MOTOR_SPEED_PID_MAX_IOUT); /* 积分限幅: ±1500 */
    }

    /* 四个原理图串口接口 */
    bsp_uart_init();
    bsp_uart_port1_init();
    bsp_uart_port2_init();
    bsp_uart_jy901s_init();
    bsp_jy901s_init(&chassis->imu);
    bsp_key_init();
    bsp_track_init();

    debug_print("\r\n=== MSPM0 底盘启动 ===\r\n");
    debug_print("UART0: 调试, PA.10/PA.11 @ 115200\r\n");
    debug_print("UART1: PA.8/PA.9 @ 115200\r\n");
    debug_print("UART2: PB.15/PB.16 @ 115200\r\n");
    debug_print("UART3: JY61P/JY901S PA.26/PB.13 @ 115200\r\n");
    debug_print("初始化完成, 串口监听中...\r\n\r\n");
}

/* ============================================================
 *  chassis_mode_change — 运行模式切换 (预留)
 *
 *  功能: 响应上位机指令或按键, 在 STOP/RUN 等模式间切换。
 *  当前状态: 空实现, 保持 CAR_MODE_RUN 不变。
 *  预期扩展:
 *    - 读取 Flag_Stop 或串口指令
 *    - CAR_MODE_STOP → 调用 chassis_stop()
 *    - CAR_MODE_RUN   → 恢复 PID
 * ============================================================ */
void chassis_mode_change(chassis_move_t *chassis)
{
    (void)chassis;
}

/* ============================================================
 *  chassis_feedback_update — 传感器数据采集 (每周期调用)
 *
 *  功能: 读取编码器和 IMU 的实时数据, 更新底盘状态。
 *
 *  Part A — 编码器 → 转速 (两路对称):
 *    1. 读取 g_encoderA/B_cnt (原子读取 32-bit volatile, ISR 实时更新)
 *    2. 计算增量 δ = (cnt - encoder_last) * encoder_dir
 *       - 左轮 dir=+1, 右轮 dir=-1 (机械安装方向导致正反转定义相反)
 *    3. 保存 encoder_last = cnt (为本周期基准)
 *    4. bsp_encoder_calc_rpm(δ, 10ms):
 *       RPM = δ * 60000 / (编码器线数 * 倍频因子 * 减速比 * 采样时间)
 *           = δ * 60000 / (13 * 2 * 30 * 10) = δ * 7.692...
 *    5. 低通滤波: 如果 |δ|≤2 (几乎静止), 当前转速衰减为 50%
 *       避免编码器微小抖动被 PID 放大为 PWM 输出
 *
 *  Part B — IMU 数据解析:
 *    bsp_jy901s_process() 从 UART3 环形缓冲取字节并解析标准 0x55 帧,
 *    直接填充 chassis->imu 的欧拉角/加速度/陀螺等字段。
 *    环形缓冲在 ISR 中填充, 此处为消费者。
 * ============================================================ */
void chassis_feedback_update(chassis_move_t *chassis)
{
    if (chassis == NULL) return;

    /* 左轮方向 = +1, 右轮方向 = -1 (机械安装对称反向) */
    static const int8_t encoder_dir[2] = {1, -1};

    for (uint8_t i = 0; i < 2; i++) {
        /* 读取 ISR 更新的编码器计数 → 计算本周期增量 */
        int32_t cnt   = *(chassis->encoder_cnt[i]);
        int32_t delta = (cnt - chassis->encoder_last[i]) * encoder_dir[i];
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

    /* IMU 串口数据解析 (消费 UART1 环形缓冲) */
    bsp_jy901s_process(&chassis->imu);
    bsp_key_update();
}

/* ============================================================
 *  chassis_set_control — 控制量设置 (预留)
 *
 *  功能: 根据上位机指令或自动策略, 设定双轮的目标转速。
 *  当前状态: 空实现, motor[].speed_set 保持初始值 0。
 *  预期扩展:
 *    - 解析串口协议帧 → 提取左右轮目标 RPM
 *    - 设置为 chassis->motor[i].speed_set
 *    - 或实现更复杂的运动学解算 (差速/阿克曼)
 * ============================================================ */
void chassis_set_control(chassis_move_t *chassis)
{
    if (chassis == NULL) return;
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

    for (uint8_t i = 0; i < 2; i++) {
        chassis->motor[i].pwm_out = (int32_t)PID_Calculate(
            &chassis->motor[i].speed_pid,
            chassis->motor[i].speed,     /* measured: 编码器反馈 RPM */
            chassis->motor[i].speed_set); /* target:  目标 RPM (当前为 0) */
    }
}

/* ============================================================
 *  chassis_send_cmd — 电机控制指令输出 (每周期调用)
 *
 *  功能: 将 PID 计算得到的占空比指令下发给电机驱动层。
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

    bsp_motor_set_pwm(chassis->motor[0].pwm_out,   /* 左轮占空比 */
                      chassis->motor[1].pwm_out);   /* 右轮占空比 */
}

/* ============================================================
 *  chassis_task — FreeRTOS 底盘控制任务入口 (100Hz)
 *
 *  功能: 系统上电后, ChassisBoardTask 唯一执行的任务函数。
 *
 *  生命周期:
 *    1. chassis_init() — 一次性初始化 (编码器/PID/串口/IMU)
 *    2. 启动握手期 (前 30 秒):
 *       每 100 次主循环 (≈1秒) 通过 UART0 发送 "MSPM0_ALIVE N"
 *       供上位机确认下位机已启动
 *    3. 主循环 (永久):
 *       按固定顺序执行 5 个步骤:
 *         mode_change   → 模式切换 (预留)
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
    (void)pvParameters;

    chassis_step = 1;
    chassis_init(&chassis_move);
    chassis_step = 2;

    /* 启动握手: 前 30 秒定期发送 alive 心跳 */
    uint32_t startup_ticks = 0;
    const  uint32_t HANDSHAKE_MAX_TICKS = pdMS_TO_TICKS(30000);

    while (1) {
        chassis_heartbeat++;
        chassis_step = 3;

        /* 启动握手期间, 每 100 次循环 (≈1秒) 输出一次 alive */
        if (startup_ticks < HANDSHAKE_MAX_TICKS) {
            if ((chassis_heartbeat % 100) == 0) {
                debug_print("MSPM0_ALIVE %lu\r\n", (unsigned long)chassis_heartbeat);
            }
        }

        /*
         * 控制流水线 5 步 (严格按序):
         *   传感器采集 → 目标设定 → PID计算 → 指令输出
         */
        chassis_mode_change(&chassis_move);     /* 步骤1: 模式切换 */
        chassis_feedback_update(&chassis_move); /* 步骤2: 传感器 → RPM + IMU */
        chassis_set_control(&chassis_move);     /* 步骤3: 目标速度设定 */
        chassis_control_loop(&chassis_move);    /* 步骤4: PID 速度环计算 */
        chassis_send_cmd(&chassis_move);        /* 步骤5: PWM 输出到电机 */

        vTaskDelay(pdMS_TO_TICKS(10));          /* 精确 10ms 周期 */
        startup_ticks += pdMS_TO_TICKS(10);
    }
}
