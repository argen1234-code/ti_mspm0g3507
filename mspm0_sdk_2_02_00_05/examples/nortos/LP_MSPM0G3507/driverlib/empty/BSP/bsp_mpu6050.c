/*
 * 引脚使用:
 *   PA.0 — I2C0_SDA (硬件 I2C 数据)
 *   PA.1 — I2C0_SCL (硬件 I2C 时钟)
 *   PA.7 — MPU6050 INT (DMP 数据就绪中断, 下降沿触发, GROUP1)
 *
 * 驱动芯片: MPU6050, I2C 接口 + DMP 姿态解算
 */

#include "bsp_mpu6050.h"
#include "../ti_msp_dl_config.h"
#include "inv_mpu.h"
#include <stdio.h>
#include <math.h>

#include <FreeRTOS.h>
#include <task.h>

imu_data_t mpu6050_data = {0};

/* ============================================================
 *  内部: 毫秒延时 (运行时用 RTOS 非阻塞延时, 初始化用 busy-wait)
 * ============================================================ */
static void mpu_delay_ms(unsigned long ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        for (unsigned long i = 0; i < ms; i++) {
            delay_cycles(32000);
        }
    }
}

/* ============================================================
 *  DMP 库依赖: 获取系统毫秒 (DMP 库需要, 用于时间戳)
 * ============================================================ */
void mget_ms(unsigned long *time)
{
    *time = 0;
}

/* ============================================================
 *  内部: MPU6050 写一个字节到寄存器
 * ============================================================ */
static uint8_t mpu_write_byte(uint8_t reg, uint8_t data)
{
    return bsp_iic_write_reg(MPU6050_ADDR, reg, &data, 1);
}

/* ============================================================
 *  内部: MPU6050 从寄存器读一个字节
 * ============================================================ */
static uint8_t mpu_read_byte(uint8_t reg)
{
    uint8_t data = 0;
    bsp_iic_read_reg(MPU6050_ADDR, reg, &data, 1);
    return data;
}

/* ============================================================
 *  DMP 库依赖: I2C 写 (供 inv_mpu.c 调用)
 * ============================================================ */
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    return bsp_iic_write_reg(addr, reg, buf, len);
}

/* ============================================================
 *  DMP 库依赖: I2C 读 (供 inv_mpu.c 调用)
 * ============================================================ */
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    return bsp_iic_read_reg(addr, reg, buf, len);
}

/* ============================================================
 *  DMP 库依赖: 毫秒延时 (供 inv_mpu.c 调用)
 * ============================================================ */
void Delay_ms(unsigned long ms)
{
    mpu_delay_ms(ms);
}

/* ============================================================
 *  DMP 运动驱动库依赖: 毫秒延时 (供 inv_mpu_dmp_motion_driver.c 调用)
 * ============================================================ */
void delay_ms(unsigned long ms)
{
    mpu_delay_ms(ms);
}

/* ============================================================
 *  MPU6050 基础初始化 (配置时钟源、量程等)
 * ============================================================ */
uint8_t mpu6050_init(void)
{
    uint8_t id;

    bsp_iic_init();

    /* 复位 MPU6050 */
    mpu_write_byte(MPU6050_PWR_MGMT1, 0x80);
    mpu_delay_ms(100);

    /* 唤醒 MPU6050 */
    mpu_write_byte(MPU6050_PWR_MGMT1, 0x00);

    /* 读取设备 ID */
    id = mpu_read_byte(MPU6050_WHO_AM_I);
    if (id != MPU6050_ADDR) {
        return 1;
    }

    /* 时钟源: PLL with X Gyro */
    mpu_write_byte(MPU6050_PWR_MGMT1, 0x01);
    /* 陀螺仪 ±2000dps */
    mpu_write_byte(MPU6050_GYRO_CONFIG, 3 << 3);
    /* 加速度计 ±2g */
    mpu_write_byte(MPU6050_ACCEL_CONFIG, 0 << 3);
    /* 采样率分频: 50Hz (1000 / (1+19) = 50) */
    mpu_write_byte(MPU6050_SMPLRT_DIV, 19);
    /* 关闭数字低通滤波 */
    mpu_write_byte(MPU6050_CONFIG, 0x06);
    /* 关闭所有中断 */
    mpu_write_byte(MPU6050_INT_ENABLE, 0x00);
    /* 关闭 FIFO 和 I2C Master */
    mpu_write_byte(MPU6050_USER_CTRL, 0x00);
    mpu_write_byte(MPU6050_FIFO_EN, 0x00);
    /* INT 引脚低电平有效 */
    mpu_write_byte(MPU6050_INT_PIN_CFG, 0x80);

    return 0;
}

/* ============================================================
 *  INT 引脚中断初始化 (PA.7, 下降沿, GROUP1)
 * ============================================================ */
static void mpu6050_int_pin_init(void)
{
    DL_GPIO_initDigitalInputFeatures(MPU6050_INT_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearInterruptStatus(MPU6050_INT_PORT, MPU6050_INT_PIN);
    DL_GPIO_enableInterrupt(MPU6050_INT_PORT, MPU6050_INT_PIN);
    DL_GPIO_setLowerPinsPolarity(MPU6050_INT_PORT, DL_GPIO_PIN_7_EDGE_FALL);
}

/* ============================================================
 *  MPU6050 DMP 初始化 (调用 DMP 库 + 硬件 INT 引脚配置)
 * ============================================================ */
uint8_t mpu6050_dmp_init(void)
{
    uint8_t res;

    res = mpu_dmp_init();
    if (res) return res;

    /* 使能 DMP 中断 */
    mpu_write_byte(MPU6050_INT_ENABLE, 0x02);

    /* 初始化 INT 引脚中断 */
    mpu6050_int_pin_init();

    return 0;
}

/* ============================================================
 *  INT 引脚中断处理 (在 GROUP1_IRQHandler 中调用)
 * ============================================================ */
void mpu6050_int_handler(void)
{
    uint32_t intp;

    intp = DL_GPIO_getEnabledInterruptStatus(MPU6050_INT_PORT,
                                              MPU6050_INT_PIN);

    if (intp & MPU6050_INT_PIN) {
        mpu_dmp_get_data(&mpu6050_data.pitch,
                         &mpu6050_data.roll,
                         &mpu6050_data.yaw);
        DL_GPIO_clearInterruptStatus(MPU6050_INT_PORT, MPU6050_INT_PIN);
    }
}
