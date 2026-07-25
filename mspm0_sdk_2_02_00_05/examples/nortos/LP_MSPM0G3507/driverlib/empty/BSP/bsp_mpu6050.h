#ifndef BSP_MPU6050_H
#define BSP_MPU6050_H

#include <stdint.h>
#include "bsp_iic.h"

/*
 * 引脚使用:
 *   PA.0 — I2C0_SDA (硬件 I2C 数据)
 *   PA.1 — I2C0_SCL (硬件 I2C 时钟)
 *   PA.7 — MPU6050 INT (DMP 数据就绪中断, 下降沿触发, GROUP1)
 */

#define MPU6050_ADDR          0x68

/* ---- MPU6050 寄存器 ---- */
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_CONFIG         0x1A
#define MPU6050_GYRO_CONFIG    0x1B
#define MPU6050_ACCEL_CONFIG   0x1C
#define MPU6050_FIFO_EN        0x23
#define MPU6050_INT_PIN_CFG    0x37
#define MPU6050_INT_ENABLE     0x38
#define MPU6050_INT_STATUS     0x3A
#define MPU6050_ACCEL_XOUT_H   0x3B
#define MPU6050_TEMP_OUT_H     0x41
#define MPU6050_GYRO_XOUT_H    0x43
#define MPU6050_USER_CTRL      0x6A
#define MPU6050_PWR_MGMT1      0x6B
#define MPU6050_PWR_MGMT2      0x6C
#define MPU6050_WHO_AM_I       0x75

/* ---- INT 引脚 ---- */
#define MPU6050_INT_PORT     GPIOA
#define MPU6050_INT_PIN      DL_GPIO_PIN_7
#define MPU6050_INT_IOMUX    ((uint32_t)IOMUX_PINCM14)

/* ============================================================
 *  IMU 数据结构
 * ============================================================ */

typedef struct {
    float x;
    float y;
    float z;
} imu_vec3_t;

typedef struct {
    imu_vec3_t gyro;
    imu_vec3_t accel;
    float roll;
    float pitch;
    float yaw;
} imu_data_t;

extern imu_data_t mpu6050_data;

/* ============================================================
 *  外部接口
 * ============================================================ */

uint8_t mpu6050_init(void);
uint8_t mpu6050_dmp_init(void);
void    mpu6050_int_handler(void);  /* 在 GROUP1_IRQHandler 中调用 */

/* ---- DMP 库依赖: I2C 读写接口 ---- */
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
void    Delay_ms(unsigned long ms);
void    delay_ms(unsigned long ms);
void    mget_ms(unsigned long *time);

#endif
