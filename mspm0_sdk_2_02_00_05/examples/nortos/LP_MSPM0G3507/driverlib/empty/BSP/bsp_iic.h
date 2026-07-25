#ifndef BSP_IIC_H
#define BSP_IIC_H

#include <stdint.h>

/*
 * 引脚使用:
 *   PA.0 — I2C0_SDA (硬件 I2C 数据)
 *   PA.1 — I2C0_SCL (硬件 I2C 时钟)
 *
 * 使用 MSPM0 硬件 I2C0 外设, 400kHz, 32MHz MCLK
 */

/* ---- 引脚 IOMUX ---- */
#define IIC_SDA_IOMUX   ((uint32_t)IOMUX_PINCM1)
#define IIC_SCL_IOMUX   ((uint32_t)IOMUX_PINCM2)

/* ---- I2C0 外设实例 ---- */
#define I2C_0_INST      I2C0

/* ============================================================
 *  外部接口
 * ============================================================ */

void    bsp_iic_init(void);
uint8_t bsp_iic_write_reg(uint8_t dev_addr, uint8_t reg_addr,
                           uint8_t *data, uint8_t len);
uint8_t bsp_iic_read_reg(uint8_t dev_addr, uint8_t reg_addr,
                          uint8_t *data, uint8_t len);

#endif
