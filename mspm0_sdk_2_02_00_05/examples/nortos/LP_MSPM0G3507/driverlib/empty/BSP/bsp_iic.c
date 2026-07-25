/*
 * 引脚使用:
 *   PA.0 — I2C0_SDA (硬件 I2C 数据)
 *   PA.1 — I2C0_SCL (硬件 I2C 时钟)
 *
 * 使用 MSPM0 硬件 I2C0 外设, 400kHz, 32MHz MCLK
 */

#include "bsp_iic.h"
#include "../ti_msp_dl_config.h"

#define I2C_TIMEOUT_MS  10

/* ============================================================
 *  I2C 初始化 — 配置 IOMUX, 使能 I2C0, 设置 400kHz
 * ============================================================ */
void bsp_iic_init(void)
{
    /* PA.0 -> I2C0_SDA */
    DL_GPIO_initPeripheralInputFunctionFeatures(IIC_SDA_IOMUX,
        IOMUX_PINCM1_PF_I2C0_SDA,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(IIC_SDA_IOMUX);

    /* PA.1 -> I2C0_SCL */
    DL_GPIO_initPeripheralInputFunctionFeatures(IIC_SCL_IOMUX,
        IOMUX_PINCM2_PF_I2C0_SCL,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(IIC_SCL_IOMUX);

    /* 使能 I2C0 */
    DL_I2C_enablePower(I2C_0_INST);

    /*
     * 时钟配置: 32MHz / 400kHz
     * SCL_PERIOD = (1 + TPR) * (SCL_LP + SCL_HP) * INT_CLK_PRD
     * 2500ns = (1 + TPR) * (6 + 4) * 31.25ns
     * TPR = 2500 / (10 * 31.25) - 1 = 7
     */
    DL_I2C_setTimerPeriod(I2C_0_INST, 7);
}

/* ============================================================
 *  I2C 写寄存器 (设备地址 + 寄存器地址 + 数据)
 * ============================================================ */
uint8_t bsp_iic_write_reg(uint8_t dev_addr, uint8_t reg_addr,
                           uint8_t *data, uint8_t len)
{
    unsigned int cnt = len;
    uint8_t const *ptr = data;

    if (!len) return 0;

    DL_I2C_transmitControllerData(I2C_0_INST, reg_addr);
    DL_I2C_clearInterruptStatus(I2C_0_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);

    while (!(DL_I2C_getControllerStatus(I2C_0_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE));

    DL_I2C_startControllerTransfer(I2C_0_INST, dev_addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, len + 1);

    do {
        unsigned fillcnt;
        fillcnt = DL_I2C_fillControllerTXFIFO(I2C_0_INST,
                   (uint8_t *)ptr, cnt);
        cnt -= fillcnt;
        ptr += fillcnt;
    } while (!DL_I2C_getRawInterruptStatus(I2C_0_INST,
                   DL_I2C_INTERRUPT_CONTROLLER_TX_DONE));

    return 0;
}

/* ============================================================
 *  I2C 读寄存器 (设备地址 + 寄存器地址 → 数据)
 * ============================================================ */
uint8_t bsp_iic_read_reg(uint8_t dev_addr, uint8_t reg_addr,
                          uint8_t *data, uint8_t len)
{
    unsigned i = 0;

    if (!len) return 1;

    /* 先发送寄存器地址 */
    DL_I2C_transmitControllerData(I2C_0_INST, reg_addr);
    I2C_0_INST->MASTER.MCTR = I2C_MCTR_RD_ON_TXEMPTY_ENABLE;
    DL_I2C_clearInterruptStatus(I2C_0_INST,
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);

    while (!(DL_I2C_getControllerStatus(I2C_0_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE));

    DL_I2C_startControllerTransfer(I2C_0_INST, dev_addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);

    do {
        if (!DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST)) {
            if (i < len) {
                data[i] = DL_I2C_receiveControllerData(I2C_0_INST);
                i++;
            }
        }
    } while (!DL_I2C_getRawInterruptStatus(I2C_0_INST,
                   DL_I2C_INTERRUPT_CONTROLLER_RX_DONE));

    /* 清空 RX FIFO 中剩余数据 */
    while (!DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST)) {
        if (i < len) {
            data[i] = DL_I2C_receiveControllerData(I2C_0_INST);
            i++;
        }
    }

    /* 清理 */
    I2C_0_INST->MASTER.MCTR = 0;
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);

    return (i == len) ? 0 : 1;
}
