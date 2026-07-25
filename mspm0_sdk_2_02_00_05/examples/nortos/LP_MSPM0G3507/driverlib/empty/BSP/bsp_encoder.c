#include "bsp_encoder.h"

/***************************************************************************************************
*   函 数 名: Encoder_QEI_Init
*   入口参数: 无
*   返 回 值: 无
*   函数功能: 配置单路编码器 (时钟 + QEI模式 + 引脚IOMUX)
****************************************************************************************************/
static void Encoder_QEI_Init(GPTIMER_Regs *inst,
    uint32_t ccp0Iomux, uint32_t ccp0Pf,
    uint32_t ccp1Iomux, uint32_t ccp1Pf)
{
    DL_Timer_ClockConfig clkCfg = {
        .clockSel    = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale    = 0
    };

    DL_TimerG_setClockConfig(inst, &clkCfg);

    DL_GPIO_initPeripheralInputFunction(ccp0Iomux, ccp0Pf);
    DL_GPIO_initPeripheralInputFunction(ccp1Iomux, ccp1Pf);

    DL_Timer_configQEI(inst, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_0_INDEX);
    DL_Timer_configQEI(inst, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_1_INDEX);

    DL_Timer_setLoadValue(inst, 0xFFFF);
    DL_TimerG_startCounter(inst);
}

/***************************************************************************************************
*   函 数 名: Encoder_Init
*   入口参数: 无
*   返 回 值: 无
*   函数功能: 启动4路编码器定时器 (TIMG0/6/7/8 正交编码器模式)
****************************************************************************************************/
void Encoder_Init(void)
{
    Encoder_QEI_Init(ENC_FL_INST,
        ENC_FL_CCP0_IOMUX, ENC_FL_CCP0_PF,
        ENC_FL_CCP1_IOMUX, ENC_FL_CCP1_PF);
    Encoder_QEI_Init(ENC_FR_INST,
        ENC_FR_CCP0_IOMUX, ENC_FR_CCP0_PF,
        ENC_FR_CCP1_IOMUX, ENC_FR_CCP1_PF);
    Encoder_QEI_Init(ENC_RL_INST,
        ENC_RL_CCP0_IOMUX, ENC_RL_CCP0_PF,
        ENC_RL_CCP1_IOMUX, ENC_RL_CCP1_PF);
    Encoder_QEI_Init(ENC_RR_INST,
        ENC_RR_CCP0_IOMUX, ENC_RR_CCP0_PF,
        ENC_RR_CCP1_IOMUX, ENC_RR_CCP1_PF);
}

/***************************************************************************************************
*   函 数 名: Encoder_Rpm_Get
*   入口参数: wheel_position — 轮子位置索引 (WHEEL_FRONT_LEFT ~ WHEEL_REAR_RIGHT)
*             rpm — 输出转速指针
*   返 回 值: 无 (通过 rpm 指针返回)
*   函数功能: 读取指定编码器当前计数并清零, 转换为转速(rpm)后通过指针返回
*   说    明: 调用者负责按控制周期换算为实际转速
****************************************************************************************************/
void Encoder_Rpm_Get(uint8_t wheel_position, int16_t *rpm)
{
    int16_t temp;

    switch (wheel_position) {
    case WHEEL_FRONT_LEFT:
        temp = (int16_t)DL_TimerG_getTimerCount(ENC_FL_INST);
        DL_TimerG_setTimerCount(ENC_FL_INST, 0);
        break;
    case WHEEL_FRONT_RIGHT:
        temp = (int16_t)DL_TimerG_getTimerCount(ENC_FR_INST);
        DL_TimerG_setTimerCount(ENC_FR_INST, 0);
        break;
    case WHEEL_REAR_LEFT:
        temp = (int16_t)DL_TimerG_getTimerCount(ENC_RL_INST);
        DL_TimerG_setTimerCount(ENC_RL_INST, 0);
        break;
    case WHEEL_REAR_RIGHT:
        temp = (int16_t)DL_TimerG_getTimerCount(ENC_RR_INST);
        DL_TimerG_setTimerCount(ENC_RR_INST, 0);
        break;
    default:
        if (rpm != NULL) *rpm = 0;
        return;
    }

    if (rpm != NULL) *rpm = temp;
}
