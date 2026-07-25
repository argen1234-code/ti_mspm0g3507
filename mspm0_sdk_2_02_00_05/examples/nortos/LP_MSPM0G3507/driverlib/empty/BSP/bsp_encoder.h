#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "../ti_msp_dl_config.h"

enum {
    WHEEL_FRONT_LEFT = 0,
    WHEEL_FRONT_RIGHT,
    WHEEL_REAR_LEFT,
    WHEEL_REAR_RIGHT
};

/* ---- 编码器引脚 (TIMG 正交编码器模式) ---- */
/*
 * TIMG0: PA23=CCP0, PA24=CCP1
 */
#define ENC_FL_INST     TIMG0
#define ENC_FL_CCP0_IOMUX   ((uint32_t)IOMUX_PINCM53)
#define ENC_FL_CCP0_PF      IOMUX_PINCM53_PF_TIMG0_CCP0
#define ENC_FL_CCP1_IOMUX   ((uint32_t)IOMUX_PINCM54)
#define ENC_FL_CCP1_PF      IOMUX_PINCM54_PF_TIMG0_CCP1

/* TIMG6: PA21=CCP0, PA22=CCP1 */
#define ENC_FR_INST     TIMG6
#define ENC_FR_CCP0_IOMUX   ((uint32_t)IOMUX_PINCM46)
#define ENC_FR_CCP0_PF      IOMUX_PINCM46_PF_TIMG6_CCP0
#define ENC_FR_CCP1_IOMUX   ((uint32_t)IOMUX_PINCM47)
#define ENC_FR_CCP1_PF      IOMUX_PINCM47_PF_TIMG6_CCP1

/* TIMG7: PA17=CCP0, PA18=CCP1 */
#define ENC_RL_INST     TIMG7
#define ENC_RL_CCP0_IOMUX   ((uint32_t)IOMUX_PINCM39)
#define ENC_RL_CCP0_PF      IOMUX_PINCM39_PF_TIMG7_CCP0
#define ENC_RL_CCP1_IOMUX   ((uint32_t)IOMUX_PINCM40)
#define ENC_RL_CCP1_PF      IOMUX_PINCM40_PF_TIMG7_CCP1

/* TIMG8: PA29=CCP0, PA30=CCP1 */
#define ENC_RR_INST     TIMG8
#define ENC_RR_CCP0_IOMUX   ((uint32_t)IOMUX_PINCM4)
#define ENC_RR_CCP0_PF      IOMUX_PINCM4_PF_TIMG8_CCP0
#define ENC_RR_CCP1_IOMUX   ((uint32_t)IOMUX_PINCM5)
#define ENC_RR_CCP1_PF      IOMUX_PINCM5_PF_TIMG8_CCP1

extern void Encoder_Init(void);
extern void Encoder_Rpm_Get(uint8_t wheel_position, int16_t *rpm);

#endif
