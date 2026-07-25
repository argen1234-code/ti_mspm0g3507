/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_MOTOR */
#define PWM_MOTOR_INST                                                     TIMA0
#define PWM_MOTOR_INST_IRQHandler                               TIMA0_IRQHandler
#define PWM_MOTOR_INST_INT_IRQN                                 (TIMA0_INT_IRQn)
#define PWM_MOTOR_INST_CLK_FREQ                                         16000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_C0_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C0_PIN                                      DL_GPIO_PIN_8
#define GPIO_PWM_MOTOR_C0_IOMUX                                  (IOMUX_PINCM19)
#define GPIO_PWM_MOTOR_C0_IOMUX_FUNC                 IOMUX_PINCM19_PF_TIMA0_CCP0
#define GPIO_PWM_MOTOR_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_C1_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C1_PIN                                      DL_GPIO_PIN_9
#define GPIO_PWM_MOTOR_C1_IOMUX                                  (IOMUX_PINCM20)
#define GPIO_PWM_MOTOR_C1_IOMUX_FUNC                 IOMUX_PINCM20_PF_TIMA0_CCP1
#define GPIO_PWM_MOTOR_C1_IDX                                DL_TIMER_CC_1_INDEX



/* Defines for TIMER_10ms */
#define TIMER_10ms_INST                                                  (TIMA1)
#define TIMER_10ms_INST_IRQHandler                              TIMA1_IRQHandler
#define TIMER_10ms_INST_INT_IRQN                                (TIMA1_INT_IRQn)
#define TIMER_10ms_INST_LOAD_VALUE                                        (199U)




/* Port definition for Pin Group GPIO_SOUND */
#define GPIO_SOUND_PORT                                                  (GPIOB)

/* Defines for PINB_2: GPIOB.2 with pinCMx 15 on package pin 50 */
#define GPIO_SOUND_PINB_2_PIN                                    (DL_GPIO_PIN_2)
#define GPIO_SOUND_PINB_2_IOMUX                                  (IOMUX_PINCM15)
/* Port definition for Pin Group GPIO_MOTOR_LEFT */
#define GPIO_MOTOR_LEFT_PORT                                             (GPIOA)

/* Defines for PINA_26: GPIOA.26 with pinCMx 59 on package pin 30 */
#define GPIO_MOTOR_LEFT_PINA_26_PIN                             (DL_GPIO_PIN_26)
#define GPIO_MOTOR_LEFT_PINA_26_IOMUX                            (IOMUX_PINCM59)
/* Defines for PINA_27: GPIOA.27 with pinCMx 60 on package pin 31 */
#define GPIO_MOTOR_LEFT_PINA_27_PIN                             (DL_GPIO_PIN_27)
#define GPIO_MOTOR_LEFT_PINA_27_IOMUX                            (IOMUX_PINCM60)
/* Port definition for Pin Group GPIO_MOTOR_RIGHT */
#define GPIO_MOTOR_RIGHT_PORT                                            (GPIOA)

/* Defines for PINA_24: GPIOA.24 with pinCMx 54 on package pin 25 */
#define GPIO_MOTOR_RIGHT_PINA_24_PIN                            (DL_GPIO_PIN_24)
#define GPIO_MOTOR_RIGHT_PINA_24_IOMUX                           (IOMUX_PINCM54)
/* Defines for PINA_25: GPIOA.25 with pinCMx 55 on package pin 26 */
#define GPIO_MOTOR_RIGHT_PINA_25_PIN                            (DL_GPIO_PIN_25)
#define GPIO_MOTOR_RIGHT_PINA_25_IOMUX                           (IOMUX_PINCM55)
/* Defines for PINA_15: GPIOA.15 with pinCMx 37 on package pin 8 */
#define GPIO_READ_PINA_15_PORT                                           (GPIOA)
#define GPIO_READ_PINA_15_PIN                                   (DL_GPIO_PIN_15)
#define GPIO_READ_PINA_15_IOMUX                                  (IOMUX_PINCM37)
/* Defines for PINA_16: GPIOA.16 with pinCMx 38 on package pin 9 */
#define GPIO_READ_PINA_16_PORT                                           (GPIOA)
#define GPIO_READ_PINA_16_PIN                                   (DL_GPIO_PIN_16)
#define GPIO_READ_PINA_16_IOMUX                                  (IOMUX_PINCM38)
/* Defines for PINB_18: GPIOB.18 with pinCMx 44 on package pin 15 */
#define GPIO_READ_PINB_18_PORT                                           (GPIOB)
#define GPIO_READ_PINB_18_PIN                                   (DL_GPIO_PIN_18)
#define GPIO_READ_PINB_18_IOMUX                                  (IOMUX_PINCM44)
/* Defines for PINB_19: GPIOB.19 with pinCMx 45 on package pin 16 */
#define GPIO_READ_PINB_19_PORT                                           (GPIOB)
#define GPIO_READ_PINB_19_PIN                                   (DL_GPIO_PIN_19)
#define GPIO_READ_PINB_19_IOMUX                                  (IOMUX_PINCM45)
/* Defines for PINA_21: GPIOA.21 with pinCMx 46 on package pin 17 */
#define GPIO_READ_PINA_21_PORT                                           (GPIOA)
#define GPIO_READ_PINA_21_PIN                                   (DL_GPIO_PIN_21)
#define GPIO_READ_PINA_21_IOMUX                                  (IOMUX_PINCM46)
/* Defines for PINA_22: GPIOA.22 with pinCMx 47 on package pin 18 */
#define GPIO_READ_PINA_22_PORT                                           (GPIOA)
#define GPIO_READ_PINA_22_PIN                                   (DL_GPIO_PIN_22)
#define GPIO_READ_PINA_22_IOMUX                                  (IOMUX_PINCM47)
/* Defines for PINB_20: GPIOB.20 with pinCMx 48 on package pin 19 */
#define GPIO_READ_PINB_20_PORT                                           (GPIOB)
#define GPIO_READ_PINB_20_PIN                                   (DL_GPIO_PIN_20)
#define GPIO_READ_PINB_20_IOMUX                                  (IOMUX_PINCM48)
/* Defines for PINB_24: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GPIO_READ_PINB_24_PORT                                           (GPIOB)
#define GPIO_READ_PINB_24_PIN                                   (DL_GPIO_PIN_24)
#define GPIO_READ_PINB_24_IOMUX                                  (IOMUX_PINCM52)
/* Defines for PINA_23: GPIOA.23 with pinCMx 53 on package pin 24 */
#define GPIO_READ_PINA_23_PORT                                           (GPIOA)
#define GPIO_READ_PINA_23_PIN                                   (DL_GPIO_PIN_23)
#define GPIO_READ_PINA_23_IOMUX                                  (IOMUX_PINCM53)
/* Defines for PINB_9: GPIOB.9 with pinCMx 26 on package pin 61 */
#define GPIO_READ_PINB_9_PORT                                            (GPIOB)
#define GPIO_READ_PINB_9_PIN                                     (DL_GPIO_PIN_9)
#define GPIO_READ_PINB_9_IOMUX                                   (IOMUX_PINCM26)
/* Defines for PINB_8: GPIOB.8 with pinCMx 25 on package pin 60 */
#define GPIO_READ_PINB_8_PORT                                            (GPIOB)
#define GPIO_READ_PINB_8_PIN                                     (DL_GPIO_PIN_8)
#define GPIO_READ_PINB_8_IOMUX                                   (IOMUX_PINCM25)
/* Defines for PINB_7: GPIOB.7 with pinCMx 24 on package pin 59 */
#define GPIO_READ_PINB_7_PORT                                            (GPIOB)
#define GPIO_READ_PINB_7_PIN                                     (DL_GPIO_PIN_7)
#define GPIO_READ_PINB_7_IOMUX                                   (IOMUX_PINCM24)
/* Defines for PINB_6: GPIOB.6 with pinCMx 23 on package pin 58 */
#define GPIO_KEY_PINB_6_PORT                                             (GPIOB)
#define GPIO_KEY_PINB_6_PIN                                      (DL_GPIO_PIN_6)
#define GPIO_KEY_PINB_6_IOMUX                                    (IOMUX_PINCM23)
/* Defines for PINA_2: GPIOA.2 with pinCMx 7 on package pin 42 */
#define GPIO_KEY_PINA_2_PORT                                             (GPIOA)
#define GPIO_KEY_PINA_2_PIN                                      (DL_GPIO_PIN_2)
#define GPIO_KEY_PINA_2_IOMUX                                     (IOMUX_PINCM7)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_MOTOR_init(void);
void SYSCFG_DL_TIMER_10ms_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
