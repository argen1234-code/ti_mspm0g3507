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



#define CPUCLK_FREQ                                                     80000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMA0
#define PWM_0_INST_IRQHandler                                   TIMA0_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA0_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             80000000
/* GPIO defines for channel 2 */
#define GPIO_PWM_0_C2_PORT                                                 GPIOB
#define GPIO_PWM_0_C2_PIN                                         DL_GPIO_PIN_20
#define GPIO_PWM_0_C2_IOMUX                                      (IOMUX_PINCM48)
#define GPIO_PWM_0_C2_IOMUX_FUNC                     IOMUX_PINCM48_PF_TIMA0_CCP2
#define GPIO_PWM_0_C2_IDX                                    DL_TIMER_CC_2_INDEX
/* GPIO defines for channel 3 */
#define GPIO_PWM_0_C3_PORT                                                 GPIOA
#define GPIO_PWM_0_C3_PIN                                         DL_GPIO_PIN_28
#define GPIO_PWM_0_C3_IOMUX                                       (IOMUX_PINCM3)
#define GPIO_PWM_0_C3_IOMUX_FUNC                      IOMUX_PINCM3_PF_TIMA0_CCP3
#define GPIO_PWM_0_C3_IDX                                    DL_TIMER_CC_3_INDEX




/* Defines for OLED_I2C */
#define OLED_I2C_INST                                                       I2C1
#define OLED_I2C_INST_IRQHandler                                 I2C1_IRQHandler
#define OLED_I2C_INST_INT_IRQN                                     I2C1_INT_IRQn
#define OLED_I2C_BUS_SPEED_HZ                                             400000
#define GPIO_OLED_I2C_SDA_PORT                                             GPIOB
#define GPIO_OLED_I2C_SDA_PIN                                      DL_GPIO_PIN_3
#define GPIO_OLED_I2C_IOMUX_SDA                                  (IOMUX_PINCM16)
#define GPIO_OLED_I2C_IOMUX_SDA_FUNC                   IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_OLED_I2C_SCL_PORT                                             GPIOB
#define GPIO_OLED_I2C_SCL_PIN                                      DL_GPIO_PIN_2
#define GPIO_OLED_I2C_IOMUX_SCL                                  (IOMUX_PINCM15)
#define GPIO_OLED_I2C_IOMUX_SCL_FUNC                   IOMUX_PINCM15_PF_I2C1_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           40000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_0_FBRD_40_MHZ_115200_BAUD                                      (45)
/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_FREQUENCY                                           40000000
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOA
#define GPIO_UART_1_TX_PORT                                                GPIOA
#define GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_9
#define GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_8
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM20)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM19)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM20_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM19_PF_UART1_TX
#define UART_1_BAUD_RATE                                                (115200)
#define UART_1_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_1_FBRD_40_MHZ_115200_BAUD                                      (45)
/* Defines for UART_2 */
#define UART_2_INST                                                        UART2
#define UART_2_INST_FREQUENCY                                           40000000
#define UART_2_INST_IRQHandler                                  UART2_IRQHandler
#define UART_2_INST_INT_IRQN                                      UART2_INT_IRQn
#define GPIO_UART_2_RX_PORT                                                GPIOB
#define GPIO_UART_2_TX_PORT                                                GPIOB
#define GPIO_UART_2_RX_PIN                                        DL_GPIO_PIN_16
#define GPIO_UART_2_TX_PIN                                        DL_GPIO_PIN_15
#define GPIO_UART_2_IOMUX_RX                                     (IOMUX_PINCM33)
#define GPIO_UART_2_IOMUX_TX                                     (IOMUX_PINCM32)
#define GPIO_UART_2_IOMUX_RX_FUNC                      IOMUX_PINCM33_PF_UART2_RX
#define GPIO_UART_2_IOMUX_TX_FUNC                      IOMUX_PINCM32_PF_UART2_TX
#define UART_2_BAUD_RATE                                                (115200)
#define UART_2_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_2_FBRD_40_MHZ_115200_BAUD                                      (45)
/* Defines for UART_3 */
#define UART_3_INST                                                        UART3
#define UART_3_INST_FREQUENCY                                           80000000
#define UART_3_INST_IRQHandler                                  UART3_IRQHandler
#define UART_3_INST_INT_IRQN                                      UART3_INT_IRQn
#define GPIO_UART_3_RX_PORT                                                GPIOB
#define GPIO_UART_3_TX_PORT                                                GPIOA
#define GPIO_UART_3_RX_PIN                                        DL_GPIO_PIN_13
#define GPIO_UART_3_TX_PIN                                        DL_GPIO_PIN_26
#define GPIO_UART_3_IOMUX_RX                                     (IOMUX_PINCM30)
#define GPIO_UART_3_IOMUX_TX                                     (IOMUX_PINCM59)
#define GPIO_UART_3_IOMUX_RX_FUNC                      IOMUX_PINCM30_PF_UART3_RX
#define GPIO_UART_3_IOMUX_TX_FUNC                      IOMUX_PINCM59_PF_UART3_TX
#define UART_3_BAUD_RATE                                                (115200)
#define UART_3_IBRD_80_MHZ_115200_BAUD                                      (43)
#define UART_3_FBRD_80_MHZ_115200_BAUD                                      (26)





/* Port definition for Pin Group BUZZER_GPIO */
#define BUZZER_GPIO_PORT                                                 (GPIOA)

/* Defines for BUZZER: GPIOA.21 with pinCMx 46 on package pin 17 */
#define BUZZER_GPIO_BUZZER_PIN                                  (DL_GPIO_PIN_21)
#define BUZZER_GPIO_BUZZER_IOMUX                                 (IOMUX_PINCM46)
/* Defines for AIN1: GPIOA.13 with pinCMx 35 on package pin 6 */
#define AIN_AIN1_PORT                                                    (GPIOA)
#define AIN_AIN1_PIN                                            (DL_GPIO_PIN_13)
#define AIN_AIN1_IOMUX                                           (IOMUX_PINCM35)
/* Defines for AIN2: GPIOB.26 with pinCMx 57 on package pin 28 */
#define AIN_AIN2_PORT                                                    (GPIOB)
#define AIN_AIN2_PIN                                            (DL_GPIO_PIN_26)
#define AIN_AIN2_IOMUX                                           (IOMUX_PINCM57)
/* Port definition for Pin Group BIN */
#define BIN_PORT                                                         (GPIOB)

/* Defines for BIN1: GPIOB.9 with pinCMx 26 on package pin 61 */
#define BIN_BIN1_PIN                                             (DL_GPIO_PIN_9)
#define BIN_BIN1_IOMUX                                           (IOMUX_PINCM26)
/* Defines for BIN2: GPIOB.7 with pinCMx 24 on package pin 59 */
#define BIN_BIN2_PIN                                             (DL_GPIO_PIN_7)
#define BIN_BIN2_IOMUX                                           (IOMUX_PINCM24)
/* Port definition for Pin Group ENCODERA */
#define ENCODERA_PORT                                                    (GPIOB)

/* Defines for E1A: GPIOB.23 with pinCMx 51 on package pin 22 */
// groups represented: ["ENCODERB","ENCODERA"]
// pins affected: ["E2A","E2B","E1A","E1B"]
#define GPIO_MULTIPLE_GPIOB_INT_IRQN                            (GPIOB_INT_IRQn)
#define GPIO_MULTIPLE_GPIOB_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define ENCODERA_E1A_IIDX                                   (DL_GPIO_IIDX_DIO23)
#define ENCODERA_E1A_PIN                                        (DL_GPIO_PIN_23)
#define ENCODERA_E1A_IOMUX                                       (IOMUX_PINCM51)
/* Defines for E1B: GPIOB.12 with pinCMx 29 on package pin 64 */
#define ENCODERA_E1B_IIDX                                   (DL_GPIO_IIDX_DIO12)
#define ENCODERA_E1B_PIN                                        (DL_GPIO_PIN_12)
#define ENCODERA_E1B_IOMUX                                       (IOMUX_PINCM29)
/* Port definition for Pin Group ENCODERB */
#define ENCODERB_PORT                                                    (GPIOB)

/* Defines for E2A: GPIOB.4 with pinCMx 17 on package pin 52 */
#define ENCODERB_E2A_IIDX                                    (DL_GPIO_IIDX_DIO4)
#define ENCODERB_E2A_PIN                                         (DL_GPIO_PIN_4)
#define ENCODERB_E2A_IOMUX                                       (IOMUX_PINCM17)
/* Defines for E2B: GPIOB.5 with pinCMx 18 on package pin 53 */
#define ENCODERB_E2B_IIDX                                    (DL_GPIO_IIDX_DIO5)
#define ENCODERB_E2B_PIN                                         (DL_GPIO_PIN_5)
#define ENCODERB_E2B_IOMUX                                       (IOMUX_PINCM18)
/* Defines for KEY_UP: GPIOA.14 with pinCMx 36 on package pin 7 */
#define KEYS_KEY_UP_PORT                                                 (GPIOA)
#define KEYS_KEY_UP_PIN                                         (DL_GPIO_PIN_14)
#define KEYS_KEY_UP_IOMUX                                        (IOMUX_PINCM36)
/* Defines for KEY_RIGHT: GPIOB.25 with pinCMx 56 on package pin 27 */
#define KEYS_KEY_RIGHT_PORT                                              (GPIOB)
#define KEYS_KEY_RIGHT_PIN                                      (DL_GPIO_PIN_25)
#define KEYS_KEY_RIGHT_IOMUX                                     (IOMUX_PINCM56)
/* Defines for KEY_CENTER: GPIOB.24 with pinCMx 52 on package pin 23 */
#define KEYS_KEY_CENTER_PORT                                             (GPIOB)
#define KEYS_KEY_CENTER_PIN                                     (DL_GPIO_PIN_24)
#define KEYS_KEY_CENTER_IOMUX                                    (IOMUX_PINCM52)
/* Defines for KEY_LEFT: GPIOA.15 with pinCMx 37 on package pin 8 */
#define KEYS_KEY_LEFT_PORT                                               (GPIOA)
#define KEYS_KEY_LEFT_PIN                                       (DL_GPIO_PIN_15)
#define KEYS_KEY_LEFT_IOMUX                                      (IOMUX_PINCM37)
/* Defines for KEY_DOWN: GPIOA.17 with pinCMx 39 on package pin 10 */
#define KEYS_KEY_DOWN_PORT                                               (GPIOA)
#define KEYS_KEY_DOWN_PIN                                       (DL_GPIO_PIN_17)
#define KEYS_KEY_DOWN_IOMUX                                      (IOMUX_PINCM39)
/* Defines for C1: GPIOA.31 with pinCMx 6 on package pin 39 */
#define TRACK_C1_PORT                                                    (GPIOA)
#define TRACK_C1_PIN                                            (DL_GPIO_PIN_31)
#define TRACK_C1_IOMUX                                            (IOMUX_PINCM6)
/* Defines for C2: GPIOA.12 with pinCMx 34 on package pin 5 */
#define TRACK_C2_PORT                                                    (GPIOA)
#define TRACK_C2_PIN                                            (DL_GPIO_PIN_12)
#define TRACK_C2_IOMUX                                           (IOMUX_PINCM34)
/* Defines for C3: GPIOB.8 with pinCMx 25 on package pin 60 */
#define TRACK_C3_PORT                                                    (GPIOB)
#define TRACK_C3_PIN                                             (DL_GPIO_PIN_8)
#define TRACK_C3_IOMUX                                           (IOMUX_PINCM25)
/* Defines for C4: GPIOA.27 with pinCMx 60 on package pin 31 */
#define TRACK_C4_PORT                                                    (GPIOA)
#define TRACK_C4_PIN                                            (DL_GPIO_PIN_27)
#define TRACK_C4_IOMUX                                           (IOMUX_PINCM60)
/* Defines for C5: GPIOA.30 with pinCMx 5 on package pin 37 */
#define TRACK_C5_PORT                                                    (GPIOA)
#define TRACK_C5_PIN                                            (DL_GPIO_PIN_30)
#define TRACK_C5_IOMUX                                            (IOMUX_PINCM5)
/* Defines for C7: GPIOB.21 with pinCMx 49 on package pin 20 */
#define TRACK_C7_PORT                                                    (GPIOB)
#define TRACK_C7_PIN                                            (DL_GPIO_PIN_21)
#define TRACK_C7_IOMUX                                           (IOMUX_PINCM49)
/* Defines for C8: GPIOB.0 with pinCMx 12 on package pin 47 */
#define TRACK_C8_PORT                                                    (GPIOB)
#define TRACK_C8_PIN                                             (DL_GPIO_PIN_0)
#define TRACK_C8_IOMUX                                           (IOMUX_PINCM12)
/* Defines for LED1: GPIOB.14 with pinCMx 31 on package pin 2 */
#define LEDS_LED1_PORT                                                   (GPIOB)
#define LEDS_LED1_PIN                                           (DL_GPIO_PIN_14)
#define LEDS_LED1_IOMUX                                          (IOMUX_PINCM31)
/* Defines for LED2: GPIOB.18 with pinCMx 44 on package pin 15 */
#define LEDS_LED2_PORT                                                   (GPIOB)
#define LEDS_LED2_PIN                                           (DL_GPIO_PIN_18)
#define LEDS_LED2_IOMUX                                          (IOMUX_PINCM44)
/* Defines for LED3: GPIOA.22 with pinCMx 47 on package pin 18 */
#define LEDS_LED3_PORT                                                   (GPIOA)
#define LEDS_LED3_PIN                                           (DL_GPIO_PIN_22)
#define LEDS_LED3_IOMUX                                          (IOMUX_PINCM47)



/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_OLED_I2C_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_UART_2_init(void);
void SYSCFG_DL_UART_3_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
