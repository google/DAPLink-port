/**
 * @file    IO_Config.h
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __IO_CONFIG_H__
#define __IO_CONFIG_H__

#include "stm32h7xx.h"
#include "compiler.h"
#include "daplink.h"

COMPILER_ASSERT(DAPLINK_HIC_ID == DAPLINK_HIC_ID_STM32H743II);

//USB control pin
#define USB_CONNECT_PORT_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()
#define USB_CONNECT_PORT_DISABLE()   __HAL_RCC_GPIOA_CLK_DISABLE()
#define USB_CONNECT_PORT             GPIOA
#define USB_CONNECT_PIN              GPIO_PIN_15
#define USB_CONNECT_ON()             (USB_CONNECT_PORT->BSRR = USB_CONNECT_PIN)
#define USB_CONNECT_OFF()            (USB_CONNECT_PORT->BSRR  = ((uint32_t)USB_CONNECT_PIN << 16))

//Connected LED

//#define CONNECTED_LED_PORT           GPIOA		//elee: LED1 on eval board
//#define CONNECTED_LED_PIN            GPIO_PIN_4  //elee: LED1 on eval board
//#define CONNECTED_LED_PIN_Bit        4

#define CONNECTED_LED_PORT           GPIOG		//elee: LED0 on UDB board
#define CONNECTED_LED_PIN            GPIO_PIN_0  //elee: LED0 on UDB board
#define CONNECTED_LED_PIN_Bit        0

// ToDo(elee):
//When bootloader, disable the target port(not used)
#define POWER_EN_PIN_PORT            UDC_DUT_USB_EN_L_PORT
#define POWER_EN_PIN                 UDC_DUT_USB_EN_L_PIN
#define POWER_EN_Bit                 UDC_DUT_USB_EN_L_PIN_Bit

// nRESET OUT Pin
// JTAG0_MCU_UDC_RESET_L, UDC0_RST_L
#define nRESET_PIN_PORT              GPIOI
#define nRESET_PIN                   GPIO_PIN_4
#define nRESET_PIN_Bit               4

// nRESET DIR Pin
// JTAG0_NRESET_DIR
#define nRESET_DIR_PIN_PORT          GPIOF
#define nRESET_DIR_PIN               GPIO_PIN_8
#define nRESET_DIR_PIN_Bit           8

//SWD
// JTAG0_MCU_UDC_TCK_SWDCLK
#define SWCLK_TCK_PIN_PORT           GPIOD
#define SWCLK_TCK_PIN                GPIO_PIN_7
#define SWCLK_TCK_PIN_Bit            7

// JTAG0_MCU_UDC_TMS_SWDIO
#define SWDIO_PIN_PORT            GPIOD
#define SWDIO_PIN                 GPIO_PIN_2
#define SWDIO_PIN_Bit             2

//#define SWDIO_OUT_PIN_PORT           GPIOD
//#define SWDIO_OUT_PIN                GPIO_PIN_2
//#define SWDIO_OUT_PIN_Bit            2

//#define SWDIO_IN_PIN_PORT            GPIOD
//#define SWDIO_IN_PIN                 GPIO_PIN_2
//#define SWDIO_IN_PIN_Bit             2

// SWD bidir buffer enable, OE_L_CTRL0
#define SWD_BUFFER_EN_PORT             GPIOC
#define SWD_BUFFER_EN_PIN              GPIO_PIN_8
#define SWD_BUFFER_EN_PIN_Bit          8



// ToDo(elee):  Enable TDO / SWO?
// JTAG0_MCU_UDC_TDO_SWO = PB7 (is a USART)

// ToDo(elee):  Enable TDI?  See source/hic_hal/nxp/lpc11u35/DAP_config.h:
// Need separate _IN_ and _OUT_ defines?
// JTAG0_MCU_UDC_TDI
#define TDI_IN_PIN_PORT            GPIOD
#define TDI_IN_PIN                 GPIO_PIN_10
#define TDI_IN_PIN_Bit             10


//LEDs
//USB status LED
#define RUNNING_LED_PORT             GPIOG
#define RUNNING_LED_PIN              GPIO_PIN_1
#define RUNNING_LED_Bit              1

#define PIN_HID_LED_PORT             GPIOG
#define PIN_HID_LED                  GPIO_PIN_2
#define PIN_HID_LED_Bit              2

#define PIN_CDC_LED_PORT             GPIOG
#define PIN_CDC_LED                  GPIO_PIN_3
#define PIN_CDC_LED_Bit              3

#define PIN_MSC_LED_PORT             GPIOG
#define PIN_MSC_LED                  GPIO_PIN_4
#define PIN_MSC_LED_Bit              4

// UDB version (P1 or P2) detection
#define PIN_UDB_VERSION_PORT         GPIOG
#define PIN_UDB_VERSION              GPIO_PIN_5

//UDB specific signals
#define USBHUB_SELFPWR_PORT          GPIOH
#define USBHUB_SELFPWR_PIN           GPIO_PIN_14
#define USBHUB_SELFPWR_PIN_Bit       14


//DUT USB port power switch
//VBUS_DUT_EN_L
#define UDC_DUT_USB_EN_L_PORT        GPIOH
#define UDC_DUT_USB_EN_L_PIN         GPIO_PIN_10
#define UDC_DUT_USB_EN_L_PIN_Bit     10

//External power relay control
//EXT_RELAY_EN
#define UDC_EXT_RELAY_PORT           GPIOE
#define UDC_EXT_RELAY_PIN            GPIO_PIN_11
#define UDC_EXT_RELAY_PIN_Bit        11


//GPIO LINES
//Open Drain/Collector usage: MCU IO has pulldown/up enabled.  
//External buffer set to OUT to pull low/high, and IN to be high-z
//JTAG0_MCU_UDC_RESET_L, JTAG0_NRESET_DIR
#define UDC0_RST_L_PORT               nRESET_PIN_PORT
#define UDC0_RST_L_PIN                nRESET_PIN
#define UDC0_RST_L_PIN_Bit            nRESET_PIN_Bit
#define UDC0_RST_L_DIR_PORT           nRESET_DIR_PIN_PORT
#define UDC0_RST_L_DIR_PIN            nRESET_DIR_PIN
#define UDC0_RST_L_DIR_PIN_Bit        nRESET_DIR_PIN_Bit

//MCU_BOOT0_UDC_L, BOOT0_DIR
#define UDC0_BOOT_L_PORT               GPIOE
#define UDC0_BOOT_L_PIN                GPIO_PIN_7
#define UDC0_BOOT_L_PIN_Bit            7
#define UDC0_BOOT_L_DIR_PORT           GPIOE
#define UDC0_BOOT_L_DIR_PIN            GPIO_PIN_15
#define UDC0_BOOT_L_DIR_PIN_Bit        15

//MCU_BUT_USR0_UDC_L, BUT_USR0_DIR
#define UDC0_BUTTON_L_PORT             GPIOE
#define UDC0_BUTTON_L_PIN              GPIO_PIN_8
#define UDC0_BUTTON_L_PIN_Bit          8
#define UDC0_BUTTON_L_DIR_PORT         GPIOF
#define UDC0_BUTTON_L_DIR_PIN          GPIO_PIN_2
#define UDC0_BUTTON_L_DIR_PIN_Bit      2

//JTAG1_MCU_UDC_RESET, JTAG1_RESET_DIR
#define UDC1_RST_PORT               GPIOI
#define UDC1_RST_PIN                GPIO_PIN_9
#define UDC1_RST_PIN_Bit            9
#define UDC1_RST_DIR_PORT           GPIOB
#define UDC1_RST_DIR_PIN            GPIO_PIN_4
#define UDC1_RST_DIR_PIN_Bit        4

//MCU_BOOT1_UDC, BOOT1_DIR
#define UDC1_BOOT_PORT               GPIOE
#define UDC1_BOOT_PIN                GPIO_PIN_1
#define UDC1_BOOT_PIN_Bit            1
#define UDC1_BOOT_DIR_PORT           GPIOE
#define UDC1_BOOT_DIR_PIN            GPIO_PIN_13
#define UDC1_BOOT_DIR_PIN_Bit        13

//MCU_BUT_USR1_UDC, BUT_USR1_DIR
#define UDC1_BUTTON_PORT             GPIOE
#define UDC1_BUTTON_PIN              GPIO_PIN_3
#define UDC1_BUTTON_PIN_Bit          3
#define UDC1_BUTTON_DIR_PORT         GPIOE
#define UDC1_BUTTON_DIR_PIN          GPIO_PIN_14
#define UDC1_BUTTON_DIR_PIN_Bit      14

// ehassman
// From MX_Device.h
/*-------------------------------- I2C1       --------------------------------*/
// I2C1
// #define MX_I2C1                                 1
/* Pin PB6 */
#define MX_I2C1_SCL_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_I2C1_SCL_GPIO_FM6                    __NULL
#define MX_I2C1_SCL_Pin                         PB6
#define MX_I2C1_SCL_GPIOx                       GPIOB
#define MX_I2C1_SCL_GPIO_PuPd                   GPIO_NOPULL
#define MX_I2C1_SCL_GPIO_Pin                    GPIO_PIN_6
#define MX_I2C1_SCL_GPIO_AF                     GPIO_AF4_I2C1
#define MX_I2C1_SCL_GPIO_Mode                   GPIO_MODE_AF_OD

/* Pin PB7 */
#define MX_I2C1_SDA_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_I2C1_SDA_Pin                         PB7
#define MX_I2C1_SDA_GPIOx                       GPIOB
#define MX_I2C1_SDA_GPIO_PuPd                   GPIO_NOPULL
#define MX_I2C1_SDA_GPIO_Pin                    GPIO_PIN_7
#define MX_I2C1_SDA_GPIO_AF                     GPIO_AF4_I2C1
#define MX_I2C1_SDA_GPIO_FM7                    __NULL
#define MX_I2C1_SDA_GPIO_Mode                   GPIO_MODE_AF_OD


// I2C2
#define MX_I2C2                                 1

/* Pin PF1 */
#define MX_I2C2_SCL_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_I2C2_SCL_GPIO_FM6                    __NULL
#define MX_I2C2_SCL_Pin                         PF1
#define MX_I2C2_SCL_GPIOx                       GPIOF
#define MX_I2C2_SCL_GPIO_PuPd                   GPIO_NOPULL
#define MX_I2C2_SCL_GPIO_Pin                    GPIO_PIN_1
#define MX_I2C2_SCL_GPIO_AF                     GPIO_AF4_I2C2
#define MX_I2C2_SCL_GPIO_Mode                   GPIO_MODE_AF_OD

/* Pin PF0 */
#define MX_I2C2_SDA_GPIO_Speed                  GPIO_SPEED_FREQ_LOW
#define MX_I2C2_SDA_Pin                         PF0
#define MX_I2C2_SDA_GPIOx                       GPIOF
#define MX_I2C2_SDA_GPIO_PuPd                   GPIO_NOPULL
#define MX_I2C2_SDA_GPIO_Pin                    GPIO_PIN_0
#define MX_I2C2_SDA_GPIO_AF                     GPIO_AF4_I2C2
#define MX_I2C2_SDA_GPIO_FM7                    __NULL
#define MX_I2C2_SDA_GPIO_Mode                   GPIO_MODE_AF_OD


/*---------------------------- Clock Configuration ---------------------------*/

#define MX_LSI_VALUE                            32000
#define MX_LSE_VALUE                            32768
#define MX_HSI_VALUE                            64000000
#define MX_HSE_VALUE                            25000000
#define MX_EXTERNAL_CLOCK_VALUE                 12288000
#define MX_PLLDSIFreq_Value                     500000000
#define MX_SYSCLKFreq_VALUE                     400000000
#define MX_HCLKFreq_Value                       100000000
#define MX_CortexFreq_Value                     400000000
#define MX_APB1Freq_Value                       25000000
#define MX_APB2Freq_Value                       25000000
#define MX_CECFreq_Value                        32000
#define MX_RTCFreq_Value                        32000
#define MX_USBFreq_Value                        400000000
#define MX_WatchDogFreq_Value                   32000
#define MX_DSIFreq_Value                        96000000
#define MX_DSIPHYCLKFreq_Value                  96000000
#define MX_DSITXEscFreq_Value                   20000000
#define MX_SPDIFRXFreq_Value                    400000000
#define MX_MCO1PinFreq_Value                    64000000
#define MX_MCO2PinFreq_Value                    400000000


#endif
