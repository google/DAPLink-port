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
#define POWER_EN_PIN_PORT            VBUS_DUT_EN_L_PORT
#define POWER_EN_PIN                 VBUS_DUT_EN_L_PIN
#define POWER_EN_Bit                 VBUS_DUT_EN_L_PIN_Bit

// nRESET OUT Pin
// JTAG0_MCU_UDC_RESET_L
#define nRESET_PIN_PORT              GPIOI
#define nRESET_PIN                   GPIO_PIN_4
#define nRESET_PIN_Bit               4

//SWD
// JTAG0_MCU_UDC_TCK_SWDCLK
#define SWCLK_TCK_PIN_PORT           GPIOD
#define SWCLK_TCK_PIN                GPIO_PIN_7
#define SWCLK_TCK_PIN_Bit            7

// JTAG0_MCU_UDC_TMS_SWDIO
#define SWDIO_OUT_PIN_PORT           GPIOD
#define SWDIO_OUT_PIN                GPIO_PIN_2
#define SWDIO_OUT_PIN_Bit            2

#define SWDIO_IN_PIN_PORT            GPIOD
#define SWDIO_IN_PIN                 GPIO_PIN_2
#define SWDIO_IN_PIN_Bit             2


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


//UDB specific signals
#define USBHUB_SELFPWR_PORT          GPIOH
#define USBHUB_SELFPWR_PIN           GPIO_PIN_14
#define USBHUB_SELFPWR_PIN_Bit       14

//Power switch signal to device USB port
#define VBUS_DUT_EN_L_PORT           GPIOH
#define VBUS_DUT_EN_L_PIN            GPIO_PIN_10
#define VBUS_DUT_EN_L_PIN_Bit        10

#endif
