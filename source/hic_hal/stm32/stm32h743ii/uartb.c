/**
 * @file    uartb.c
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

#include "string.h"

#include "stm32h7xx.h"
#include "uart.h"
#include "gpio.h"
#include "util.h"
#include "circ_buf.h"
#include "IO_Config.h"

//#include "stm32h7xx_hal.h"  //elee: porting to h7

/*
// UDB: UART4, on the AUX connector (for bringup).
// pd1=UART4_TX pd0=UART4_RX
#define CDC_UART                     UART4
#define CDC_UART_ENABLE()            __HAL_RCC_UART4_CLK_ENABLE()
#define CDC_UART_DISABLE()           __HAL_RCC_UART4_CLK_DISABLE()
#define CDC_UART_IRQn                UART4_IRQn
#define CDC_UART_IRQn_Handler        UART4_IRQHandler

#define UART_ALTFUNC                 GPIO_AF8_UART4  //Select the correct alt func

#define UART_TX_PORT                 GPIOD
#define UART_TX_PIN                  GPIO_PIN_1

#define UART_RX_PORT                 GPIOD
#define UART_RX_PIN                  GPIO_PIN_0
*/

// UDB: USART2, UART1_MCU_UDC_TXD or UART1_UDC_TXD
// pd5=USART3_TX pd6=USART3_RX
#define CDC_UARTB                   USART2
#define CDC_UARTB_ENABLE()          __HAL_RCC_USART2_CLK_ENABLE()
#define CDC_UARTB_DISABLE()         __HAL_RCC_USART2_CLK_DISABLE()
#define CDC_UARTB_IRQn              USART2_IRQn
#define CDC_UARTB_IRQn_Handler      USART2_IRQHandler

#define UARTB_ALTFUNC               GPIO_AF7_USART2 //Select the correct alt func

#define UARTB_TX_PORT               GPIOD
#define UARTB_TX_PIN                GPIO_PIN_5

#define UARTB_RX_PORT               GPIOD
#define UARTB_RX_PIN                GPIO_PIN_6

//ToDo(elee): CTS/RTS not used on main UDC uarts.  How to ensure they are not used?  Just don't set the Alternate function?
// Just choose one of the possible pins for CTS/RTS for usart2 for now.
#define UARTB_CTS_PORT              GPIOD
#define UARTB_CTS_PIN               GPIO_PIN_3

#define UARTB_RTS_PORT              GPIOD
#define UARTB_RTS_PIN               GPIO_PIN_4


#define RX_OVRF_MSG         "<DAPLink:Overflow>\n"
#define RX_OVRF_MSG_SIZE    (sizeof(RX_OVRF_MSG) - 1)
#define BUFFER_SIZE         (512)

static circ_buf_t write_buffer;
static uint8_t write_buffer_data[BUFFER_SIZE];
static circ_buf_t read_buffer;
static uint8_t read_buffer_data[BUFFER_SIZE];

static UART_Configuration configuration = {
    .Baudrate = 9600,
    .DataBits = UART_DATA_BITS_8,
    .Parity = UART_PARITY_NONE,
    .StopBits = UART_STOP_BITS_1,
    .FlowControl = UART_FLOW_CONTROL_NONE,
};

extern uint32_t SystemCoreClock;



static void clear_buffers(void)
{
    circ_buf_init(&write_buffer, write_buffer_data, sizeof(write_buffer_data));
    circ_buf_init(&read_buffer, read_buffer_data, sizeof(read_buffer_data));
}

int32_t uartb_initialize(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    CDC_UARTB->CR1 &= ~(USART_ISR_TXE_TXFNF | USART_ISR_RXNE_RXFNE);
    //Disable interrupt on framing, noise, overrun errors (until they are
    // handled in IRS)
    CDC_UARTB->CR3 &= ~USART_CR3_EIE;
    clear_buffers();

    CDC_UARTB_ENABLE();

    //TX pin
    GPIO_InitStructure.Pin = UARTB_TX_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Alternate = UARTB_ALTFUNC;
    HAL_GPIO_Init(UARTB_TX_PORT, &GPIO_InitStructure);
    //RX pin
    GPIO_InitStructure.Pin = UARTB_RX_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Alternate = UARTB_ALTFUNC;
    HAL_GPIO_Init(UARTB_RX_PORT, &GPIO_InitStructure);
    //CTS pin, input
    GPIO_InitStructure.Pin = UARTB_CTS_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    //GPIO_InitStructure.Alternate = UARTB_ALTFUNC;  //cts is not used?
    HAL_GPIO_Init(UARTB_CTS_PORT, &GPIO_InitStructure);
    //RTS pin, output low
    HAL_GPIO_WritePin(UARTB_RTS_PORT, UARTB_RTS_PIN, GPIO_PIN_RESET);
    GPIO_InitStructure.Pin = UARTB_RTS_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    //GPIO_InitStructure.Alternate = UARTB_ALTFUNC;  //rts is not used?
    HAL_GPIO_Init(UARTB_RTS_PORT, &GPIO_InitStructure);

    NVIC_EnableIRQ(CDC_UARTB_IRQn);

    return 1;
}

int32_t uartb_uninitialize(void)
{
    CDC_UARTB->CR1 &= ~(USART_ISR_TXE_TXFNF | USART_ISR_RXNE_RXFNE);
    clear_buffers();
    return 1;
}

int32_t uartb_reset(void)
{
    const uint32_t cr1 = CDC_UARTB->CR1;
    CDC_UARTB->CR1 = cr1 & ~(USART_ISR_TXE_TXFNF | USART_ISR_RXNE_RXFNE);
    clear_buffers();
    CDC_UARTB->CR1 = cr1 & ~USART_ISR_TXE_TXFNF;
    return 1;
}

int32_t uartb_set_configuration(UART_Configuration *config)
{
    UART_HandleTypeDef uart_handle;
    HAL_StatusTypeDef status;

    memset(&uart_handle, 0, sizeof(uart_handle));
    uart_handle.Instance = CDC_UARTB;

    // parity
    configuration.Parity = config->Parity;
    if(config->Parity == UART_PARITY_ODD) {
        uart_handle.Init.Parity = STM32_UART_PARITY_ODD;
    } else if(config->Parity == UART_PARITY_EVEN) {
        uart_handle.Init.Parity = STM32_UART_PARITY_EVEN;
    } else if(config->Parity == UART_PARITY_NONE) {
        uart_handle.Init.Parity = STM32_UART_PARITY_NONE;
    } else {   //Other not support
        uart_handle.Init.Parity = STM32_UART_PARITY_NONE;
        configuration.Parity = UART_PARITY_NONE;
    }

    // stop bits
    configuration.StopBits = config->StopBits;
    if(config->StopBits == UART_STOP_BITS_2) {
        uart_handle.Init.StopBits = UART_STOPBITS_2;
    } else if(config->StopBits == UART_STOP_BITS_1_5) {
        uart_handle.Init.StopBits = UART_STOPBITS_2;
        configuration.StopBits = UART_STOP_BITS_2;
    } else if(config->StopBits == UART_STOP_BITS_1) {
        uart_handle.Init.StopBits = UART_STOPBITS_1;
    } else {
        uart_handle.Init.StopBits = UART_STOPBITS_1;
        configuration.StopBits = UART_STOP_BITS_1;
    }

    //Only 8 bit support
    configuration.DataBits = UART_DATA_BITS_8;
    if (uart_handle.Init.Parity == STM32_UART_PARITY_ODD || uart_handle.Init.Parity == STM32_UART_PARITY_EVEN) {
        uart_handle.Init.WordLength = UART_WORDLENGTH_9B;
    } else {
        uart_handle.Init.WordLength = UART_WORDLENGTH_8B;
    }

    // No flow control
    configuration.FlowControl = UART_FLOW_CONTROL_NONE;
    uart_handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;

    // Specified baudrate
    configuration.Baudrate = config->Baudrate;
    uart_handle.Init.BaudRate = config->Baudrate;

    // TX and RX
    uart_handle.Init.Mode = UART_MODE_TX_RX;

    // Disable uart and tx/rx interrupt
    CDC_UARTB->CR1 &= ~(USART_ISR_TXE_TXFNF | USART_ISR_RXNE_RXFNE);

    clear_buffers();

    status = HAL_UART_DeInit(&uart_handle);
    util_assert(HAL_OK == status);
    status = HAL_UART_Init(&uart_handle);
    util_assert(HAL_OK == status);
    (void)status;

    CDC_UARTB->CR1 |= USART_ISR_RXNE_RXFNE;

    return 1;
}

int32_t uartb_get_configuration(UART_Configuration *config)
{
    config->Baudrate = configuration.Baudrate;
    config->DataBits = configuration.DataBits;
    config->Parity   = configuration.Parity;
    config->StopBits = configuration.StopBits;
    config->FlowControl = UART_FLOW_CONTROL_NONE;

    return 1;
}

int32_t uartb_write_free(void)
{
    return circ_buf_count_free(&write_buffer);
}

int32_t uartb_write_data(uint8_t *data, uint16_t size)
{
    uint32_t cnt = circ_buf_write(&write_buffer, data, size);
    //USART_IT_TXE; elee: USART_IT_TXE doesn't work!  Some bit packed format to use with __HAL_USART_ENABLE_IT
    CDC_UARTB->CR1 |= USART_ISR_TXE_TXFNF;
    return cnt;
}

int32_t uartb_read_data(uint8_t *data, uint16_t size)
{
    return circ_buf_read(&read_buffer, data, size);
}

void CDC_UARTB_IRQn_Handler(void)
{
    const uint32_t sr = CDC_UARTB->ISR;

    //Rx not empty (data remaining...)
    if (sr & USART_ISR_RXNE_RXFNE) {
        uint8_t dat = CDC_UARTB->RDR; //RDR = Read Data Register
        uint32_t free = circ_buf_count_free(&read_buffer);
        if (free > RX_OVRF_MSG_SIZE) {
            circ_buf_push(&read_buffer, dat);
        } else if (RX_OVRF_MSG_SIZE == free) {
            circ_buf_write(&read_buffer, (uint8_t*)RX_OVRF_MSG, RX_OVRF_MSG_SIZE);
        } else {
            // Drop character
        }
    }

    if (sr & USART_ISR_TXE_TXFNF)  {   //USART_SR_TXE) {
        if (circ_buf_count_used(&write_buffer) > 0) {
            CDC_UARTB->TDR = circ_buf_pop(&write_buffer);
        } else {
            CDC_UARTB->CR1 &= ~USART_ISR_TXE_TXFNF;
        }
    }

    if (sr & USART_ISR_ORE) {
        //Overrun (can be seen when closing cdc_uart and lots of traffic from
        // DUT.  Clear the interrupt here to avoid getting stuck in a loop.
        //ToDo: Potentially count overruns/flag errors and report to host.
        CDC_UARTB->ICR |= USART_ICR_ORECF;
    }
}
