/**
 * @file    gpio.c
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

#include "stm32h7xx.h"
#include "DAP_config.h"
#include "gpio.h"
#include "daplink.h"
#include "util.h"

#include "stm32h7xx_hal.h"

static TIM_HandleTypeDef timer;

static void busy_wait(uint32_t cycles)
{
    volatile uint32_t i;
    i = cycles;

    while (i > 0) {
        i--;
    }
}

static uint32_t tim1_clk_div(uint32_t apb2clkdiv)
{
    switch (apb2clkdiv) {
        case RCC_APB2_DIV1:
            return 1;
        case RCC_APB2_DIV4:
            return 2;
        case RCC_APB2_DIV8:
            return 4;
        default: // RCC_CFGR_PPRE2_DIV1
            return 1;
    }
}

static void output_clock_enable(void)
{
    HAL_StatusTypeDef ret;
    RCC_ClkInitTypeDef clk_init;
    TIM_OC_InitTypeDef pwm_config;
    uint32_t unused;
    uint32_t period;
    uint32_t source_clock;

    HAL_RCC_GetClockConfig(&clk_init, &unused);

    /* Compute the period value to have TIMx counter clock equal to 8000000 Hz */
    source_clock = SystemCoreClock / tim1_clk_div(clk_init.APB2CLKDivider);
    period = (uint32_t)(source_clock / 8000000) - 1;

    /* Set TIMx instance */
    timer.Instance = TIM1;

    timer.Init.Period            = period;
    timer.Init.Prescaler         = 0;
    timer.Init.ClockDivision     = 0;
    timer.Init.CounterMode       = TIM_COUNTERMODE_UP;
    timer.Init.RepetitionCounter = 0;//period / 2;

    __HAL_RCC_TIM1_CLK_ENABLE();

    ret = HAL_TIM_PWM_DeInit(&timer);
    if (ret != HAL_OK) {
        util_assert(0);
        return;
    }

    ret = HAL_TIM_PWM_Init(&timer);
    if (ret != HAL_OK) {
        util_assert(0);
        return;
    }

    pwm_config.OCMode = TIM_OCMODE_PWM2;
    pwm_config.Pulse = 0; // TODO - make sure this isn't used
    pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwm_config.OCNPolarity = TIM_OCPOLARITY_HIGH;
    pwm_config.OCFastMode = TIM_OCFAST_DISABLE;
    pwm_config.OCIdleState = TIM_OCIDLESTATE_RESET;
    pwm_config.OCNIdleState = TIM_OCIDLESTATE_RESET;
    ret = HAL_TIM_PWM_ConfigChannel(&timer, &pwm_config, TIM_CHANNEL_1);
    if (ret != HAL_OK) {
        util_assert(0);
        return;
    }

    __HAL_TIM_SET_COMPARE(&timer, TIM_CHANNEL_1, period / 2);
    ret = HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_1);
    if (ret != HAL_OK) {
        util_assert(0);
        return;
    }

    return;
}

void gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // enable clock to ports
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();  //elee: udb led's
    __HAL_RCC_GPIOH_CLK_ENABLE();  //elee: usb hub signals, SPI, I2C
    // Enable USB connect pin
		__HAL_RCC_SYSCFG_CLK_ENABLE();   //elee: this macro maps to the same as __HAL_RCC_AFIO_CLK_ENABLE(); (in the F1) and still exists.  Try it...
    
		// Disable JTAG to free pins for other uses
    // Note - SWD is still enabled
		//ToDo: elee: this doesn't exist in the H7 hal.  Can it be skipped, or need to find a replacement?  Skip it for now...
		//__HAL_AFIO_REMAP_SWJ_NOJTAG();

    USB_CONNECT_PORT_ENABLE();
    USB_CONNECT_OFF();
	  GPIO_InitStructure.Pin = USB_CONNECT_PIN;  //elee: WHAT does this pin do?  PA15?  what board is it from?  stm429?  discovery?  MB1075.pdf doesn't have it.  I guess USB power (to detect if connected?).  Or LED to show USB connected?
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(USB_CONNECT_PORT, &GPIO_InitStructure);
    // configure LEDs
    HAL_GPIO_WritePin(RUNNING_LED_PORT, RUNNING_LED_PIN, GPIO_PIN_SET);
    GPIO_InitStructure.Pin = RUNNING_LED_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(RUNNING_LED_PORT, &GPIO_InitStructure);

    HAL_GPIO_WritePin(CONNECTED_LED_PORT, CONNECTED_LED_PIN, GPIO_PIN_SET);
    GPIO_InitStructure.Pin = CONNECTED_LED_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(CONNECTED_LED_PORT, &GPIO_InitStructure);

    HAL_GPIO_WritePin(PIN_CDC_LED_PORT, PIN_CDC_LED, GPIO_PIN_SET);
    GPIO_InitStructure.Pin = PIN_CDC_LED;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(PIN_CDC_LED_PORT, &GPIO_InitStructure);

    HAL_GPIO_WritePin(PIN_MSC_LED_PORT, PIN_MSC_LED, GPIO_PIN_SET);
    GPIO_InitStructure.Pin = PIN_MSC_LED;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(PIN_MSC_LED_PORT, &GPIO_InitStructure);

    // reset button configured as gpio open drain output with a pullup
    HAL_GPIO_WritePin(nRESET_PIN_PORT, nRESET_PIN, GPIO_PIN_SET);
    GPIO_InitStructure.Pin = nRESET_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(nRESET_PIN_PORT, &GPIO_InitStructure);

    // Turn on power to the board. When the target is unpowered
    // it holds the reset line low.
    HAL_GPIO_WritePin(POWER_EN_PIN_PORT, POWER_EN_PIN, GPIO_PIN_RESET);
    GPIO_InitStructure.Pin = POWER_EN_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(POWER_EN_PIN_PORT, &GPIO_InitStructure);

    // Setup the 8MHz MCO
    GPIO_InitStructure.Pin = GPIO_PIN_8;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    output_clock_enable();

    // Setup the USB Hub to be "self powered" (very common setting, even if not strictly compliant).
    GPIO_InitStructure.Pin = USBHUB_SELFPWR_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(USBHUB_SELFPWR_PORT, &GPIO_InitStructure);
    HAL_GPIO_WritePin(USBHUB_SELFPWR_PORT, USBHUB_SELFPWR_PIN, GPIO_PIN_RESET);

    // Enable power to DUT USB port.
    GPIO_InitStructure.Pin = VBUS_DUT_EN_L_PIN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(VBUS_DUT_EN_L_PORT, &GPIO_InitStructure);
    HAL_GPIO_WritePin(VBUS_DUT_EN_L_PORT, VBUS_DUT_EN_L_PIN, GPIO_PIN_RESET);  //enable DUT USB power


    // Let the voltage rails stabilize.  This is especailly important
    // during software resets, since the target's 3.3v rail can take
    // 20-50ms to drain.  During this time the target could be driving
    // the reset pin low, causing the bootloader to think the reset
    // button is pressed.
    // Note: With optimization set to -O2 the value 1000000 delays for ~85ms
    busy_wait(1000000);
}

void gpio_set_hid_led(gpio_led_state_t state)
{
    // LED is active low
    HAL_GPIO_WritePin(PIN_HID_LED_PORT, PIN_HID_LED, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void gpio_set_cdc_led(gpio_led_state_t state)
{
    // LED is active low
    HAL_GPIO_WritePin(PIN_CDC_LED_PORT, PIN_CDC_LED, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void gpio_set_msc_led(gpio_led_state_t state)
{
    // LED is active low
    HAL_GPIO_WritePin(PIN_MSC_LED_PORT, PIN_MSC_LED, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void gpio_toggle_LED(void)
{
    // Change pin state.
    HAL_GPIO_TogglePin( CONNECTED_LED_PORT, CONNECTED_LED_PIN);
}


uint8_t gpio_get_reset_btn_no_fwrd(void)
{
    return (nRESET_PIN_PORT->IDR & nRESET_PIN) ? 0 : 1;
}

uint8_t gpio_get_reset_btn_fwrd(void)
{
    return 0;
}


uint8_t GPIOGetButtonState(void)
{
    return 0;
}

void target_forward_reset(bool assert_reset)
{
    // Do nothing - reset is forwarded in gpio_get_sw_reset
}

void gpio_set_board_power(bool powerEnabled)
{
}