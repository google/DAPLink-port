#include "adapter_adc.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"

static __IO uint16_t s_flex_hotplug_adc_value;
static ADC_HandleTypeDef s_adc_handle;
static ADC_ChannelConfTypeDef s_chan_conf;

static void error_handler(void)
{
    while (1)
    {
    }
}

void adc_dap_initialize(void)
{
    s_adc_handle.Instance = ADC3;
    if (HAL_ADC_DeInit(&s_adc_handle) != HAL_OK)
    {
      error_handler();
    }
    s_adc_handle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
    s_adc_handle.Init.Resolution = ADC_RESOLUTION_16B;
    s_adc_handle.Init.ScanConvMode = DISABLE;

    s_adc_handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    s_adc_handle.Init.LowPowerAutoWait = DISABLE;
    s_adc_handle.Init.ContinuousConvMode = DISABLE;
    s_adc_handle.Init.NbrOfConversion = 1;
    s_adc_handle.Init.DiscontinuousConvMode = DISABLE;
    s_adc_handle.Init.NbrOfDiscConversion = 1;
    s_adc_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;

    s_adc_handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    s_adc_handle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    s_adc_handle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;

    s_adc_handle.Init.OversamplingMode = DISABLE;

    if (HAL_ADC_Init(&s_adc_handle) != HAL_OK)
    {
      error_handler();
    }

    s_chan_conf.Channel = ADC_CHANNEL_9;
    s_chan_conf.Rank = ADC_REGULAR_RANK_1;
    s_chan_conf.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
    s_chan_conf.SingleDiff = ADC_SINGLE_ENDED;
    s_chan_conf.OffsetNumber = ADC_OFFSET_NONE;
    s_chan_conf.Offset = 0;

    if (HAL_ADC_ConfigChannel(&s_adc_handle, &s_chan_conf) != HAL_OK)
    {
        error_handler();
    }

    if (HAL_ADCEx_Calibration_Start(&s_adc_handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
        error_handler();
    }
}

void update_adc_value(void)
{
    if (HAL_ADC_Start(&s_adc_handle) != HAL_OK)
    {
        error_handler();
    }

    if (HAL_ADC_PollForConversion(&s_adc_handle, 10) != HAL_OK)
    {
        error_handler();
    } 
    else
    {
        s_flex_hotplug_adc_value = HAL_ADC_GetValue(&s_adc_handle);
    }

    if (HAL_ADC_Stop(&s_adc_handle) != HAL_OK) 
    {
        error_handler();
    }
}

uint16_t get_flex_hotplug_adc_value(void)
{
  return s_flex_hotplug_adc_value;
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  __HAL_RCC_ADC3_CLK_ENABLE();
  UNUSED(hadc);
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  __HAL_RCC_ADC3_CLK_DISABLE();
  UNUSED(hadc);
}

