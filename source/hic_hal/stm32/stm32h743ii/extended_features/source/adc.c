#include "adc.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"

static __IO uint16_t uhADCxConvertedValue = 0;
static ADC_HandleTypeDef AdcHandle;
static ADC_ChannelConfTypeDef sConfig;

static char adapter_board_info[4];

void ADC_DAP_Initialize(void)
{
  AdcHandle.Instance = ADC3;
  if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK) {
    // Error
    while (1) {}
  }
  AdcHandle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  AdcHandle.Init.Resolution = ADC_RESOLUTION_16B;
  AdcHandle.Init.ScanConvMode = DISABLE;

  AdcHandle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  AdcHandle.Init.LowPowerAutoWait = DISABLE;
  AdcHandle.Init.ContinuousConvMode = DISABLE;
  AdcHandle.Init.NbrOfConversion = 1;
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle.Init.NbrOfDiscConversion = 1;
  AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;

  AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  AdcHandle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  AdcHandle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;

  AdcHandle.Init.OversamplingMode = DISABLE;

  if (HAL_ADC_Init(&AdcHandle) != HAL_OK) {
    // Error
    while (1) {}
  }

  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
    // Error
    while (1) {}
  }

  if (HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
    // Error
    while (1) {}
  }
}

void update_ADC_value(void)
{
  if (HAL_ADC_Start(&AdcHandle) != HAL_OK) {
    // Error
    while (1) {}
  }

  if (HAL_ADC_PollForConversion(&AdcHandle, 10) != HAL_OK) {
    // Error
    while (1) {}
  } else {
    uhADCxConvertedValue = HAL_ADC_GetValue(&AdcHandle);
  }

  if (HAL_ADC_Stop(&AdcHandle) != HAL_OK) {
    // Error
    while (1) {}
  }

  if (uhADCxConvertedValue < 24000) {
    adapter_board_info[0] = 'N';
    adapter_board_info[1] = '\0';
  } else {
    adapter_board_info[0] = 'Y';
    adapter_board_info[1] = '\0';
  }
}

const char *get_adapter_board_info(void)
{
  return adapter_board_info;
}
