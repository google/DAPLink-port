/*
 *  Description:
 *    Detect UDC adapter type:
 *      0: UDB-13-Flex
 *      1: Joker-Flex
 *      2: UDB-6-Flex
 *      3: UDB-17-Flex
 *      4: UDB-12-Flex
 *      5: Unknown
 */
#include "adapter_detector.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "adc.h"

#define ADAPTER_DETECTOR_ADC_CHANNEL ADC_CHANNEL_9

static adapter_type_t s_adapter_type_adc;

void adapter_detector_update_adapter_type_adc(void)
{
    uint32_t adc = adc_read_channel(0, ADAPTER_DETECTOR_ADC_CHANNEL, 0);

    if (adc > ADAPTER_UDB_13_FLEX_ADC_MIN)
    {
        s_adapter_type_adc = ADAPTER_UDB_13_FLEX;
    }
    else if (adc > ADAPTER_JOKER_FLEX_ADC_MIN)
    {
        s_adapter_type_adc = ADAPTER_JOKER_FLEX;
    }
    else if (adc > ADAPTER_UDB_6_FLEX_ADC_MIN)
    {
        s_adapter_type_adc = ADAPTER_UDB_6_FLEX;
    }
    else if (adc > ADAPTER_UDB_17_FLEX_ADC_MIN)
    {
        s_adapter_type_adc = ADAPTER_UDB_17_FLEX;
    }
    else if (adc > ADAPTER_UDB_12_FLEX_ADC_MIN)
    {
        s_adapter_type_adc = ADAPTER_UDB_12_FLEX;
    }
    else
    {
        s_adapter_type_adc = ADAPTER_UNKNOWN;
    }
}

adapter_type_t adapter_detector_get_adapter_type_adc(void)
{
    return s_adapter_type_adc;
}
