/*
 *  Description:
 *    Detect UDC adapter type:
 *      0: UDB-13-Flex
 *      1: Joker-Flex
 *      2: UDB-6-Flex
 *      3: UDB-17-Flex
 *      4: UDB-12-Flex
 *      5: Unknown
 *      6: None
 */
#include "adapter_detector.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "adc.h"

// Use PF4 (AF: ADC3_INP9)
#define ADAPTER_DETECTOR_ADC_CHANNEL ADC_CHANNEL_9

// The value is selected based on the average of the desired ADC value
// of adapter type i and adapter type i+1
static const uint16_t k_adapter_type_adc_map[] =
{
    [ADAPTER_UDB_13_FLEX]   = 54102,
    [ADAPTER_JOKER_FLEX]    = 50487,
    [ADAPTER_UDB_6_FLEX]    = 46871,
    [ADAPTER_UDB_17_FLEX]   = 43470,
    [ADAPTER_UDB_12_FLEX]   = 40147,
    [ADAPTER_TYPE_5]        = 36988,
    [ADAPTER_TYPE_6]        = 33697,
    [ADAPTER_TYPE_7]        = 30231,
    [ADAPTER_TYPE_8]        = 26850,
    [ADAPTER_TYPE_9]        = 23526,
    [ADAPTER_TYPE_10]       = 20285,
    [ADAPTER_TYPE_11]       = 17164,
    [ADAPTER_TYPE_12]       = 14227,
    [ADAPTER_TYPE_13]       = 11040,
    [ADAPTER_TYPE_14]       = 7567,
    [ADAPTER_TYPE_15]       = 5904,
    [ADAPTER_UNKNOWN]       = 0xff,
    [ADAPTER_NONE]          = 0,
};

static adapter_type_t s_adapter_type_adc;

void adapter_detector_update_adapter_type_adc(void)
{
    uint32_t adc = adc_read_channel(0, ADAPTER_DETECTOR_ADC_CHANNEL, 0);

    uint8_t adapter_type_adc_map_size = sizeof(k_adapter_type_adc_map) / sizeof(uint16_t);
    for (int8_t adapter_type = 0; adapter_type < adapter_type_adc_map_size; ++adapter_type)
    {
        if (adc > k_adapter_type_adc_map[adapter_type])
        {
            s_adapter_type_adc = adapter_type;
            break;
        }
    }
}

adapter_type_t adapter_detector_get_adapter_type_adc(void)
{
    return s_adapter_type_adc;
}
