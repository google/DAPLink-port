#ifndef _ADAPTER_ADC_H_INCLUDED_
#define _ADAPTER_ADC_H_INCLUDED_

#include <stdint.h>

void adc_dap_initialize(void);

void update_adc_value(void);

uint16_t get_flex_hotplug_adc_value(void);

#endif

