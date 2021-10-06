#ifndef _ADAPTER_DETECTOR_H_INCLUDED_
#define _ADAPTER_DETECTOR_H_INCLUDED_

#include <stdint.h>
#include "adapter.h"

void adapter_detector_update_adapter_type_adc(void);
adapter_type_t adapter_detector_get_adapter_type_adc(void);

#endif

