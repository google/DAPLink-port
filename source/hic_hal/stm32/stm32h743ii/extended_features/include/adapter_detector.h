#ifndef _ADAPTER_DETECTOR_H_INCLUDED_
#define _ADAPTER_DETECTOR_H_INCLUDED_

#include <stdint.h>

typedef enum
{
    ADAPTER_UDB_13_FLEX,
    ADAPTER_JOKER_FLEX,
    ADAPTER_UDB_6_FLEX,
    ADAPTER_UDB_17_FLEX,
    ADAPTER_UDB_12_FLEX,
    ADAPTER_UNKNOWN,
    ADAPTER_NONE
} adapter_type_t;

void adapter_detector_update_adapter_type_adc(void);
adapter_type_t adapter_detector_get_adapter_type_adc(void);

#endif

