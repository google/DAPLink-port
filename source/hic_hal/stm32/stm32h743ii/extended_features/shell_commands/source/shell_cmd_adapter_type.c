#include <stdio.h>
#include "shell_cmd_adapter_type.h"
#include "adapter_detector.h"

void cmd_adapter_type(int argc, char *argv[])
{
    adapter_type_t adapter_type = adapter_detector_get_adapter_type_adc();
    printf("%s\n", adapter_detector_get_adapter_type_name(adapter_type));
}
