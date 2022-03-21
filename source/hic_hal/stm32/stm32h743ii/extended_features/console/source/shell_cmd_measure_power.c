#include <stdio.h>
#include "shell_cmd_measure_power.h"
#include "udb_power_measurement.h"
#include "udb_errno.h"

void cmd_measure_power(int argc, char *argv[])
{
    int ret = udb_power_measurement_measure();
    uint16_t voltage_mV;
    uint32_t current_uA;

    if (ret == UDB_SUCCESS)
    {
        for (uint8_t target_type = 0; target_type < UDB_POWER_MEASUREMENT_TARGET_COUNT; ++target_type)
        {
            ret = udb_power_measurement_read_voltage_mV(target_type, &voltage_mV);
            if (ret != UDB_SUCCESS)
            {
                printf("ERROR: can not read voltage\n");
                break;
            }
            ret = udb_power_measurement_read_current_uA(target_type, &current_uA);

            if (ret != UDB_SUCCESS)
            {
                printf("ERROR: can not read current\n");
                break;
            }
            printf("Target: %s\n", udb_power_measurement_get_target_name(target_type));
            printf("\tvoltage: %u mV\n", voltage_mV);
            printf("\tcurrent: %lu uA\n", current_uA);
        }
    }
    else
    {
        printf("ERROR: can not measure UDB power\n");
    }
}
