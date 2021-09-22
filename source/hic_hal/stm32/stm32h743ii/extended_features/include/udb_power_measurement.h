/*
 *  Description:
 *     Provide an interface to measure the power on UDB
 */
#ifndef _UDB_POWER_MEASUREMENT_H_INCLUDED_
#define _UDB_POWER_MEASUREMENT_H_INCLUDED_

#include <stdint.h>

typedef enum
{
    UDB_POWER_MEASUREMENT_TARGET_UDB    = 0,
    UDB_POWER_MEASUREMENT_TARGET_DUT    = 1,

    UDB_POWER_MEASUREMENT_TARGET_COUNT  = 2
} udb_power_measurement_target_t;

int udb_power_measurement_init(void);
int udb_power_measurement_measure(void);
int udb_power_measurement_read_voltage_mV(udb_power_measurement_target_t target, uint16_t *voltage_mV);
int udb_power_measurement_read_current_uA(udb_power_measurement_target_t target, uint32_t *current_uA);

#endif // _UDB_POWER_MEASUREMENT_H_INCLUDED_
