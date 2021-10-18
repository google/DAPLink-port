/*
 *  Description:
 *     Provide an interface to measure the power on UDB
 */
#ifndef _UDB_POWER_MEASUREMENT_H_INCLUDED_
#define _UDB_POWER_MEASUREMENT_H_INCLUDED_

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    UDB_POWER_MEASUREMENT_TARGET_UDB    = 0,
    UDB_POWER_MEASUREMENT_TARGET_DUT    = 1,

    UDB_POWER_MEASUREMENT_TARGET_SIZE   = 2
} udb_power_measurement_target_t;

bool udb_power_measurement_init(void);
bool udb_power_measurement_measure(void);
bool udb_power_measurement_read_voltage_mv(udb_power_measurement_target_t target, uint16_t *out);
bool udb_power_measurement_read_current_microamp(udb_power_measurement_target_t target, uint32_t *out);

#endif // _UDB_POWER_MEASUREMENT_H_INCLUDED_
