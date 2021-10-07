#include "udb_power_measurement.h"
#include "pac193x.h"
#include <stdbool.h>
#include <stdint.h>

static pac193x_cfg_t s_udb_power_measurement_cfg;

static const float k_udb_power_measurement_resister[] =
{
    [UDB_POWER_MEASUREMENT_TARGET_UDB] = 0.015f,
    [UDB_POWER_MEASUREMENT_TARGET_DUT] = 0.015f
};

bool udb_power_measurement_init(void)
{
    s_udb_power_measurement_cfg.ctrl_cfg.val      = PAC193X_CTRL_POR_VALUE;
    s_udb_power_measurement_cfg.chan_dis_cfg.val  = PAC1934_CHANNEL_DIS_POR_VALUE;
    s_udb_power_measurement_cfg.neg_pwr_cfg.val   = PAC193X_NEG_PWR_POR_VALUE;
    s_udb_power_measurement_cfg.slow_cfg.val      = PAC193X_SLOW_POR_VALUE;

    s_udb_power_measurement_cfg.chan_dis_cfg.ch3_dis = 1;
    s_udb_power_measurement_cfg.chan_dis_cfg.ch4_dis = 1;

    return pac193x_init(&s_udb_power_measurement_cfg);
}

bool udb_power_measurement_measure(void)
{
    return pac193x_send_command(PAC193X_COMMAND_REFRESH);
}

static const uint8_t k_udb_power_measurement_voltage_reg[] =
{
    [UDB_POWER_MEASUREMENT_TARGET_UDB] = PAC193X_VBUS1_REG,
    [UDB_POWER_MEASUREMENT_TARGET_DUT] = PAC193X_VBUS2_REG
};

static const uint8_t k_udb_power_measurement_current_reg[] =
{
    [UDB_POWER_MEASUREMENT_TARGET_UDB] = PAC193X_VSENSE1_REG,
    [UDB_POWER_MEASUREMENT_TARGET_DUT] = PAC193X_VSENSE2_REG
};

#define UDB_POWER_MEASUREMENT_DENOMINATOR       (1<<16)

bool udb_power_measurement_read_voltage_v(udb_power_measurement_target_t target, float *out)
{
    uint8_t vbus_reg[PAC193X_VBUSN_REG_SIZE];
    
    bool ret = pac193x_read_reg(k_udb_power_measurement_voltage_reg[target], PAC193X_VBUSN_REG_SIZE, vbus_reg);

    if (ret == false)
    {
        return false;
    }

    uint32_t vbus = 0;
    for (int8_t i = 0; i < PAC193X_VBUSN_REG_SIZE; ++i)
    {
        vbus = vbus * (1<<8) + vbus_reg[i];
    }

    *out = PAC193X_FULL_SCALE_VOLTAGE_V * ((float)vbus) / UDB_POWER_MEASUREMENT_DENOMINATOR;

    return true;
}

bool udb_power_measurement_read_current_mamp(udb_power_measurement_target_t target, float *out)
{
    uint8_t vsense_reg[PAC193X_VSENSEN_REG_SIZE];

    bool ret = pac193x_read_reg(k_udb_power_measurement_current_reg[target], PAC193X_VSENSEN_REG_SIZE, vsense_reg);

    if (ret == false)
    {
        return false;
    }

    uint32_t vsense = 0;
    for (int8_t i = 0; i < PAC193X_VSENSEN_REG_SIZE; ++i)
    {
        vsense = vsense * (1<<8) + vsense_reg[i];
    }

    float full_scale_current_mamp = PAC193X_FULL_SCALE_RANGE_MV / k_udb_power_measurement_resister[target];
    *out = full_scale_current_mamp * ((float)vsense) / UDB_POWER_MEASUREMENT_DENOMINATOR;

    return true;
}
