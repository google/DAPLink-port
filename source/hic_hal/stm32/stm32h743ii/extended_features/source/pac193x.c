#include "pac193x.h"
#include "i2c.h"
#include "udb_errno.h"
#include <stdbool.h>

#define PAC193X_ADDR            (0x17)
#define PAC193X_CFG_REG_SIZE    (1)

static const uint8_t k_pac193x_command_reg_addr[] =
{
    [PAC193X_COMMAND_REFRESH]   = PAC193X_REFRESH_REG,
    [PAC193X_COMMAND_REFRESH_V] = PAC193X_REFRESH_V_REG,
    [PAC193X_COMMAND_REFRESH_G] = PAC193X_REFRESH_G_REG,
};

COMPILER_ASSERT(sizeof(k_pac193x_command_reg_addr) / sizeof(uint8_t) == PAC193X_COMMAND_SIZE);

typedef enum
{
    PAC193X_CFG_TYPE_CTRL,
    PAC193X_CFG_TYPE_CHAN_DIS,
    PAC193X_CFG_TYPE_NEG_PWR,
    PAC193X_CFG_TYPE_SLOW
} pac193x_cfg_type_t;

int pac193x_send_command(pac193x_command_type_t command_type)
{
    int ret;

    if (I2C_DAP_MasterTransfer(PAC193X_ADDR, &k_pac193x_command_reg_addr[command_type], NULL, 0) == true)
    {
        ret = UDB_SUCCESS;
    }
    else
    {
        ret = -UDB_ERROR;
    }

    return ret;
}

int pac193x_init(const pac193x_cfg_t *cfg)
{
    int ret = UDB_SUCCESS;
    bool i2c_status;
    uint8_t addr;

    addr = PAC193X_CTRL_REG;
    i2c_status = I2C_DAP_MasterTransfer(PAC193X_ADDR, &addr, (uint8_t*)&cfg->ctrl_cfg, PAC193X_CFG_REG_SIZE);

    if (i2c_status == true)
    {
        addr = PAC193X_CHANNEL_DIS_REG;
        i2c_status = I2C_DAP_MasterTransfer(PAC193X_ADDR, &addr, (uint8_t*)&cfg->chan_dis_cfg, PAC193X_CFG_REG_SIZE);
    }

    if (i2c_status == true)
    {
        addr = PAC193X_NEG_PWR_REG;
        i2c_status = I2C_DAP_MasterTransfer(PAC193X_ADDR, &addr, (uint8_t*)&cfg->neg_pwr_cfg, PAC193X_CFG_REG_SIZE);
    }

    if (i2c_status == true)
    {
        addr = PAC193X_SLOW_REG;
        i2c_status = I2C_DAP_MasterTransfer(PAC193X_ADDR, &addr, (uint8_t*)&cfg->slow_cfg, PAC193X_CFG_REG_SIZE);
    }


    if (i2c_status == true)
    {
        // Need to refresh to change the configuration
        ret = pac193x_send_command(PAC193X_COMMAND_REFRESH);
    }

    if (i2c_status == false)
    {
        ret = -UDB_ERROR;
    }

    return ret;
}

int pac193x_read_reg(pac193x_reg_t reg_addr, uint32_t reg_size, void* out)
{
    int ret;

    if (I2C_DAP_MasterRead(PAC193X_ADDR, &reg_addr, out, reg_size) == true)
    {
        ret = UDB_SUCCESS;
    }
    else
    {
        ret = -UDB_ERROR;
    }

    return ret;
}
