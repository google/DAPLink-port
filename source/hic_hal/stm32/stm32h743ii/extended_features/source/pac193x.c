#include "pac193x.h"
#include "i2c.h"

#define PAC193X_ADDR            (0x17)
#define PAC193X_CFG_REG_SIZE    (1)

static const uint8_t k_pac193x_command_reg_addr[] =
{
    [PAC193X_COMMAND_REFRESH]   = PAC193X_REFRESH_REG,
    [PAC193X_COMMAND_REFRESH_V] = PAC193X_REFRESH_V_REG,
    [PAC193X_COMMAND_REFRESH_G] = PAC193X_REFRESH_G_REG,
};

typedef enum
{
    PAC193X_CFG_TYPE_CTRL,
    PAC193X_CFG_TYPE_CHAN_DIS,
    PAC193X_CFG_TYPE_NEG_PWR,
    PAC193X_CFG_TYPE_SLOW
} pac193x_cfg_type_t;

static const uint8_t k_pac193x_cfg_reg_addr[] =
{
    [PAC193X_CFG_TYPE_CTRL]     = PAC193X_CTRL_REG,
    [PAC193X_CFG_TYPE_CHAN_DIS] = PAC193X_CHANNEL_DIS_REG,
    [PAC193X_CFG_TYPE_NEG_PWR]  = PAC193X_NEG_PWR_REG,
    [PAC193X_CFG_TYPE_SLOW]     = PAC193X_SLOW_REG
};

bool pac193x_send_command(pac193x_command_type_t command_type)
{
    return I2C_DAP_MasterTransfer(PAC193X_ADDR, &k_pac193x_command_reg_addr[command_type], NULL, 0); 
}

bool pac193x_init(pac193x_cfg_t *cfg)
{
    bool ret;
    ret = I2C_DAP_MasterTransfer(PAC193X_ADDR, &k_pac193x_cfg_reg_addr[PAC193X_CFG_TYPE_CTRL], (uint8_t*)&cfg->ctrl_cfg, PAC193X_CFG_REG_SIZE);

    if (ret == false)
    {
        return false;
    }

    ret = I2C_DAP_MasterTransfer(PAC193X_ADDR, &k_pac193x_cfg_reg_addr[PAC193X_CFG_TYPE_CHAN_DIS], (uint8_t*)&cfg->chan_dis_cfg, PAC193X_CFG_REG_SIZE);

    if (ret == false)
    {
        return false;
    }

    ret = I2C_DAP_MasterTransfer(PAC193X_ADDR, &k_pac193x_cfg_reg_addr[PAC193X_CFG_TYPE_NEG_PWR], (uint8_t*)&cfg->neg_pwr_cfg, PAC193X_CFG_REG_SIZE);

    if (ret == false)
    {
        return false;
    }

    ret = I2C_DAP_MasterTransfer(PAC193X_ADDR, &k_pac193x_cfg_reg_addr[PAC193X_CFG_TYPE_SLOW], (uint8_t*)&cfg->slow_cfg, PAC193X_CFG_REG_SIZE);

    if (ret == false)
    {
        return false;
    }

    // Need to refresh to change the configuration
    return pac193x_send_command(PAC193X_COMMAND_REFRESH);
}

bool pac193x_read_reg(uint8_t reg_addr, uint8_t reg_size, uint8_t* out)
{
    return I2C_DAP_MasterRead(PAC193X_ADDR, &reg_addr, out, reg_size);
}
