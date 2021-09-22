#include "pac193x.h"
#include "i2c.h"

static void error_handler(void)
{
    for (;;)
    {
    }
}

static uint32_t s_duration;

uint32_t get_duration(void)
{
    return s_duration;
}

void pac193x_refresh(void)
{
    const uint8_t reg_addr = PAC193X_REFRESH_REG;
    bool ret = true;
    ret = I2C_DAP_MasterTransfer(PAC193X_ADDR_SEL, &reg_addr, NULL, 0);
    if (ret == false)
    {
        error_handler();
    }
}

void pac193x_refresh_g(void)
{
    const uint8_t reg_addr = PAC193X_REFRESH_G_REG;
    bool ret = true;
    ret = I2C_DAP_MasterTransfer(PAC193X_ADDR_SEL, &reg_addr, NULL, 0);
    if (ret == false)
    {
        error_handler();
    }
}

void pac193x_refresh_v(void)
{
    const uint8_t reg_addr = PAC193X_REFRESH_V_REG;
    bool ret = true;
    ret = I2C_DAP_MasterTransfer(PAC193X_ADDR_SEL, &reg_addr, NULL, 0);
    if (ret == false)
    {
        error_handler();
    }
}

void pac193x_init(void)
{
    uint8_t reg_addr;
    bool ret = true;

    reg_addr = PAC193X_CHANNEL_DIS_REG;
    pac193x_cfg_t cfg;
    cfg.chan_dis_cfg.val = 0;
    cfg.chan_dis_cfg.ch2_dis = 1;
    cfg.chan_dis_cfg.ch3_dis = 1;
    cfg.chan_dis_cfg.ch4_dis = 1;
    ret = I2C_DAP_MasterTransfer(PAC193X_ADDR_SEL, &reg_addr, (uint8_t*)&cfg.chan_dis_cfg, 1);
    if (ret == false)
    {
        error_handler();
    }

    reg_addr = PAC193X_CTRL_REG;
    cfg.ctrl_cfg.val = 0;
    cfg.ctrl_cfg.sample_rate = PAC193X_SAMPLE_RATE_1024;
    ret = I2C_DAP_MasterTransfer(PAC193X_ADDR_SEL, &reg_addr, (uint8_t*)&cfg.ctrl_cfg, 1);
    if (ret == false)
    {
        error_handler();
    }

    pac193x_refresh();
}

bool pac193x_read_vpower_acc1(uint8_t* out)
{
    uint8_t buf[PAC193X_ACC_POWER_BYTES];
    const uint8_t reg_addr = PAC193X_VPOWER1ACC_REG;
    bool ret = true;
    ret = I2C_DAP_MasterRead(PAC193X_ADDR_SEL, &reg_addr, buf, PAC193X_ACC_POWER_BYTES);
    if (!ret)
    {
      return false;
    }

    for (int i = 0; i < PAC193X_ACC_POWER_BYTES; ++i)
    {
      *out++ = buf[i];
    }
    return true;
}

uint8_t pac193x_read_product_id(void)
{
    uint8_t product_id;
    const uint8_t reg_addr = PAC193X_PRODUCT_ID_REG;
    bool ret = true;
    ret = I2C_DAP_MasterRead(PAC193X_ADDR_SEL, &reg_addr, &product_id, 1);
    if (ret == false)
    {
      return 0;
    }

    return product_id;
}

uint8_t pac193x_read_manufacturer_id(void)
{
    uint8_t manufacturer_id;
    const uint8_t reg_addr = PAC193X_MANUFACTURER_ID_REG;
    bool ret = true;
    ret = I2C_DAP_MasterRead(PAC193X_ADDR_SEL, &reg_addr, &manufacturer_id, 1);
    if (ret == false)
    {
      return 0;
    }

    return manufacturer_id;
}

uint8_t pac193x_read_revision_id(void)
{
    uint8_t revision_id;
    const uint8_t reg_addr = PAC193X_REVISION_ID_REG;
    bool ret = true;
    ret = I2C_DAP_MasterRead(PAC193X_ADDR_SEL, &reg_addr, &revision_id, 1);
    if (ret == false)
    {
      return 0;
    }

    return revision_id;
}
