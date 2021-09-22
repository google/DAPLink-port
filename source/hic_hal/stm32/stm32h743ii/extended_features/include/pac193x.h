#ifndef _PAC193x_H_INCLUDED_
#define _PAC193x_H_INCLUDED_

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    PAC193X_REFRESH_REG =           0x00,
    PAC193X_CTRL_REG =              0x01,
    PAC193X_ACC_COUNT_REG =         0x02,
    PAC193X_VPOWER1ACC_REG =        0x03,
    PAC193X_VPOWER2ACC_REG =        0x04,
    PAC193X_VPOWER3ACC_REG =        0x05,
    PAC193X_VPOWER4ACC_REG =        0x06,
    PAC193X_VBUS1_REG =             0x07,
    PAC193X_VBUS2_REG =             0x08,
    PAC193X_VBUS3_REG =             0x09,
    PAC193X_VBUS4_REG =             0x0a,
    PAC193X_VSENSE1_REG =           0x0b,
    PAC193X_VSENSE2_REG =           0x0c,
    PAC193X_VSENSE3_REG =           0x0d,
    PAC193X_VSENSE4_REG =           0x0e,
    PAC193X_VBUS1_AVG_REG =         0x0f,
    PAC193X_VBUS2_AVG_REG =         0x10,
    PAC193X_VBUS3_AVG_REG =         0x11,
    PAC193X_VBUS4_AVG_REG =         0x12,
    PAC193X_VSENSE1_AVG_REG =       0x13,
    PAC193X_VSENSE2_AVG_REG =       0x14,
    PAC193X_VSENSE3_AVG_REG =       0x15,
    PAC193X_VSENSE4_AVG_REG =       0x16,
    PAC193X_VPOWER1_AVG_REG =       0x17,
    PAC193X_VPOWER2_AVG_REG =       0x18,
    PAC193X_VPOWER3_AVG_REG =       0x19,
    PAC193X_VPOWER4_AVG_REG =       0x1a,
    PAC193X_CHANNEL_DIS_REG =       0x1c,
    PAC193X_NEG_PWR_REG =           0x1d,
    PAC193X_REFRESH_G_REG =         0x1e,
    PAC193X_REFRESH_V_REG =         0x1f,
    PAC193X_SLOW_REG =              0x20,
    PAC193X_CTRL_ACT_REG =          0x21,
    PAC193X_CHANNEL_DIS_ACT_REG =   0x22,
    PAC193X_NEG_PWR_ACT_REG =       0x23,
    PAC193X_CTRL_LAT_REG =          0x24,
    PAC193X_CHANNEL_DIS_LAT_REG =   0x25,
    PAC193X_NEG_PWR_LAT_REG =       0x26,
    PAC193X_PRODUCT_ID_REG =        0xfd,
    PAC193X_MANUFACTURER_ID_REG =   0xfe,
    PAC193X_REVISION_ID_REG =       0xff,
} pac193x_reg_t;

#define PAC193X_ADDR_SEL (0x17)

#define PAC193X_ACC_POWER_BYTES  (6)
#define PAC193X_POWER_BYTES      (4)
#define PAC193X_VBUS_BYTES       (2)
#define PAC193X_VSENSE_BYTES     (2)

typedef union
{
    struct
    {
        uint8_t ovf:1;
        uint8_t ovf_alert_en:1;
        uint8_t conv_alert_en:1;
        uint8_t alert_pin_en:1;
        uint8_t sing_mode:1;
        uint8_t sleep_mode:1;
        uint8_t sample_rate:2;
    };
    uint8_t val;
} pac193x_ctrl_reg_t;

#define PAC193X_CTRL_REG_OVF_POS              (0)
#define PAC193X_CTRL_REG_OVF_MASK             (0x01)
#define PAC193X_CTRL_REG_OVF_ALERT_EN_POS     (1)
#define PAC193X_CTRL_REG_OVF_ALERT_EN_MASK    (0x02)
#define PAC193X_CTRL_REG_CONV_ALERT_EN_POS    (2)
#define PAC193X_CTRL_REG_CONV_ALERT_EN_MASK   (0x04)
#define PAC193X_CTRL_REG_ALERT_PIN_EN_POS     (3)
#define PAC193X_CTRL_REG_ALERT_PIN_EN_MASK    (0x08)
#define PAC193X_CTRL_REG_SING_MODE_EN_POS     (4)
#define PAC193X_CTRL_REG_SING_MODE_EN_MASK    (0x10)
#define PAC193X_CTRL_REG_SLEEP_MODE_EN_POS    (5)
#define PAC193X_CTRL_REG_SLEEP_MODE_EN_MASK   (0x20)
#define PAC193X_CTRL_REG_SAMPLE_RATE_EN_POS   (6)
#define PAC193X_CTRL_REG_SAMPLE_RATE_EN_MASK  (0xc0)

#define PAC193X_SAMPLE_RATE_1024 (0x00)
#define PAC193X_SAMPLE_RATE_256 (0x01)
#define PAC193X_SAMPLE_RATE_64 (0x02)
#define PAC193X_SAMPLE_RATE_8 (0x03)

typedef union
{
    struct
    {
        uint8_t nop:1;
        uint8_t noskip_en:1;
        uint8_t byte_count_en:1;
        uint8_t timeout_en:1;
        uint8_t ch4_dis:1;
        uint8_t ch3_dis:1;
        uint8_t ch2_dis:1;
        uint8_t ch1_dis:1;
    };
    uint8_t val;
} pac193x_chan_dis_reg_t;

#define PAC193X_CHAN_DIS_REG_NOSKIP_EN_POS        (1)
#define PAC193X_CHAN_DIS_REG_NOSKIP_EN_MASK       (0x02)
#define PAC193X_CHAN_DIS_REG_BYTE_COUNT_EN_POS    (2)
#define PAC193X_CHAN_DIS_REG_BYTE_COUNT_EN_MASK   (0x04)
#define PAC193X_CHAN_DIS_REG_TIMEOUT_EN_POS       (3)
#define PAC193X_CHAN_DIS_REG_TIMEOUT_EN_MASK      (0x08)
#define PAC193X_CHAN_DIS_REG_CH4_DIS_POS          (4)
#define PAC193X_CHAN_DIS_REG_CH4_DIS_MASK         (0x10)
#define PAC193X_CHAN_DIS_REG_CH3_DIS_POS          (5)
#define PAC193X_CHAN_DIS_REG_CH3_DIS_MASK         (0x20)
#define PAC193X_CHAN_DIS_REG_CH2_DIS_POS          (6)
#define PAC193X_CHAN_DIS_REG_CH2_DIS_MASK         (0x40)
#define PAC193X_CHAN_DIS_REG_CH1_DIS_POS          (7)
#define PAC193X_CHAN_DIS_REG_CH1_DIS_MASK         (0x80)

typedef union
{
    struct
    {
        uint8_t vbus4_bi_en:1;
        uint8_t vbus3_bi_en:1;
        uint8_t vbus2_bi_en:1;
        uint8_t vbus1_bi_en:1;
        uint8_t vsense4_bi_en:1;
        uint8_t vsense3_bi_en:1;
        uint8_t vsense2_bi_en:1;
        uint8_t vsense1_bi_en:1;
    };
    uint8_t val;
} pac193x_neg_pwr_reg_t;

#define PAC193X_NEG_PWR_REG_VBUS4_BI_EN_POS     (0)
#define PAC193X_NEG_PWR_REG_VBUS4_BI_EN_MASK    (0x01)
#define PAC193X_NEG_PWR_REG_VBUS3_BI_EN_POS     (1)
#define PAC193X_NEG_PWR_REG_VBUS3_BI_EN_MASK    (0x02)
#define PAC193X_NEG_PWR_REG_VBUS2_BI_EN_POS     (2)
#define PAC193X_NEG_PWR_REG_VBUS2_BI_EN_MASK    (0x04)
#define PAC193X_NEG_PWR_REG_VBUS1_BI_EN_POS     (3)
#define PAC193X_NEG_PWR_REG_VBUS1_BI_EN_MASK    (0x08)
#define PAC193X_NEG_PWR_REG_VSENSE4_BI_EN_POS   (4)
#define PAC193X_NEG_PWR_REG_VSENSE4_BI_EN_MASK  (0x10)
#define PAC193X_NEG_PWR_REG_VSENSE3_BI_EN_POS   (5)
#define PAC193X_NEG_PWR_REG_VSENSE3_BI_EN_MASK  (0x20)
#define PAC193X_NEG_PWR_REG_VSENSE2_BI_EN_POS   (6)
#define PAC193X_NEG_PWR_REG_VSENSE2_BI_EN_MASK  (0x40)
#define PAC193X_NEG_PWR_REG_VSENSE1_BI_EN_POS   (7)
#define PAC193X_NEG_PWR_REG_VSENSE1_BI_EN_MASK  (0x80)

typedef union
{
    struct
    {
        uint8_t por_status:1;
        uint8_t fall_refresh_v_en:1;
        uint8_t fall_refresh_en:1;
        uint8_t rise_refresh_v_en:1;
        uint8_t rise_refresh_en:1;
        uint8_t is_hi_to_lo:1;
        uint8_t is_lo_to_hi:1;
        uint8_t is_pull_high:1;
    };
    uint8_t val;
} pac193x_slow_reg_t;

#define PAC193X_SLOW_REG_POR_STATUS_POS           (0)
#define PAC193X_SLOW_REG_POR_STATUS_MASK          (0x01)
#define PAC193X_SLOW_REG_FALL_REFRESH_V_EN_POS    (1)
#define PAC193X_SLOW_REG_FALL_REFRESH_V_EN_MASK   (0x02)
#define PAC193X_SLOW_REG_FALL_REFRESH_EN_POS      (2)
#define PAC193X_SLOW_REG_FALL_REFRESH_EN_MASK     (0x04)
#define PAC193X_SLOW_REG_RISE_REFRESH_V_EN_POS    (3)
#define PAC193X_SLOW_REG_RISE_REFRESH_V_EN_MASK   (0x08)
#define PAC193X_SLOW_REG_RISE_REFRESH_EN_POS      (4)
#define PAC193X_SLOW_REG_RISE_REFRESH_EN_MASK     (0x10)
#define PAC193X_SLOW_REG_IS_HI_TO_LO_POS          (5)
#define PAC193X_SLOW_REG_IS_HI_TO_LO_MASK         (0x20)
#define PAC193X_SLOW_REG_IS_LO_TO_HI_POS          (6)
#define PAC193X_SLOW_REG_IS_LO_TO_HI_MASK         (0x40)
#define PAC193X_SLOW_REG_IS_PULL_HIGH_POS         (7)
#define PAC193X_SLOW_REG_IS_PULL_HIGH_MASK        (0x80)

typedef struct
{
    pac193x_ctrl_reg_t      ctrl_cfg;
    pac193x_chan_dis_reg_t  chan_dis_cfg;
    pac193x_neg_pwr_reg_t   neg_pwr_cfg;
    pac193x_slow_reg_t      slow_cfg;
} pac193x_cfg_t;

void pac193x_refresh(void);
void pac193x_refresh_g(void);
void pac193x_refresh_v(void);

void pac193x_init(void);

bool pac193x_read_vpower_acc1(uint8_t* data);

uint8_t pac193x_read_product_id(void);
uint8_t pac193x_read_manufacturer_id(void);
uint8_t pac193x_read_revision_id(void);
uint32_t get_duration(void);

#endif // _PAC193x_H_INCLUDED_
