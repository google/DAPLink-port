#include "nluif_udb-daplink.h"
#include "shell_cmd_gpio.h"
#include "shell_cmd_pwm.h"
#include "shell_cmd_version.h"
#include "shell_cmd_fault.h"
#include "shell_cmd_reset.h"
#include "shell_cmd_adapter_type.h"
#include "shell_cmd_measure_power.h"
#include "shell_cmd_reset_into_swu_mode.h"
#include "shell_cmd_i2c.h"
#include "shell_cmd_dut_reset.h"
#include "shell_cmd_fault_info.h"

const UIF_CMD UIF_CMDTAB[] = {
    UIF_CMD_HELP,
    UIF_CMD_GPIO,
    UIF_CMD_PWM,
    UIF_CMD_VERSION,
    UIF_CMD_FAULT,
    UIF_CMD_RESET,
    UIF_CMD_ADAPTER_TYPE,
    UIF_CMD_MEASURE_POWER,
    UIF_CMD_RESET_INTO_SWU_MODE,
    UIF_CMD_I2C,
    UIF_CMD_DUT_RESET,
    UIF_CMD_FAULT_INFO,
};

const int UIF_NUM_CMD = sizeof (UIF_CMDTAB) / sizeof (UIF_CMDTAB[0]);

const UIF_SETCMD UIF_SETCMDTAB[0] = {
};

const int UIF_NUM_SETCMD = 0;
