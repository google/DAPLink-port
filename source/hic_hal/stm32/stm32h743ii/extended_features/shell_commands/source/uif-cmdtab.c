#include "nluif_udb-daplink.h"
#include "shell_cmd_gpio.h"
#include "shell_cmd_pwm.h"
#include "shell_cmd_version.h"

const UIF_CMD UIF_CMDTAB[] = {
    UIF_CMD_HELP,
    UIF_CMD_GPIO,
    UIF_CMD_PWM,
    UIF_CMD_VERSION,
};

const int UIF_NUM_CMD = sizeof (UIF_CMDTAB) / sizeof (UIF_CMDTAB[0]);

const UIF_SETCMD UIF_SETCMDTAB[0] = {
};

const int UIF_NUM_SETCMD = 0;
