/*
 * udb_version.c
 * Yang-te Chen
 * yangtechen@google.com
 * September 7, 2021
 *
 */

#include <string.h>
#include <stdio.h>
#include "udb_version.h"
#include "version_git.h"
#include "DAP_config.h"
#include "IO_Config.h"

#if defined(UDB_VERSION_BASE) && defined(UDB_BUILD_NUMBER)

#define VAL(str) #str
#define TO_STRING(str) VAL(str)
#define UDB_BUILD_VERSION        TO_STRING(UDB_VERSION_BASE) TO_STRING(UDB_BUILD_NUMBER)

#else

#if GIT_LOCAL_MODS == 1
#define GIT_LOCAL_MODS_STR      "_modified"
#else
#define GIT_LOCAL_MODS_STR      ""
#endif // GIT_LOCAL_MODS
#define UDB_BUILD_VERSION       "local_build" GIT_LOCAL_MODS_STR

#endif // defined(UDB_VERSION_BASE) && defined(UDB_BUILD_NUMBER)

#define PIN_UDB_HW_VERSION_PORT GPIOG
#define PIN_UDB_HW_VERSION      GPIO_PIN_15

typedef enum
{
    HW_VERSION_UNKNOWN,
    HW_VERSION_P1,
    HW_VERSION_P2,
} hw_version_t;

static const char s_build_version_str[] = "udb_" UDB_BUILD_VERSION "_" GIT_DESCRIPTION "_hw:";
static hw_version_t s_hw_version = HW_VERSION_UNKNOWN;

static bool is_hw_version_p1(void)
{
    uint8_t len;
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_PinState bitstatus;
    GPIO_InitStructure.Pin = PIN_UDB_HW_VERSION;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PIN_UDB_HW_VERSION_PORT, &GPIO_InitStructure);

    HAL_Delay(osKernelGetTickFreq() / 50); // 10ms for GPIO stabilization

    bitstatus = HAL_GPIO_ReadPin(PIN_UDB_HW_VERSION_PORT, PIN_UDB_HW_VERSION);
    HAL_GPIO_DeInit(PIN_UDB_HW_VERSION_PORT, PIN_UDB_HW_VERSION);

    return bitstatus != GPIO_PIN_RESET;
}

void udb_read_hw_version(void)
{
    if (is_hw_version_p1())
    {
        s_hw_version = HW_VERSION_P1;
    }
    else
    {
        // Todo: read ADC to further differentiate P2, EVT, DVT...
        s_hw_version = HW_VERSION_P2;
    }
}

int udb_get_version(uint8_t *buffer, unsigned size)
{
    return snprintf(buffer, size, "%s%d", s_build_version_str, s_hw_version);
}
