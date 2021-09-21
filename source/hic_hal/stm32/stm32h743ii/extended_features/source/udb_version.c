/*
 * udb_version.c
 * Yang-te Chen
 * yangtechen@google.com
 * September 7, 2021
 * 
 */

#include <string.h>
#include "udb_version.h"
#include "version_git.h"
#include "DAP_config.h"
#include "IO_Config.h"


#if GIT_LOCAL_MODS == 1
#define GIT_LOCAL_MODS_STR ""
#else
#define GIT_LOCAL_MODS_STR ""
#endif //GIT_LOCAL_MODS

#define STR_IMPL_(x) #x      //stringify argument
#define STR(x) STR_IMPL_(x)  //indirection to expand argument macros

/*
 * Read UDB version from GPIO PG15 with weak pull-up register enabled
 * Prerequisite: initialize GPIO PG15 at gpio_init() in gpio.c
 */

static char s_udb_version[DAP_PACKET_SIZE-1];
static char s_build_version_str[] = "udb_" STR(UDB_VERSION) "_" GIT_COMMIT_SHA GIT_LOCAL_MODS_STR "_p";

static void busy_wait(uint32_t cycles)
{
    volatile uint32_t i;
    i = cycles;

    while (i > 0) {
        i--;
    }
}

void read_udb_version(void)
{
  // Initialize PG5, this is used to decide UDB is P1 or P2
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Pin = PIN_UDB_VERSION;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PIN_UDB_VERSION_PORT, &GPIO_InitStructure);

  busy_wait(1000000);

  GPIO_PinState bitstatus;
  uint8_t len;

  len = strlen(s_build_version_str);
  memset(s_udb_version, 0, sizeof(s_udb_version));
  strcat(s_udb_version, s_build_version_str);
  bitstatus = HAL_GPIO_ReadPin(PIN_UDB_VERSION_PORT, PIN_UDB_VERSION);

  if (bitstatus == GPIO_PIN_RESET)
  {
    s_udb_version[len] = '2';
  }
  else
  {
    s_udb_version[len] = '1';
  }
}

const char *get_udb_version(void)
{
  return s_udb_version;
}


