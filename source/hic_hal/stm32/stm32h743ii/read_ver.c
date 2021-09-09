/*
 * read_ver.h
 * Yang-te Chen
 * yangtechen@google.com
 * September 7, 2021
 * 
 */

#include <string.h>
#include "read_ver.h"
#include "IO_Config.h"

/*
 * Read UDB version from GPIO PG15 with weak pull-up register enabled
 * Prerequisite: initialize GPIO PG15 at gpio_init() in gpio.c
 */

// DAP_PACKET_SIZE = 64
// "_" + git-sha(40) + "_" + modified(8) + '\0'
// 64 - 1 - 40 - 1 - 8  - 1= 13
static char udb_board_version[13];

void read_udb_board_version(void)
{
  GPIO_PinState bitstatus;
  uint8_t prefix_len = 0;
  const char *prefix = "UDB_P";

  prefix_len = strlen(prefix);
  memset(udb_board_version, 0, sizeof(udb_board_version));
  strcat(udb_board_version, prefix);
  bitstatus = HAL_GPIO_ReadPin(PIN_UDB_VERSION_PORT, PIN_UDB_VERSION);

  if (bitstatus == GPIO_PIN_RESET) {
    udb_board_version[prefix_len] = '2';
  } else {
    udb_board_version[prefix_len] = '1';
  }
}

const char *get_udb_board_version(void)
{
  return udb_board_version;
}


