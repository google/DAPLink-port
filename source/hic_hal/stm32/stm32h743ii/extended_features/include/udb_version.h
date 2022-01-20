/*
 * udb_version.h
 * Yang-te Chen
 * yangtechen@google.com
 * September 7, 2021
 *
 */

#ifndef UDB_VERSION_H_
#define UDB_VERSION_H_

#include "stm32h7xx.h"
#include "DAP_config.h"

#define UDB_VERSION_MAX_LENGTH      DAP_PACKET_SIZE

void udb_read_bootloader_version(void);
void udb_read_hw_version(void);
int udb_get_interface_version(uint8_t *buffer, unsigned size);
int udb_get_bootloader_version(uint8_t *buffer, unsigned size);

#endif
