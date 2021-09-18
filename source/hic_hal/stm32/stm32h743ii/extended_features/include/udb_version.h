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

#define PIN_UDB_VERSION_PORT         GPIOG
#define PIN_UDB_VERSION              GPIO_PIN_15

void read_udb_version(void);
const char *get_udb_version(void);

#endif
