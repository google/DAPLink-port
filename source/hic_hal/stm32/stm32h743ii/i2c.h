/* 
 * i2c.h
 *
 * Evan Hassman
 * ehassman@google.com
 *
 * Eric Lee
 * eleenest@google.com
 * August 14, 2020
 *
 */

#ifndef I2C_H_
#define I2C_H_

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "stm32h743xx.h"
#include "Driver_I2C.h"
#include "I2C_STM32H7xx.h"

void I2C_DAP_PinInit(void);

void I2C_DAP_SignalEvent (uint32_t event);

void I2C_DAP_Initialize(void);

bool I2C_DAP_MasterTransfer(uint16_t device_addr, const uint8_t* reg_addr, const uint8_t* data, uint32_t len);

bool I2C_DAP_MasterRead(uint16_t device_addr, const uint8_t* reg_addr, uint8_t* buf, uint32_t num);

#endif /* I2C_H_ */
