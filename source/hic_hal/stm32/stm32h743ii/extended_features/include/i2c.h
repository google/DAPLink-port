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

/*
 * I2C_DAP_SignalEvent
 *
 * event: status of I2C
 *
 * Process retutned I2C event, callback function, to adjust DAP I2C flags
 */
void I2C_DAP_SignalEvent(uint32_t event);

/*
 * I2C_DAP_Initialize
 *
 * Setup up I2C bus, GPIO ports and pins, and enables interrupts
 */
void I2C_DAP_Initialize(void);

/*
 * I2C_DAP_MasterTransfer
 *
 * device_addr: Target (slave) device to be communicated with
 * reg_addr: Target device register to start writing to
 * data: bytes to be written to target device
 * len: length of data, or number of bytes to be sequentially written
 *
 * Write sequential bytes, starting at given register address, on target I2C device
 */
bool I2C_DAP_MasterTransfer(uint16_t device_addr, const uint8_t* reg_addr, const uint8_t* data, uint32_t len);

/*
 * I2C_DAP_MasterRead
 *
 * device_addr: Target (slave) device to be communicated with
 * reg_addr: Target device register to start reading from
 * buf: buffer to store data read from target device
 * len: number of bytes to be read
 *
 * Read sequential bytes, starting at given register address, on target I2C device
 */
bool I2C_DAP_MasterRead(uint16_t device_addr, const uint8_t* reg_addr, uint8_t* buf, uint32_t num);

#endif /* I2C_H_ */
