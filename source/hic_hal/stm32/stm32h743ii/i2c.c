/* 
 * i2c.c
 *
 * Evan Hassman
 * ehassman@google.com
 *
 * Eric Lee
 * eleenest@google.com
 * August 14, 2020
 *
 * Note that the code within I2C1_DAP_PinInit and I2C2_DAP_PinInit come from STM
 * Need to double check licensing and usesage of that code
 */

#include "i2c.h"

#if defined(MX_I2C1)
extern ARM_DRIVER_I2C            Driver_I2C1;
static ARM_DRIVER_I2C *I2Cdrv = &Driver_I2C1;
#endif

#if defined(MX_I2C2)
extern ARM_DRIVER_I2C            Driver_I2C2;
static ARM_DRIVER_I2C *I2Cdrv = &Driver_I2C2;
#endif

// I2C_SignalEvent from CMSIS I2C Sample code
static volatile uint32_t I2C_Event;

volatile bool completionFlag = false;
volatile bool nakFlag        = false;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c1);
    while(1);
}

void I2C2_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c2);
}
/**
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
static void I2C1_DAP_PinInit(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00606092;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
            // Error
        while(1){};
    }
    /** Configure Analogue filter 
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        // Error
        while(1){};
    }
    /** Configure Digital filter 
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        // Error
        while(1){};
    }
}

/**
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
static void I2C2_DAP_PinInit(void)
{
    hi2c2.Instance = I2C2;
    hi2c2.Init.Timing = 0x00606092;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
            // Error
        while(1){};
    }
    /** Configure Analogue filter 
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        // Error
        while(1){};
    }
    /** Configure Digital filter 
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
    {
        // Error
        while(1){};
    }
}

static void I2C_DAP_PinInit(void)
{
    #if defined(MX_I2C1)
    I2C1_DAP_PinInit1();
    #endif

    #if defined(MX_I2C2)
    I2C2_DAP_PinInit();
    #endif
}

/* I2C Signal Event function callback */
void I2C_DAP_SignalEvent (uint32_t event)
{
    /* Save received events */
    I2C_Event |= event;

    if (event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE) {
        /* Less data was transferred than requested */
        completionFlag = false;
    }
 
    if (event & ARM_I2C_EVENT_TRANSFER_DONE) {
        /* Transfer or receive is finished */
        completionFlag = true;
    }

    if (event & ARM_I2C_EVENT_ADDRESS_NACK) {
        /* Slave address was not acknowledged */
        nakFlag = true;
    }
 
    if (event & ARM_I2C_EVENT_ARBITRATION_LOST) {
        /* Master lost bus arbitration */
    }
 
    if (event & ARM_I2C_EVENT_BUS_ERROR) {
        /* Invalid start/stop position detected */
    }
 
    if (event & ARM_I2C_EVENT_BUS_CLEAR) {
        /* Bus clear operation completed */
    }
 
    if (event & ARM_I2C_EVENT_GENERAL_CALL) {
        /* Slave was addressed with a general call address */
    }

    if (event & ARM_I2C_EVENT_SLAVE_RECEIVE) {
        /* Slave addressed as receiver but SlaveReceive operation is not started */
    }

    if (event & ARM_I2C_EVENT_SLAVE_TRANSMIT) {
        /* Slave addressed as transmitter but SlaveTransmit operation is not started */
    }
}


void I2C_DAP_Initialize(void)
{
    /* I2C driver instance and pin initialization*/
    I2C_DAP_PinInit();

    I2Cdrv->Initialize(I2C_DAP_SignalEvent);
    I2Cdrv->PowerControl(ARM_POWER_FULL);
    I2Cdrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST_PLUS);
    I2Cdrv->Control(ARM_I2C_BUS_CLEAR, 0);

    /* Setup Interrupt, CMSIS I2C uses non-blocking */
    NVIC_SetPriority(I2C2_EV_IRQn, 1);
    NVIC_EnableIRQ(I2C2_EV_IRQn);
}

bool I2C_DAP_MasterTransfer(uint16_t device_addr, const uint8_t* reg_addr, const uint8_t* data, uint32_t len)
{
    /* Send slave address and device address without stop command at end */
    I2Cdrv->MasterTransmit(device_addr, reg_addr, 1, true);
    
    /* Wait until transfer completed */
    while (I2Cdrv->GetStatus().busy);
    
    /* Check if all data transferred */
    if ((I2C_Event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE) != 0U)
        return false;
    
    /* Send slave address and write data to register address with stop command at end */
    I2Cdrv->MasterTransmit(device_addr, data, len, false);
    
    /* Wait until transfer completed */
    while (I2Cdrv->GetStatus().busy);
    
    /* Check if all data transferred */
    if ((I2C_Event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE) != 0U)
        return false;
    
    return true;
}

bool I2C_DAP_MasterRead(uint16_t device_addr, const uint8_t* reg_addr, uint8_t* buf, uint32_t len)
{
    /* Send slave address and device address without stop command at end */
    I2Cdrv->MasterTransmit(device_addr, reg_addr, 1, true);
    
    /* Wait until transfer completed */
    while (I2Cdrv->GetStatus().busy);
    
    /* Check if all data transferred */
    if ((I2C_Event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE) != 0U)
        return false;
    
    /* Send slave address and read from register address with stop command at end */
    I2Cdrv->MasterReceive (device_addr, buf, len, false);
 
    /* Wait until transfer completed */
    while (I2Cdrv->GetStatus().busy);
    
    /* Check if all data transferred */
    if ((I2C_Event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE) != 0U)
        return false;
    
    return true;
}
