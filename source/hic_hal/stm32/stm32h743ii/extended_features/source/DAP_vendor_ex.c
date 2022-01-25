#include "DAP.h"
#include "settings.h"
#include <string.h>
#include "i2c.h"
#include "cortex_m.h"
#include "stm32h7xx.h"
#include "DAP_vendor_ex.h"
#include "DAP_config.h"
#include "udb_version.h"
#include "adapter_detector.h"
#include "udb_reset.h"
#include "util.h"
#include "udb_power_measurement.h"
#include "udb_errno.h"

#define UDB_RESET_TIMER_MS   (500)


/** Read back the output value and direction of a UDB DUT gpio pin.
 * External buffers are used, so there is one MCU pin to set the buffer
 * direction, and a separate MCU pin to set the gpio value.
 * The return value is formatted so that up to 8 gpio's can be left shifted and OR'ed
 * together.
\param GPIO_io_port   PORT for the "gpio value" pin
\param GPIO_io_pin    PIN for the "gpio value" pin
\param GPIO_dir_port  PORT for the "direction" pin
\param GPIO_dir_pin   PIN for the "direction" pin

\return          a 16 bit value formatted as 0b0000000<dir>0000000<value>
*/
static uint16_t read_gpio(GPIO_TypeDef* GPIO_io_port, uint16_t GPIO_io_pin,
                  GPIO_TypeDef* GPIO_dir_port, uint16_t GPIO_dir_pin)
{
    uint16_t pin_value = 0;
    // ToDo: port to "HAL_GPIO_xxx" functions, like
    // pin_value |= HAL_GPIO_ReadPin(GPIO_io_port, GPIO_io_pin) ? 0x1 : 0x0;
    pin_value |= (GPIO_io_port->IDR & GPIO_io_pin) ? 0x1 : 0x0;
    pin_value |= (GPIO_dir_port->IDR & GPIO_dir_pin) ? 0x0100 : 0x0000;
    return pin_value;
}


/** Set one of the DUT GPIO pins.
 * External buffers are used, so there is one MCU pin to set the buffer
 * direction, and a separate MCU pin to set the gpio value.
 * To set the output value we actually set a weak pullup/pulldown on the MCU
 * gpio, then there is no issue if the external buffer drives a value into the
 * MCU pin.
\param GPIO_io_PORT   PORT for the "gpio value" pin
\param GPIO_io_PIN_Bit   PIN_Bit for the "gpio value" pin.
                          Note: this is a decimal (0-15), NOT a bit-mask.
\param GPIO_dir_PORT  PORT for the "direction" pin
\param GPIO_dir_PIN   PIN for the "direction" pin.  This is a bit-mask.
\param pin_direction   bitmask of all the gpio directions
\param pin_values   bitmask of all the gpio values
\param which_bit    Which bit to extract from the pin_direction/pin_values.
\return          NA
*/
static void write_gpio(GPIO_TypeDef* GPIO_io_PORT, uint16_t GPIO_io_PIN_Bit,
                  GPIO_TypeDef* GPIO_dir_PORT, uint16_t GPIO_dir_PIN,
                  uint8_t pin_direction,
                  uint8_t pin_values,
                  uint8_t which_bit)
{
  uint32_t temp = 0;

  //Only update pullup/down if we are driving the DUT.
  if(pin_direction & which_bit)
  {
    // ToDo: port to "HAL_xxx" functions
    // The pullups have 2bits per pin.  01: Pull-up, 10: Pull-down
    temp = GPIO_io_PORT->PUPDR;
    temp &= ~(3UL << (GPIO_io_PIN_Bit * 2U)); //Clear 2 bits in register

    if(pin_values & which_bit)  // If pull-high...
      temp |= (1U << (GPIO_io_PIN_Bit * 2U));  //01: Pull-up
    else
      temp |= (2U << (GPIO_io_PIN_Bit * 2U));  //10: Pull-down

    GPIO_io_PORT->PUPDR = temp;
    GPIO_dir_PORT->BSRR = GPIO_dir_PIN; //Set input or output.
  }
  else
  {
    // If is an input, just set the direction pin.
    GPIO_dir_PORT->BSRR = (uint32_t)GPIO_dir_PIN << 16;
  }
}



static uint32_t DAP_ProcessVendorCommandEx40_MeasurePower(const uint8_t *request, uint8_t *response)
{
    // 2 bytes for voltage and 4 bytes for current
    util_assert(6 * UDB_POWER_MEASUREMENT_TARGET_COUNT <= DAP_PACKET_SIZE);

    uint32_t num = 0;
    int ret = udb_power_measurement_measure();

    if (ret != UDB_SUCCESS)
    {
        goto power_measurement_error;
    }

    uint8_t data_buf[DAP_PACKET_SIZE];
    uint8_t data_buf_idx = 0;
    for (uint8_t target_type = 0; target_type < UDB_POWER_MEASUREMENT_TARGET_COUNT; ++target_type)
    {
        uint16_t voltage_mV;
        uint32_t current_uA;

        ret = udb_power_measurement_read_voltage_mV(target_type, &voltage_mV);
        if (ret != UDB_SUCCESS)
        {
            goto power_measurement_error;
        }
        data_buf[data_buf_idx++] = (voltage_mV & 0xFF);
        data_buf[data_buf_idx++] = ((voltage_mV >> 8) & 0xFF);

        ret = udb_power_measurement_read_current_uA(target_type, &current_uA);
        if (ret != UDB_SUCCESS)
        {
            goto power_measurement_error;
        }

        data_buf[data_buf_idx++] = (current_uA & 0xFF);
        data_buf[data_buf_idx++] = ((current_uA >> 8) & 0xFF);
        data_buf[data_buf_idx++] = ((current_uA >> 16) & 0xFF);
        data_buf[data_buf_idx++] = ((current_uA >> 24) & 0xFF);
    }

    *response++ = DAP_OK;

    memcpy(response, data_buf, data_buf_idx);
    response += data_buf_idx;

    num += 1 + data_buf_idx;

    return num;

power_measurement_error:
    *response = DAP_ERROR;
    num += 1;

    return num;
}

/** Process DAP Vendor Command from the Extended Command range and prepare Response Data
\param request   pointer to request data
\param response  pointer to response data
\return          number of bytes in response (lower 16 bits)
                 number of bytes in request (upper 16 bits)
*/
uint32_t DAP_ProcessVendorCommandEx(const uint8_t *request, uint8_t *response) {
  uint32_t num = (1U << 16) | 1U;  // count the Command ID byte

  *response++ = *request;        // copy Command ID

  switch (*request++) {          // first byte in request is Command ID
    case ID_DAP_VendorEx32_I2C_READ: {
        //  Read I2C data
        //
        //  This reads from the "I2C2" bus on the UDB board.  This is routed to an EEPROM
        //    (24AA02UIDT) at address 0x50, a voltage measurement chip (PAC1934T-I/J6CX)
        //    at address 0x17, and pins 13 and 15 of
        //    connector J1305 (with pullups to 1.8V).  It uses the MCU's internal I2C2 hw block.

        //  The inputs to this command are:
        //    I2C_TargetAddress, 1 byte (7 bit I2C chip address)
        //    I2C_starting_register_address, 1 byte
        //    Read_length, 1 byte (1 to 61 bytes is valid)
        //
        //  This returns a byte array which contains:
        //    status byte (0x00 = Success, 0xFF = Failure)
        //    I2C "event" status (2 bytes).  Search for "ARM_I2C_EVENT_TRANSFER_DONE" for bit definitions
        //      0x01 is "done", 0x10 is "cleared", other bits indicate errors.
        //    length byte
        //    1 to 61 data bytes
        //
        uint8_t target_addr = *request++;
        const uint8_t* internal_addr = request++;
        uint8_t len = *request;
        uint8_t data_buf[64] = { 0 };
        uint32_t returnVal = 0;


        returnVal = I2C_DAP_MasterRead(target_addr, internal_addr, data_buf, len);
        if (returnVal== 0x01) {
            *response++ = DAP_OK;
        } else {
            // transfer incomplete or error
            *response++ = DAP_ERROR;
        }
        *response++ = (returnVal & 0xFF); //9 bits in an event.  Get first byte
        *response++ = ((returnVal >> 8) & 0xFF); //Get 2nd byte
        *response++ = len;  //length byte

        for (int i = 0; i < len; i++) {
            *response++ = data_buf[i];
        }

        //3 bytes read for the command, returns status byte, returnVal, length byte, and len data bytes
        num += (3 << 16) | (4 + len);
        break;
    }
    case ID_DAP_VendorEx33_I2C_WRITE: {
        //  Write I2C data
        //
        //  This writes data to the "I2C2" bus on the UDB board.  This is routed to an EEPROM
        //    (24AA02UIDT) at address 0x50, a voltage measurement chip (PAC1934T-I/J6CX)
        //    at address 0x17, and pins 13 and 15 of
        //    connector J1305 (with pullups to 1.8V).  It uses the MCU's internal I2C2 hw block.

        //  The inputs to this command are:
        //    I2C_TargetAddress, 1 byte (7 bit I2C chip address)
        //    I2C_starting_register_address, 1 byte
        //    Write_length, 1 byte (1 to 60 bytes is valid)
        //    Data, 1-60 bytes
        //
        //  This returns a byte array which contains:
        //    status byte (0x00 = Success, 0xFF = Failure)
        //    I2C "event" status (2 bytes).  Search for "ARM_I2C_EVENT_TRANSFER_DONE" for bit definitions
        //      0x01 is "done", 0x10 is "cleared", other bits indicate errors.
        //
        uint8_t target_addr = *request++;
        const uint8_t* internal_addr = request++;
        uint8_t len = *request++;
        uint8_t data_buf[60] = {0};
        uint32_t returnVal = 0;

        for (int i = 0; i < len; i++) {
            data_buf[i] = *request++;
        }

        // DONE: can add additional responses from I2C_DAP_MasterRead to provide better status
        returnVal = I2C_DAP_MasterTransfer(target_addr, internal_addr, data_buf, len);
        if (returnVal== 0x01) {
            // transfer done
            *response++ = DAP_OK;
        } else {
            // transfer incomplete
            *response++ = DAP_ERROR;
        }
        *response++ = (returnVal & 0xFF); //9 bits in an event.  Get first byte
        *response++ = ((returnVal >> 8) & 0xFF); //Get 2nd byte

        //3 + len bytes read for the command, 3 bytes returned (status, returnVal)
        num += ((3 + len) << 16) | (3);
        break;
    }
    case ID_DAP_VendorEx34_GPIO: {
        //  Write GPIO pins (as OpenDrain or PushPull) and also read back values.
        //
        //  There are several gpio pins used to drive DUT signals.  They can be
        //  "open-drain" (for many products), or "push-pull" for some adaptor boards.
        //
        //  This command has the following bytes:
        //    OUTPUT_VALUE, DIRECTION, MASK (optional), DURATION(future):
        //
        //  OUTPUT_VALUE byte:
        //    bit 0 : 0_RST_L.  0 = pull low, 1 = pull high
        //    bit 1 : 0_BOOT_L
        //    bit 2 : 0_BTN_L
        //    bit 3 : RSVD
        //    bit 4 : 1_RST_H
        //    bit 5 : 1_BOOT_H
        //    bit 6 : 1_BTN_H
        //    bit 7 :  RSVD
        //
        //  DIRECTION byte:
        //    Same bit mapping as OUTPUT_VALUE. 0 = INPUT, 1 = OUTPUT
        //
        //  MASK byte:
        //    Same bit mapping as OUTPUT_VALUE. 0 = MASKED (don't write to that pin), 1 = write to that pin.
        //  DURATION (future)
        //    0.1 sec increments, 0 = no pulse (stay at that level).  1-255 = 0.1 to 25.5 sec pulse duration.
        //    TBD: blocking or non-blocking?  Blocking is simplest, might need to increase timeout on host, though.
        //    For a 0.1 sec pulse it is probably fine.
        //
        //  This command also reads the values from all the gpios (even if writing is masked)
        //  It returns 2 bytes, the first byte is the DIRECTION, the 2nd byte is the PIN_VALUE
        //  NOTE: If you just want to READ the pins, this command can be used with 0x00, 0x00, 0x00 (to mask writing to any pins)
        //
        uint8_t pin_values = *request++;            // When is an output, should it pull HIGH (1) or LOW (0)?  For
                                                    //  "open-drain" operation it should be 0 to pull low.  For
                                                    //  "push-pull" toggle between 1 and 0.
        uint8_t pin_direction = *request++;         // Should the buffer drive the DUT (1), or be an input (0) from DUT?
        uint8_t pin_mask = *request;                // Masking byte, to allow indiviual pins to be set.  1=set,
                                                    //  0=don't change that pin.

        uint16_t pin_readback = 0;

        //Port0 WRITE
        if (pin_mask & 0x01)
            write_gpio(UDC0_RST_L_PORT, UDC0_RST_L_PIN_Bit,
                       UDC0_RST_L_DIR_PORT, UDC0_RST_L_DIR_PIN,
                       pin_direction, pin_values,
                       0x01);

        if (pin_mask & 0x02)
            write_gpio(UDC0_BOOT_L_PORT, UDC0_BOOT_L_PIN_Bit,
                       UDC0_BOOT_L_DIR_PORT, UDC0_BOOT_L_DIR_PIN,
                       pin_direction, pin_values,
                       0x02);
        if (pin_mask & 0x04)
            write_gpio(UDC0_BUTTON_L_PORT, UDC0_BUTTON_L_PIN_Bit,
                       UDC0_BUTTON_L_DIR_PORT, UDC0_BUTTON_L_DIR_PIN,
                       pin_direction, pin_values,
                       0x04);

        //Port1 WRITE
        if (pin_mask & 0x10)
            write_gpio(UDC1_RST_PORT, UDC1_RST_PIN_Bit,
                       UDC1_RST_DIR_PORT, UDC1_RST_DIR_PIN,
                       pin_direction, pin_values,
                       0x10);
        if (pin_mask & 0x20)
            write_gpio(UDC1_BOOT_PORT, UDC1_BOOT_PIN_Bit,
                       UDC1_BOOT_DIR_PORT, UDC1_BOOT_DIR_PIN,
                       pin_direction, pin_values,
                       0x20);
        if (pin_mask & 0x40)
            write_gpio(UDC1_BUTTON_PORT, UDC1_BUTTON_PIN_Bit,
                       UDC1_BUTTON_DIR_PORT, UDC1_BUTTON_DIR_PIN,
                       pin_direction, pin_values,
                       0x40);

        //Port0 READ
        pin_readback = read_gpio(UDC0_RST_L_PORT, UDC0_RST_L_PIN, UDC0_RST_L_DIR_PORT, UDC0_RST_L_DIR_PIN);
        pin_readback |= (read_gpio(UDC0_BOOT_L_PORT, UDC0_BOOT_L_PIN, UDC0_BOOT_L_DIR_PORT, UDC0_BOOT_L_DIR_PIN) << 1);
        pin_readback |= (read_gpio(UDC0_BUTTON_L_PORT, UDC0_BUTTON_L_PIN, UDC0_BUTTON_L_DIR_PORT, UDC0_BUTTON_L_DIR_PIN) << 2);

        //Port1 READ
        pin_readback |= (read_gpio(UDC1_RST_PORT, UDC1_RST_PIN, UDC1_RST_DIR_PORT, UDC1_RST_DIR_PIN) << 4);
        pin_readback |= (read_gpio(UDC1_BOOT_PORT, UDC1_BOOT_PIN, UDC1_BOOT_DIR_PORT, UDC1_BOOT_DIR_PIN) << 5);
        pin_readback |= (read_gpio(UDC1_BUTTON_PORT, UDC1_BUTTON_PIN, UDC1_BUTTON_DIR_PORT, UDC1_BUTTON_DIR_PIN) << 6);

        *response++ = ((pin_readback >> 8) & 0xFF); //Get MSByte (DIRECTION)
        *response++ = (pin_readback & 0xFF);        //Get LSByte (PIN_VALUES)

        num += (3U << 16) | 2U; // 3 bytes read, 2 byte written
        break;
    }
    case ID_DAP_VendorEx35_DUT_PWR_CTRL: {
        //  DUT Power control
        //
        //  This command controls several DUT power signals.  The command has the following bytes:
        //    OUTPUT_VALUE, MASK (optional), DURATION(future)
        //  Note: There is no DIRECTION byte, as these pins are "output only"
        //
        //  OUTPUT_VALUE byte:
        //    bit 0 : UDC_DUT_USB_EN (1 = enabled)
        //    bit 1 : UDC_EXT_RELAY (1 = enabled)
        //    bit 2 : bits 2-7 RSVD
        //
        //  MASK byte:
        //    bit 0 : UDC_DUT_USB_EN
        //    bit 1 : UDC_EXT_RELAY
        //    bit 2 : bits 2-7 RSVD
        //
        //  This command also reads the values from all the gpios (even if writing is masked)
        //  It returns 1 btye, with the PIN_VALUEs
        //  NOTE: If you just want to READ the pins, this command can be used with 0x00, 0x00, 0x00 (to mask writing to any pins)
        uint8_t pins = *request++;
        uint8_t mask = *request;

        uint8_t readback_byte = 0;

        if(mask & 0x1) {
            if(pins & 0x1)
              // ToDo: port to "HAL_xxx" functions
              // Set low to enable USB power
              UDC_DUT_USB_EN_L_PORT->BSRR = (uint32_t)UDC_DUT_USB_EN_L_PIN << 16;
            else
              UDC_DUT_USB_EN_L_PORT->BSRR = UDC_DUT_USB_EN_L_PIN;
          }
        if(mask & 0x2) {
            if(pins & 0x2)
              UDC_EXT_RELAY_PORT->BSRR = UDC_EXT_RELAY_PIN;
            else
              UDC_EXT_RELAY_PORT->BSRR = UDC_EXT_RELAY_PIN << 16;
          }

        //Readback actual values (even if write is masked out)
        readback_byte |= (UDC_DUT_USB_EN_L_PORT->IDR & UDC_DUT_USB_EN_L_PIN) ? 0x01 : 0x00;
        readback_byte |= (UDC_EXT_RELAY_PORT->IDR & UDC_EXT_RELAY_PIN) ? 0x02 : 0x00;

        *response++ = readback_byte;

        num += (2U << 16) | 1U; // 2 bytes read, 1 byte written
        break;
    }
    case ID_DAP_VendorEx36_INTERFACE_VERSION_DETAILS: {
        // Add a more specific internal version string.
        num += udb_get_interface_version(response, DAP_PACKET_SIZE - 1);
        break;
    }
    case ID_DAP_VendorEx37_HOLD_IN_BL:
    {
        // Request to stay in bootloader mode on the next boot for SWU
        *response = DAP_OK;
        config_ram_set_hold_in_bl(true);
        num += 1;
        break;
    }
    case ID_DAP_VendorEx38_RESET_DAPLINK:
    {
        // Request to reset the UDB.
        *response = DAP_OK;
        udb_reset_async(UDB_RESET_TIMER_MS);
        num += 1;
        break;
    }
    case ID_DAP_VendorEx39_READ_UDC_ADAPTER_TYPE_ADC:
    {
        adapter_type_t adapter = adapter_detector_get_adapter_type_adc();
        memcpy(response, &adapter, sizeof(adapter));
        num += sizeof(adapter);
        break;
    }
    case ID_DAP_VendorEx40_MEASURE_POWER:
    {
        num += DAP_ProcessVendorCommandEx40_MeasurePower(request, response);
        break;
    }
    case ID_DAP_VendorEx41_BOOTLOADER_VERSION_DETAILS: {
        // Add a more specific internal version string.
        num += udb_get_bootloader_version(response, DAP_PACKET_SIZE - 1);
        break;
    }
    default:
      break;
  }

  return (num);
}
