#include "DAP.h"
#include "settings.h"
#include <string.h>
#include "i2c.h"
#include "version_git.h"
#include "cortex_m.h"
#include "stm32h7xx.h"
#include "DAP_vendor_ex.h"
#include "DAP_config.h"
#include "read_ver.h"

#if GIT_LOCAL_MODS == 1
#define GIT_LOCAL_MODS_STR "_modified"
#else
#define GIT_LOCAL_MODS_STR ""
#endif //GIT_LOCAL_MODS

#define STR_IMPL_(x) #x      //stringify argument
#define STR(x) STR_IMPL_(x)  //indirection to expand argument macros

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
        //  Write GPIO pins (Open Drain / Open collector)
        //
        //  There are several gpio pins used as open-drain (pull-low) or open-collector (pull-high) signals.
        //  This command writes to those pins.  The command has the following bytes:
        //    VALUE, MASK (optional), DURATION(future):
        //
        //  VALUE byte:
        //    0_RST_L
        //    0_BOOT_L
        //    0_BTN_L
        //    RSVD
        //    1_RST_H
        //    1_BOOT_H
        //    1_BTN_H
        //    RSVD

        //  MASK byte:
        //    0_RST_L
        //    0_BOOT_L
        //    0_BTN_L
        //    RSVD
        //    1_RST_H
        //    1_BOOT_H
        //    1_BTN_H
        //    RSVD

        //  DURATION (future)
        //    0.1 sec increments, 0 = no pulse (stay at that level).  1-255 = 0.1 to 25.5 sec pulse duration.
        //    TBD: blocking or non-blocking?  Blocking is simplest, might need to increase timeout on host, though.
        //    For a 0.1 sec pulse it is probably fine.
        //
        uint8_t pins = *request++;
        uint8_t mask = *request;

        *response = DAP_OK;

        //Port0
        if(mask & 0x1) {
            if(pins & 0x1)
              UDC0_RST_L_DIR_PORT->BSRR = UDC0_RST_L_DIR_PIN;
            else
              UDC0_RST_L_DIR_PORT->BSRR = (uint32_t)UDC0_RST_L_DIR_PIN << 16;
          }
        if(mask & 0x2) {
            if(pins & 0x2)
              UDC0_BOOT_L_DIR_PORT->BSRR = UDC0_BOOT_L_DIR_PIN;
            else
              UDC0_BOOT_L_DIR_PORT->BSRR = (uint32_t)UDC0_BOOT_L_DIR_PIN << 16;
          }
        if(mask & 0x4) {
            if(pins & 0x4)
              UDC0_BUTTON_L_DIR_PORT->BSRR = UDC0_BUTTON_L_DIR_PIN;
            else
              UDC0_BUTTON_L_DIR_PORT->BSRR = UDC0_BUTTON_L_DIR_PIN << 16;
          }

        //Port1
        if(mask & 0x10) {
            if(pins & 0x10)
              UDC1_RST_DIR_PORT->BSRR = UDC1_RST_DIR_PIN;
            else
              UDC1_RST_DIR_PORT->BSRR = (uint32_t)UDC1_RST_DIR_PIN << 16;
          }
        if(mask & 0x20) {
            if(pins & 0x20)
              //Can use port1 boot pin to get an active high signal
              UDC1_BOOT_DIR_PORT->BSRR = UDC1_BOOT_DIR_PIN;
            else
              UDC1_BOOT_DIR_PORT->BSRR = (uint32_t)UDC1_BOOT_DIR_PIN << 16;
          }
        if(mask & 0x40) {
            if(pins & 0x40)
              UDC1_BUTTON_DIR_PORT->BSRR = UDC1_BUTTON_DIR_PIN;
            else
              UDC1_BUTTON_DIR_PORT->BSRR = UDC1_BUTTON_DIR_PIN << 16;
          }

        //ToDo(elee): zero out the rest of the response buffer?  Can see stale data (from request) in pyOCD.
        num += (2U << 16) | 1U; // 2 bytes read, 1 byte written
        break;
    }
    case ID_DAP_VendorEx35_DUT_PWR_CTRL: {
        //  DUT Power control
        //
        //  This command controls several DUT power signals.  The command has the following bytes:
        //    VALUE, MASK (optional), DURATION(future):
        //
        //  VALUE byte:
        //    UDC_DUT_USB_EN (1 = enabled)
        //    UDC_EXT_RELAY (1 = enabled)
        //    bits 2-7 RSVD
        //
        //  MASK byte:
        //    UDC_DUT_USB_EN
        //    UDC_EXT_RELAY
        //    bits 2-7 RSVD
        //
        uint8_t pins = *request++;
        uint8_t mask = *request;

        *response = DAP_OK;

        if(mask & 0x1) {
            if(pins & 0x1)
              //Set low to enable USB power
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

        //ToDo(elee): zero out the rest of the response buffer?  Can see stale data (from request) in pyOCD.
        num += (2U << 16) | 1U; // 2 bytes read, 1 byte written
        break;
    }
    case ID_DAP_VendorEx36_VERSION_DETAILS: {
        // Add a more specific internal version string.
        static char version_suffix[] = "_" GIT_COMMIT_SHA GIT_LOCAL_MODS_STR;
        static char build_version_str[DAP_PACKET_SIZE];
        memset(build_version_str, 0, sizeof(build_version_str));
        strcat(build_version_str, get_udb_board_version());
        strcat(build_version_str, version_suffix);

        uint8_t len = strlen(build_version_str);

        uint8_t data_buf[DAP_PACKET_SIZE] = { 0 };

        *response++ = len;

        memcpy(data_buf, build_version_str, len);
        num += (len + 1); // increment response count by ID length + length byte

        for (int i = 0; i < (DAP_PACKET_SIZE - 1); i++) {
            *response++ = data_buf[i];
        }

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
        // Request to reset the UDB. PyOCD will crash.
        *response = DAP_OK;

        SystemReset();
        // We should be resetting here
        while(1){};
        num += 1;
        break;
    }
    default: break;
  }

  return (num);
}
