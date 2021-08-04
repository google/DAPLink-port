/*
 * Copyright (c) 2013-2020 Arm Limited. All rights reserved.
 * Copyright 2019, Cypress Semiconductor Corporation
 * or a subsidiary of Cypress Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ----------------------------------------------------------------------
 *
 * $Date:        20. May 2015
 * $Revision:    V1.10
 *
 * Project:      CMSIS-DAP Source
 * Title:        DAP_vendor.c CMSIS-DAP Vendor Commands
 *
 *---------------------------------------------------------------------------*/

#include "rl_usb.h"
#include "DAP_config.h"
#include "DAP.h"
#include "info.h"
#include "daplink.h"
#include DAPLINK_MAIN_HEADER
#include "uart.h"
#include "settings.h"
#include "target_family.h"
#include "flash_manager.h"
#include <string.h>
#include "i2c.h"
#include "version_git.h"

#ifdef INTERFACE_STM32H743
#include "stm32h7xx.h"
#endif

#ifdef DRAG_N_DROP_SUPPORT
#include "file_stream.h"
#endif

//**************************************************************************************************
/**
\defgroup DAP_Vendor_Adapt_gr Adapt Vendor Commands
\ingroup DAP_Vendor_gr
@{

The file DAP_vendor.c provides template source code for extension of a Debug Unit with
Vendor Commands. Copy this file to the project folder of the Debug Unit and add the
file to the MDK-ARM project under the file group Configuration.
*/

/** Process DAP Vendor Command and prepare Response Data
\param request   pointer to request data
\param response  pointer to response data
\return          number of bytes in response (lower 16 bits)
                 number of bytes in request (upper 16 bits)
*/
uint32_t DAP_ProcessVendorCommand(const uint8_t *request, uint8_t *response) {
  uint32_t num = (1U << 16) | 1U;  // count the Command ID byte

  *response++ = *request;        // copy Command ID

  switch (*request++) {          // first byte in request is Command ID
    case ID_DAP_Vendor0: {
        const char *id_str = info_get_unique_id();
        uint8_t len = strlen(id_str);
        *response++ = len;
        memcpy(response, id_str, len);
        num += (len + 1); // increment response count by ID length + length byte
        break;
    }
    case ID_DAP_Vendor1: {
        // get line coding
        int32_t read_len = sizeof(CDC_LINE_CODING);
        CDC_LINE_CODING cdc_line_coding;
        USBD_CDC_ACM_PortGetLineCoding(&cdc_line_coding);
        memcpy(response, &cdc_line_coding, read_len);
        num += (read_len + 1);
        break;
    }
    case ID_DAP_Vendor2: {
        // set uart configuration
        CDC_LINE_CODING cdc_line_coding;
        USBD_CDC_ACM_PortGetLineCoding(&cdc_line_coding);
        //set BaudRate
        uint32_t baud_rate = 0;
        memcpy(&baud_rate, request, sizeof(uint32_t));
        cdc_line_coding.dwDTERate = baud_rate;
        USBD_CDC_ACM_PortSetLineCoding(&cdc_line_coding);
        USBD_CDC_ACM_SendBreak(0);
        *response = 1;
        num += (sizeof(uint32_t) << 16) | 1;
        break;
    }
    case ID_DAP_Vendor3:  {
        // uart read
        int32_t read_len = 62;
        read_len = uart_read_data(response + 1, read_len);
        response[0] = read_len;
        // increment request and response count
        num += (read_len + 1);
        break;
    }
    case ID_DAP_Vendor4:  {
        // uart write
        int32_t write_len = *request;
        request++;
        uart_write_data((uint8_t *)request, write_len);
        *response = 1;
        num += ((write_len + 1) << 16) | 1;
        break;
    }
    case ID_DAP_Vendor5:  break;
    case ID_DAP_Vendor6:  break;
    case ID_DAP_Vendor7:  break;
    case ID_DAP_Vendor8: {
        *response = 1;
        if (0 == *request) {
            main_usb_set_test_mode(false);
        } else if (1 == *request) {
            main_usb_set_test_mode(true);
        } else {
            *response = 0;
        }
        num += (1U << 16) | 1U; // increment request and response count each by 1
        break;
    }
    case ID_DAP_Vendor9: {
        // reset target
        *response = 1;
        if (!config_get_auto_rst()) {
            target_set_state(RESET_RUN);
        }
        num += 1;
        break;
    }
#ifdef DRAG_N_DROP_SUPPORT
    case ID_DAP_Vendor10: {
        // open mass storage device stream
        *response = stream_open((stream_type_t)(*request));
        num += (1 << 16) | 1;
        break;
    }
    case ID_DAP_Vendor11: {
        // close mass storage device stream
        *response = stream_close();
        num += 1;
        break;
    }
    case ID_DAP_Vendor12: {
        // write to mass storage device
        uint32_t write_len = *request;
        request++;
        *response = stream_write((uint8_t *)request, write_len);
        num += ((write_len + 1) << 16) | 1;
        break;
    }
#endif
    case ID_DAP_Vendor13: {
        // switching between chip erase and page erase
        //              COMMAND(OUT Packet)
        //              BYTE 0 1000 1110 0x8D
        //              BYTE 1 Desired Mode:
        //                                              0x00 - Chip Erase
        //                                              nonzero - Page Erase
        //              RESPONSE(IN Packet)
        //              BYTE 0
        //                                              0x00 - OK
        *response = DAP_OK;
        if (0x00U == *request) {
            flash_manager_set_page_erase(false);
        } else {
            flash_manager_set_page_erase(true);
        }
        num += (1U << 16) | 1U; // increment request and response count each by 1
        break;
    }
    case ID_DAP_Vendor14: break;
    case ID_DAP_Vendor15: {
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
    case ID_DAP_Vendor16: {
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
    case ID_DAP_Vendor17: {
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
#ifdef INTERFACE_STM32H743
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
#endif /* INTERFACE_STM32H743 */
        break;
    }
    case ID_DAP_Vendor18: {
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
#ifdef INTERFACE_STM32H743
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
#endif /* INTERFACE_STM32H743 */
        break;
    }
    case ID_DAP_Vendor19: {
        // Add a more specific internal version string.

#ifdef INTERFACE_STM32H743

#if GIT_LOCAL_MODS == 1
#define GIT_LOCAL_MODS_STR "_modified"
#else
#define GIT_LOCAL_MODS_STR ""
#endif //GIT_LOCAL_MODS

#define STR_IMPL_(x) #x      //stringify argument
#define STR(x) STR_IMPL_(x)  //indirection to expand argument macros


        static char build_version_str[] = "udb_" STR(UDB_VERSION) "_" GIT_COMMIT_SHA GIT_LOCAL_MODS_STR;
        uint8_t len = strlen(build_version_str);

        uint8_t data_buf[63] = { 0 };

        *response++ = len;

        memcpy(data_buf, build_version_str, len);
        num += (len + 1); // increment response count by ID length + length byte

        for (int i = 0; i < 62; i++) {
            *response++ = data_buf[i];
        }


#endif /* INTERFACE_STM32H743 */
        break;
    }
    case ID_DAP_Vendor20: break;
    case ID_DAP_Vendor21: break;
    case ID_DAP_Vendor22: break;
    case ID_DAP_Vendor23: break;
    case ID_DAP_Vendor24: break;
    case ID_DAP_Vendor25: break;
    case ID_DAP_Vendor26: break;
    case ID_DAP_Vendor27: break;
    case ID_DAP_Vendor28: break;
    case ID_DAP_Vendor29: break;
    case ID_DAP_Vendor30: break;
    case ID_DAP_Vendor31: break;
    default: break;
  }

  return (num);
}

///@}
