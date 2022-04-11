/**
 * @file    stm32h743ii.c
 * @brief   board ID for the STM32 NUCLEO-F103RB board
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2019, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "target_family.h"
#include "target_board.h"
#include <stdio.h>

#ifdef UDB
#include "i2c.h"
#include "udb_version.h"
#include "udb_extended_features_task.h"
#include "udb_power_measurement.h"
#include "udb_log.h"
#include "udb_version.h"
#include "nluif_udb-daplink.h"

static uint32_t s_count_blink = 0;

static void udb_welcome_message(void)
{
    char ver_buf[UDB_VERSION_MAX_LENGTH];

    printf("Welcome to\n");
    printf("\t          ______   ______      \n");
    printf("\t|\\     /|(  __  \\ (  ___ \\  \n");
    printf("\t| )   ( || (  \\  )| (   ) )   \n");
    printf("\t| |   | || |   ) || (__/ /     \n");
    printf("\t| |   | || |   | ||  __ (      \n");
    printf("\t| |   | || |   ) || (  \\ \\   \n");
    printf("\t| (___) || (__/  )| )___) )    \n");
    printf("\t(_______)(______/ |/ \\___/    \n\n");

    udb_get_interface_version((uint8_t*)ver_buf, UDB_VERSION_MAX_LENGTH);
    printf("Interface version: %s\n", ver_buf);

    printf("To know more about udb, visit go/udb.\n");
    printf("Please report issues at go/udb-bug.\n");
    printf("You can start typing commands.\n");

    uif_prompt();
}

static void prerun_board_config(void)
{
    I2C_DAP_Initialize();
    udb_read_hw_version();
    udb_extended_features_task_create();
    udb_power_measurement_init();
    udb_welcome_message();
}

void board_30ms_hook()
{
    s_count_blink++;
    if ((s_count_blink % 50) == 0)
    {
        HAL_GPIO_TogglePin( CONNECTED_LED_PORT, CONNECTED_LED_PIN);
    }

    if (udb_log_cdc_ready())
    {
        udb_log_flush();
    }
}
#endif // UDB

const board_info_t g_board_info =
{
    .info_version = kBoardInfoVersion,
    .board_id = "0000",
    .family_id = kStub_HWReset_FamilyID,
    .daplink_drive_name = "DAPLINK",
    .prerun_board_config = prerun_board_config,
    // .target_cfg = &target_device,   This board doesn't have a target
};
