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

#ifdef UDB
#include "i2c.h"
#include "udb_version.h"
#include "udb_extended_features_task.h"

static uint32_t s_count_blink = 0;

static void prerun_board_config(void)
{
    I2C_DAP_Initialize();
    udb_read_hw_version();
    udb_extended_features_task_create();
}

void board_30ms_hook()
{
    s_count_blink++;
    if ((s_count_blink % 50) == 0)
    {
        HAL_GPIO_TogglePin( CONNECTED_LED_PORT, CONNECTED_LED_PIN);
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
