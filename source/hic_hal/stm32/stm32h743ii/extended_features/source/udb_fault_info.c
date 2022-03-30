#include "udb_fault_info.h"

#include <stdio.h>
#include <string.h>
#include <settings.h>
#include <stm32h7xx.h>
#include <stm32h743xx.h>
#include "nlbacktrace_udb.h"
#include "udb_reset.h"
#include "settings.h"
#include "util.h"

// This must be the same as ALLOWED_HEXDUMP in daplink/settings/settings.c
#define UDB_BT_BUFFER_SIZE  (16U)
static uint32_t s_udb_bt_buffer[UDB_BT_BUFFER_SIZE];

void udb_write_backtrace_info(const char *file, uint16_t line, uint32_t pc, uint32_t sp)
{
    memset(s_udb_bt_buffer, 0, sizeof(s_udb_bt_buffer));
    nlbacktrace(pc, sp, 0, s_udb_bt_buffer, ARRAY_SIZE(s_udb_bt_buffer));

    config_ram_clear_assert();
    config_ram_set_assert(file, line);
    printf("Oops! Found UDB fault at \nFile: %s, Line: %u\n", file, line);

    for (int i = 0; i < UDB_BT_BUFFER_SIZE && s_udb_bt_buffer[i]; ++i)
    {
        // if the cdc b is connected, print to the console directly
        printf("%lx\n", s_udb_bt_buffer[i]);

        config_ram_add_hexdump(s_udb_bt_buffer[i]);
    }
    printf("\n");
}

bool udb_is_fault_info_uncleared(void)
{
    return config_ram_get_assert(NULL, 0, NULL, NULL);
}

void udb_print_fault_info(void)
{
    if (udb_is_fault_info_uncleared())
    {
        char assert_file_name[ASSERT_BUF_SIZE];
        uint16_t assert_line;
        uint32_t* hexdumps;
        config_ram_get_assert(assert_file_name, ASSERT_BUF_SIZE, &assert_line, NULL);
        uint8_t num_hexdumps = config_ram_get_hexdumps(&hexdumps);

        printf("File: %s, Line: %u\n", assert_file_name, assert_line);
        for (uint8_t i = 0; i < num_hexdumps; ++i)
        {
            printf("%x\n", hexdumps[i]);
        }
    }
}

void udb_clear_fault_info(void)
{
    config_ram_clear_assert();
    // need to reset to clear the fault info message
    udb_reset();
}

void udb_check_unexpected_watchdog_reset(void)
{
    if ((RCC->RSR & RCC_RSR_WWDG1RSTF) != 0)
    {
        // Clear the reset source. It will not be cleared even if we reset MCU again.
        SET_BIT(RCC->RSR, RCC_RSR_RMVF);
        util_assert(false);
    }
}
