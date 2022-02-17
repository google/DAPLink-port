#include "udb_fault_info.h"

#include <settings.h>
#include <stdio.h>
#include "udb_reset.h"

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
