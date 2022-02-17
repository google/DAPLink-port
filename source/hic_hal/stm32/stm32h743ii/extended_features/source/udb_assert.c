#ifdef BUILD_FEATURE_UDB_ASSERT

#include "settings.h"
#include "stm32h743xx.h"
#include "cmsis_os2.h"
#include "rtx_os.h"
#include "core_cm7.h"
#include "nlbacktrace_udb.h"
#include "cmsis_gcc.h"
#include "udb_reset.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdio.h>
#include "util.h"
#include "udb_fault_info.h"

#define UDB_BT_BUFFER_SIZE  (32)

// Flash memory bank 1 of stm32h743x
#define UDB_RAM_BASE_ADDRESS    FLASH_BANK1_BASE
#define UDB_RAM_TOP_ADDRESS     (FLASH_BANK2_BASE-1)

#define DEBUGGER_ATTACHED       ((CoreDebug->DHCSR & CoreDebug_DHCSR_S_RESET_ST_Msk) == 0)

static uint32_t s_udb_bt_buffer[UDB_BT_BUFFER_SIZE];

static uint32_t get_PC()
{
    register uint32_t result;
    __asm volatile ("mov %0, pc\n"  : "=r" (result));

    return result;
}

static uint32_t get_SP()
{
    register uint32_t result;
    __asm volatile ("mov %0, sp\n"  : "=r" (result) );

    return result;
}

static void udb_backtrace(const char *file, uint16_t line)
{
    uint32_t pc = get_PC();
    uint32_t sp = get_SP();
    memset(s_udb_bt_buffer, 0, sizeof(s_udb_bt_buffer));
    nlbacktrace(pc, sp, 0, s_udb_bt_buffer, ARRAY_SIZE(s_udb_bt_buffer));

    config_ram_clear_assert();
    config_ram_set_assert(file, line);
    printf("Oops! UDB hit an assert at:\nFile: %s, Line: %u\n", file, line);

    for (int i = 0; i < UDB_BT_BUFFER_SIZE && s_udb_bt_buffer[i]; ++i)
    {
        config_ram_add_hexdump(s_udb_bt_buffer[i] & (~1U));
    }

    udb_print_fault_info();
}

// overwrite the functions in backtrace.c

bool isAddressValidForCode(uint32_t addr)
{
    return (addr >= UDB_RAM_BASE_ADDRESS && addr <= UDB_RAM_TOP_ADDRESS);
}

bool isAddressValidForStack(uint32_t addr)
{
    // only work on RTX, but there's no better way to get the stack of the running task
    osRtxThread_t *thread = (osRtxThread_t*)osThreadGetId();
    uint32_t base = (uint32_t)thread->stack_mem;
    uint32_t top = ((uint32_t)thread->stack_mem) + thread->stack_size - 1;
    return (addr >= base && addr <= top);
}

void _util_assert(bool expression, const char *filename, uint16_t line)
{
    if (!expression)
    {
        __disable_irq();
        udb_backtrace(filename, line);

        HAL_Delay(300U);
        if (DEBUGGER_ATTACHED)
        {
            __BKPT(0);
        }
        else
        {
            udb_reset();
        }
    }
}

#endif // BUILD_FEATURE_UDB_ASSERT
