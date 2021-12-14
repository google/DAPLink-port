#include "shell_cmd_fault.h"
#include "udb_assert.h"
#include <string.h>
#include <stdio.h>

void cmd_fault(int argc, char *argv[])
{
    if (strcmp(argv[1], "test_assert") == 0)
    {
        udb_assert(0);
    }
    else
    {
        printf("ERROR: unknown cmd\n");
    }
}
