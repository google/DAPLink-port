#include "shell_cmd_fault.h"
#include "util.h"
#include <string.h>
#include <stdio.h>

void cmd_fault(int argc, char *argv[])
{
    if (strcmp(argv[1], "test_assert") == 0)
    {
        util_assert(false);
    }
    else
    {
        printf("ERROR: unknown cmd\n");
    }
}
