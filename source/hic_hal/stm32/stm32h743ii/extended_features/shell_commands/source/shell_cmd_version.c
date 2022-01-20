#include "shell_cmd_version.h"
#include "udb_version.h"
#include <stdio.h>

void cmd_version(int argc, char *argv[])
{
    char buf[UDB_VERSION_MAX_LENGTH];
    udb_get_interface_version(buf, UDB_VERSION_MAX_LENGTH);
    printf("Interface ver: %s\n", buf);
    udb_get_bootloader_version(buf, UDB_VERSION_MAX_LENGTH);
    printf("Bootloader ver: %s\n\n", buf);
}
