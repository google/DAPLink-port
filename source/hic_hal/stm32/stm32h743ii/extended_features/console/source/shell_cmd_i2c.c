#include "shell_cmd_i2c.h"
#include "i2c.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define I2C_ARGC_READ                   (5)
#define I2C_ARGC_WRITE                  (I2C_ARGC_READ + 1)

// range for 7-bit addressing
#define I2C_SLAVE_ADDR_MIN              (0x08)
#define I2C_SLAVE_ADDR_MAX              (0x77)
#define UDB_I2C_CMD_READ_BUFFER_SIZE    (32)
#define I2C_WRITE_MAX_LEN               (sizeof(uint32_t) / sizeof(uint8_t))

void cmd_i2c(int argc, char *argv[])
{
    if (strcmp(argv[1], "probe") == 0)
    {
        printf("probing...\n");
        uint8_t bus_id = strtoul(argv[2], NULL, 0);
        if (bus_id == 2)
        {
            for (uint16_t addr = I2C_SLAVE_ADDR_MIN; addr <= I2C_SLAVE_ADDR_MAX; ++addr)
            {
                uint8_t reg = 0;
                if (I2C_DAP_MasterTransfer(addr, &reg, 0, 0) == true)
                {
                    printf("%#.*x\n", 2, addr);
                }
            }
        }
        else
        {
            printf("Error: UDB only supports bus 2\n");
        }
    }
    else if ((argc == I2C_ARGC_READ) || (argc == I2C_ARGC_WRITE))
    {
        uint8_t bus_id = strtoul(argv[1], NULL, 0);
        uint16_t slave_addr = strtoul(argv[2], NULL, 0);
        uint8_t start_reg = strtoul(argv[3], NULL, 0);
        uint16_t len = strtoul(argv[4], NULL, 0);
        if (bus_id == 2)
        {
            if (argc == I2C_ARGC_READ)
            {
                printf("reading..\n");

                uint8_t buffer[UDB_I2C_CMD_READ_BUFFER_SIZE];
                bool res;

                if (len > UDB_I2C_CMD_READ_BUFFER_SIZE)
                {
                    len = UDB_I2C_CMD_READ_BUFFER_SIZE;
                    printf("Error: max buffer size is %u\n", UDB_I2C_CMD_READ_BUFFER_SIZE);
                }
                res = I2C_DAP_MasterRead(slave_addr, &start_reg, buffer, len);
                if (res == true)
                {
                    for (int i = 0; i < len; ++i)
                    {
                        printf("%d:0x%x\n", i, buffer[i]);
                    }
                }
                else
                {
                    printf("Error: i2c read failed\n");
                }
            }
            else
            {
                printf("writing...\n");

                uint32_t val = strtoul(argv[5], NULL, 0);
                bool res;

                if (len > I2C_WRITE_MAX_LEN)
                {
                    len = I2C_WRITE_MAX_LEN;
                    printf("Error: max buffer size is %u\n", I2C_WRITE_MAX_LEN);
                }

                res = I2C_DAP_MasterTransfer(slave_addr, &start_reg, (uint8_t *)&val, len);
                if (res == false)
                {
                    printf("Error: i2c write failed\n");
                }
            }
        }
        else
        {
            printf("Error: UDB only supports bus 2\n");
        }
    }
    else
    {
        printf("Error: Invalid i2c command\n");
    }
}
