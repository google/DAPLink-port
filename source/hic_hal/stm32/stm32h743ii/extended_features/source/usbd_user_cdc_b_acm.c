#include "rl_usb.h"
#include "util.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "nluif_udb-daplink.h"
#include "daplink.h"
#include DAPLINK_MAIN_HEADER

#if defined(UDB_DEBUG) && defined(CDC_B_ENDPOINT)

static const char error_msg[] = "\r\n<OVERFLOW>\r\n";

int32_t USBD_CDC_B_ACM_PortInitialize(void)
{
    // Disable buffering of stdout, otherwise prints without newline won't
    // appear until stdout is flushed or sth else is printed
    setbuf(stdout, NULL);
    main_cdc_b_send_event();
    return 1;
}

void cdc_b_process_event()
{
    uint8_t data[64];
    int32_t len_data = USBD_CDC_B_ACM_DataRead(data, sizeof(data));

    if (len_data)
    {
        for (uint8_t i = 0; i < len_data; i++)
        {
            if (uif_handle_input_char(data[i]))
            {
                uif_run_cmd();
                uif_prompt();
            }
        }
    }
    // Always process events
    main_cdc_b_send_event();
}

/*
   overwrite the _write() in libnosys to enable logging with printf()
 */
int _write(int file, char *ptr, int len_to_write)
{
    uint32_t write_free;
    uint32_t total_free = USBD_CDC_B_ACM_DataFree();
    uint32_t error_len = strlen(error_msg);

    if (total_free < error_len) {
        // No space
        return 0;
    }

    write_free = total_free - error_len;

    for (int idx = 0; idx < len_to_write; idx++)
    {
        if (write_free < 2)
        {
            // No more space to write a \r\n in the worst case
            USBD_CDC_B_ACM_DataSend(error_msg, error_len);
            break;
        }

        if (ptr[idx] == '\n')
        {
            USBD_CDC_B_ACM_DataSend("\r", 1);
            write_free--;
        }
        USBD_CDC_B_ACM_DataSend(ptr + idx, 1);
        write_free--;
    }

    // force USB to process the data
    USBD_Handler();
    return len_to_write;
}
#endif // UDB_DEBUG && CDC_B_ENDPOINT
