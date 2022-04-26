#if defined(CDC_B_ENDPOINT)

#include "rl_usb.h"
#include "usb_for_lib.h"
#include "util.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "nluif_udb-daplink.h"
#include "daplink.h"
#include "udb_log.h"
#include DAPLINK_MAIN_HEADER

static const char error_msg[] = "\r\n<OVERFLOW>\r\n";

int32_t USBD_CDC_B_ACM_PortInitialize(void)
{
    // Disable buffering of stdout, otherwise prints without newline won't
    // appear until stdout is flushed or sth else is printed
    setbuf(stdout, NULL);
    main_cdc_b_send_event();
    return 1;
}

int32_t USBD_CDC_B_ACM_PortSetControlLineState(uint16_t ctrl_bmp)
{
    // bit 0 of ctrl_bmp is DTR state. If the terminal implements
    // flow control, it will set DTR to 1 when cdc is connected
    // and set DTR to 0 when disconnected
    if (ctrl_bmp & 1)
    {
        udb_log_set_cdc_ready(true);
    }
    else
    {
        udb_log_set_cdc_ready(false);
    }
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
                udb_log_set_cdc_ready(true);
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

    if (!udb_log_cdc_ready())
    {
        udb_log_push(ptr, len_to_write);
        return len_to_write;
    }

    if (total_free < error_len) {
        // drop everything so that printf() don't keep polling
        return len_to_write;
    }

    write_free = total_free - error_len;

    for (int idx = 0; idx < len_to_write; idx++)
    {
        if (write_free < 2)
        {
            // No more space to write a \r\n in the worst case
            USBD_CDC_B_ACM_DataSend((uint8_t*)error_msg, error_len);
            break;
        }

        if (ptr[idx] == '\n')
        {
            USBD_CDC_B_ACM_DataSend((uint8_t*)"\r", 1);
            write_free--;
        }
        USBD_CDC_B_ACM_DataSend((uint8_t*)(ptr + idx), 1);
        write_free--;
    }

    return len_to_write;
}
#endif // CDC_B_ENDPOINT
