#include "rl_usb.h"
#include "util.h"
#include <stdint.h>

#if defined(UDB_DEBUG) && defined(CDC_B_ENDPOINT)

/*
   overwrite the _write() in libnosys to enable logging with printf()
 */
int _write(int file, char *ptr, int len)
{
    uint32_t total_free = USBD_CDC_B_ACM_DataFree();
    uint32_t size = MIN(total_free, (uint32_t)len);
    USBD_CDC_B_ACM_DataSend(ptr, size);

    // force USB to process the data
    USBD_Handler();
    return size;
}
#endif // UDB_DEBUG && CDC_B_ENDPOINT
