/*----------------------------------------------------------------------------
 *      RL-ARM - USB
 *----------------------------------------------------------------------------
 *      Name:    usbd_STM32H7xx_HS.c
 *      Purpose: Hardware Layer module for ST STM32H743
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <rl_usb.h>
#include "stm32h7xx.h"
#include "cmsis_gcc.h"
#include "usb_def.h"

#define __NO_USB_LIB_C
#include "usb_config.c"

#define OTG             USB1_OTG_HS
#define USBx_BASE       USB1_OTG_HS_PERIPH_BASE
#define USBx_DEVICE     ((USB_OTG_DeviceTypeDef *)(USBx_BASE + USB_OTG_DEVICE_BASE))

// If a change to the FIFO sizes is required, first investigate the max packet size
// and the driver for the respective endpoints.
#define RX_FIFO_SIZE    1024
#define TX0_FIFO_SIZE   64
#define TX1_FIFO_SIZE   512
#define TX2_FIFO_SIZE   512
#define TX3_FIFO_SIZE   512
#define TX4_FIFO_SIZE   512
#define TX5_FIFO_SIZE   448
#define TX6_FIFO_SIZE   512
// This chip has 4kB shared RAM for the FIFOs. Check if the FIFOs combined are within this limit.
COMPILER_ASSERT((RX_FIFO_SIZE + TX0_FIFO_SIZE + TX1_FIFO_SIZE + TX2_FIFO_SIZE + TX3_FIFO_SIZE +
                 TX4_FIFO_SIZE + TX5_FIFO_SIZE + TX6_FIFO_SIZE) <= 4096);

// 00: Control, 01: Isochronous, 10: Bulk, 11: Interrupt
#define EP_IN_TYPE(num)     ((USBx_INEP(num)->DIEPCTL & USB_OTG_DIEPCTL_EPTYP) >> USB_OTG_DIEPCTL_EPTYP_Pos)
#define EP_OUT_TYPE(num)    ((USBx_OUTEP(num)->DOEPCTL & USB_OTG_DOEPCTL_EPTYP) >> USB_OTG_DOEPCTL_EPTYP_Pos)

static uint32_t OutMaxPacketSize[USBD_EP_NUM];
static uint8_t  OutPacketCnt[USBD_EP_NUM];
static uint8_t  InPacketCnt[USBD_EP_NUM];
static uint32_t InPacketDataCnt[USBD_EP_NUM];
static uint32_t InPacketDataReady;
static uint32_t SyncWriteEP;

#if (USBD_HID_ENABLE == 1)
static uint32_t HID_IntInPacketData[(USBD_HID_MAX_PACKET + 3) / 4];
#endif

#if (USBD_CDC_ACM_ENABLE == 1)
static uint32_t CDC_ACM_IntInPacketData[(USBD_CDC_ACM_MAX_PACKET + 3) / 4];
#endif

#if (USBD_CDC_B_ACM_ENABLE == 1)
static uint32_t CDC_B_ACM_IntInPacketData[(USBD_CDC_B_ACM_MAX_PACKET + 3) / 4];
#endif

static uint32_t *InPacketDataPtr[USBD_EP_NUM] =
{
/* endpoint 0 */
    0,
/* endpoint 1 */
#if ((USBD_HID_ENABLE == 1) && (USBD_HID_EP_INTIN == 1))
    HID_IntInPacketData,
#elif ((USBD_CDC_ACM_ENABLE == 1) && (USBD_CDC_ACM_EP_INTIN == 1))
    CDC_ACM_IntInPacketData,
#elif (USBD_EP_NUM > 1)
    0,
#endif
/* endpoint 2 */
#if ((USBD_HID_ENABLE == 1) && (USBD_HID_EP_INTIN == 2))
    HID_IntInPacketData,
#elif ((USBD_CDC_ACM_ENABLE == 1) && (USBD_CDC_ACM_EP_INTIN == 2))
    CDC_ACM_IntInPacketData,
#elif (USBD_EP_NUM > 2)
    0,
#endif
/* endpoint 3 */
#if ((USBD_HID_ENABLE == 1) && (USBD_HID_EP_INTIN == 3))
    HID_IntInPacketData,
#elif ((USBD_CDC_ACM_ENABLE == 1) && (USBD_CDC_ACM_EP_INTIN == 3))
    CDC_ACM_IntInPacketData,
#elif (USBD_EP_NUM > 3)
    0,
#endif
/* endpoint 4 */
#if ((USBD_HID_ENABLE == 1) && (USBD_HID_EP_INTIN == 4))
    HID_IntInPacketData,
#elif ((USBD_CDC_ACM_ENABLE == 1) && (USBD_CDC_ACM_EP_INTIN == 4))
    CDC_ACM_IntInPacketData,
#elif (USBD_EP_NUM > 4)
    0,
#endif
/* endpoint 5 */
#if ((USBD_HID_ENABLE == 1) && (USBD_HID_EP_INTIN == 5))
    HID_IntInPacketData,
#elif ((USBD_CDC_B_ACM_ENABLE == 1) && (USBD_CDC_B_ACM_EP_INTIN == 5))
    CDC_B_ACM_IntInPacketData,
#elif (USBD_EP_NUM > 5)
    0,
#endif
/* endpoint 6 */
#if ((USBD_HID_ENABLE == 1) && (USBD_HID_EP_INTIN == 6))
    HID_IntInPacketData,
#elif ((USBD_CDC_B_ACM_ENABLE == 1) && (USBD_CDC_B_ACM_EP_INTIN == 6))
    CDC_B_ACM_IntInPacketData,
#elif (USBD_EP_NUM > 6)
    0,
#endif
};

/*
 *  USB Device Interrupt enable
 *   Called by USBD_Init to enable the USB Interrupt
 *    Return Value:    None
 */

static void USBD_IntrEna(void)
{
    NVIC_EnableIRQ(OTG_HS_IRQn); /* Enable OTG_HS interrupt */
}

/*
 *  USB Device Initialize Function
 *   Called by the User to initialize USB
 *   Return Value:    None
 */

void USBD_Init(void)
{
    int32_t tout;
    GPIO_InitTypeDef gpio_init_struct;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();

    // PA5 (OTG_HS_ULPI alternate function, CLOCK)
    gpio_init_struct.Pin = GPIO_PIN_5;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_struct.Alternate = GPIO_AF10_OTG1_HS;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    // ULPI data pins
    // PA3 (OTG_HS_ULPI alternate function, DATA0)
    gpio_init_struct.Pin = GPIO_PIN_3;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_struct.Alternate = GPIO_AF10_OTG1_HS;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    // PB0, PB1 (OTG_HS_ULPI alternate function, DATA1, DATA2)
    // PB10..13 (OTG_HS_ULPI alternate function, DATA3 to DATA6)
    // PB5 (OTG_HS_ULPI alternate function, DATA7)
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_5 |
                           GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Alternate = GPIO_AF10_OTG1_HS;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);

    // ULPI control pins
    // PC0 (OTG_HS_ULPI alternate function, STP)
    gpio_init_struct.Pin = GPIO_PIN_0;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Alternate = GPIO_AF10_OTG1_HS;
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);

    // PH4 (OTG_HS_ULPI alternate function, NXT)
    gpio_init_struct.Pin = GPIO_PIN_4;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Alternate = GPIO_AF10_OTG1_HS;
    HAL_GPIO_Init(GPIOH, &gpio_init_struct);

    // PI11 (OTG_HS_ULPI alternate functon, DIR)
    gpio_init_struct.Pin = GPIO_PIN_11;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Alternate = GPIO_AF10_OTG1_HS;
    HAL_GPIO_Init(GPIOI, &gpio_init_struct);

    __HAL_RCC_USB1_OTG_HS_ULPI_CLK_ENABLE();
    __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();

    HAL_Delay(20);
    tout = 1000;           // Wait max 1 s for AHBIDL = 0
    while (!(OTG->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL))
    {
        if (!tout)
        {
            break;
        }
        tout--;
        HAL_Delay(1);
    }
    OTG->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
    // reset otg core
    tout = 1000; // Wait max 1 s for CRST = 0
    while (OTG->GRSTCTL & USB_OTG_GRSTCTL_CSRST)
    {
        if (!tout)
        {
            break;
        }
        tout--;
        HAL_Delay(1);
    }
    HAL_Delay(3);

    OTG->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT; // Disable interrupts
    // No VBUS sensing
    // ToDo(elee): do we want this disabled?
    OTG->GCCFG &= ~USB_OTG_GCCFG_VBDEN;
    USBx_DEVICE->DCTL |= USB_OTG_DCTL_SDIS; // soft disconnect enabled

    // Force device mode
    OTG->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
    HAL_Delay(100);

    USBx_DEVICE->DCFG &= ~USB_OTG_DCFG_DSPD; // High speed phy

    OTG->GINTMSK =
        USB_OTG_GINTMSK_USBSUSPM |  // suspend int unmask
        USB_OTG_GINTMSK_USBRST |    // reset int unmask
        USB_OTG_GINTMSK_ENUMDNEM |  // enumeration done int unmask
        USB_OTG_GINTMSK_RXFLVLM |   // receive fifo non-empty int  unmask
        USB_OTG_GINTMSK_IEPINT |    // IN EP int unmask
        USB_OTG_GINTMSK_OEPINT |    // OUT EP int unmask
        USB_OTG_GINTMSK_WUIM_Msk |  // resume int unmask
        ((USBD_P_SOF_Event != 0) ? USB_OTG_GINTMSK_SOFM : 0); // SOF int unmask

    USBD_IntrEna(); // Enable OTG interrupt
    OTG->GAHBCFG |= USB_OTG_GAHBCFG_GINT | USB_OTG_GAHBCFG_TXFELVL;
}

/*
 *  USB Device Connect Function
 *   Called by the User to Connect/Disconnect USB Device
 *    Parameters:      con:   Connect/Disconnect
 *    Return Value:    None
 */

void USBD_Connect(BOOL con)
{
    if (con)
    {
        USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS; // soft disconnect disabled
    }
    else
    {
        USBx_DEVICE->DCTL |= USB_OTG_DCTL_SDIS; // soft disconnect enabled
    }
}

/*
 *  USB Device Reset Function
 *   Called automatically on USB Device Reset
 *    Return Value:    None
 */

void USBD_Reset(void)
{
    uint32_t i;

    SyncWriteEP = 0;
    InPacketDataReady = 0;
    USBx_DEVICE->DOEPMSK = 0;
    USBx_DEVICE->DIEPMSK = 0;

    for (i = 0; i < (USBD_EP_NUM + 1); i++)
    {
        if (USBx_OUTEP(i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA)
        {
            // OUT EP disable, Set NAK
            USBx_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK;
        }
        if (USBx_INEP(i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
        {
            // IN EP disable, Set NAK
            USBx_INEP(i)->DIEPCTL =
                USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
        }

        USBx_INEP(i)->DIEPINT = 0x1B;
        // clear IN Ep interrupts
        // Todo(elee): confirm no other interrupts matter
        USBx_OUTEP(i)->DOEPINT = 0x1B; // clear OUT Ep interrupts
    }

    USBD_SetAddress(0, 1);

    USBx_DEVICE->DAINTMSK = (1 << USB_OTG_DAINTMSK_OEPM_Pos) | // unmask IN&OUT EP0 interruts
                            (1 << USB_OTG_DAINTMSK_IEPM_Pos);
    USBx_DEVICE->DOEPMSK = USB_OTG_DOEPMSK_STUPM |  // setup phase done
                           USB_OTG_DOEPMSK_EPDM |   // endpoint disabled
                           USB_OTG_DOEPMSK_XFRCM;   // transfer complete
    USBx_DEVICE->DIEPMSK = USB_OTG_DIEPMSK_EPDM |   // endpoint disabled
                           USB_OTG_DIEPMSK_XFRCM;   // transfer completed

    OTG->GRXFSIZ = RX_FIFO_SIZE / 4;

    OTG->DIEPTXF0_HNPTXFSIZ = (RX_FIFO_SIZE / 4) | ((TX0_FIFO_SIZE / 4) << 16);

    OTG->DIEPTXF[0] =
        ((RX_FIFO_SIZE + TX0_FIFO_SIZE) / 4) | ((TX1_FIFO_SIZE / 4) << 16);

    // DIEPTXF2
    OTG->DIEPTXF[1] = ((RX_FIFO_SIZE + TX0_FIFO_SIZE + TX1_FIFO_SIZE) / 4) |
                      ((TX2_FIFO_SIZE / 4) << 16);

    // DIEPTXF3
    OTG->DIEPTXF[2] =
        ((RX_FIFO_SIZE + TX0_FIFO_SIZE + TX1_FIFO_SIZE + TX2_FIFO_SIZE) / 4) |
        ((TX3_FIFO_SIZE / 4) << 16);

    // DIEPTXF4
    OTG->DIEPTXF[3] = ((RX_FIFO_SIZE + TX0_FIFO_SIZE + TX1_FIFO_SIZE +
                        TX2_FIFO_SIZE + TX3_FIFO_SIZE) / 4) |
                      ((TX4_FIFO_SIZE / 4) << 16);
    // DIEPTXF5
    OTG->DIEPTXF[4]  = ((RX_FIFO_SIZE + TX0_FIFO_SIZE + TX1_FIFO_SIZE + TX2_FIFO_SIZE + TX3_FIFO_SIZE + TX4_FIFO_SIZE)/4) |
                       ((TX5_FIFO_SIZE/4) << 16);

    // DIEPTXF6
    OTG->DIEPTXF[5]  = ((RX_FIFO_SIZE + TX0_FIFO_SIZE + TX1_FIFO_SIZE + TX2_FIFO_SIZE + TX3_FIFO_SIZE + TX4_FIFO_SIZE + TX5_FIFO_SIZE)/4) |
                       ((TX6_FIFO_SIZE/4) << 16);

    USBx_OUTEP(0)->DOEPTSIZ = (1 << USB_OTG_DOEPTSIZ_STUPCNT_Pos) | // setup count = 1
                              (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos) |  // packet count
                              USBD_MAX_PACKET0;

    OutMaxPacketSize[0] = USBD_MAX_PACKET0;
    OutPacketCnt[0] = 1;
    InPacketCnt[0] = 1;
}

/*
 *  USB Device Suspend Function
 *   Called automatically on USB Device Suspend
 *    Return Value:    None
 */

void USBD_Suspend(void) {}

/*
 *  USB Device Resume Function
 *   Called automatically on USB Device Resume
 *    Return Value:    None
 */

void USBD_Resume(void) {}

/*
 *  USB Device Remote Wakeup Function
 *   Called automatically on USB Device Remote Wakeup
 *    Return Value:    None
 */

void USBD_WakeUp(void)
{
    USBx_DEVICE->DCTL |= USB_OTG_DCTL_RWUSIG; // remote wakeup signaling
    HAL_Delay(5);
    USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;
}

/*
 *  USB Device Remote Wakeup Configuration Function
 *    Parameters:      cfg:   Device Enable/Disable
 *    Return Value:    None
 */

void USBD_WakeUpCfg(BOOL cfg) {}

/*
 *  USB Device Set Address Function
 *    Parameters:      adr:   USB Device Address
 *    Return Value:    None
 */

void USBD_SetAddress(U32 adr, U32 setup)
{
    if (setup)
    {
        USBx_DEVICE->DCFG = (USBx_DEVICE->DCFG & ~USB_OTG_DCFG_DAD) |
                            (adr << USB_OTG_DCFG_DAD_Pos);
    }
}

/*
 *  USB Device Configure Function
 *    Parameters:      cfg:   Device Configure/Deconfigure
 *    Return Value:    None
 */

void USBD_Configure(BOOL cfg)
{
    InPacketDataReady &= ~1UL;
}

/*
 *  Configure USB Device Endpoint according to Descriptor
 *    Parameters:      pEPD:  Pointer to Device Endpoint Descriptor
 *    Return Value:    None
 */

void USBD_ConfigEP(USB_ENDPOINT_DESCRIPTOR *pEPD)
{
    uint32_t num, val, type;

    num = pEPD->bEndpointAddress & ~USB_ENDPOINT_DIRECTION_MASK;
    val = pEPD->wMaxPacketSize;
    type = pEPD->bmAttributes & USB_ENDPOINT_TYPE_MASK;

    if (pEPD->bEndpointAddress & USB_ENDPOINT_DIRECTION_MASK)
    {
        InPacketCnt[num] = 1;

        USBx_DEVICE->DAINTMSK |= (1 << num);     // unmask IN EP int
        USBx_INEP(num)->DIEPCTL = (num << USB_OTG_DIEPCTL_TXFNUM_Pos) |  // fifo number
                                  (type << USB_OTG_DIEPCTL_EPTYP_Pos) | // ep type
                                  (val & USB_OTG_DIEPCTL_MPSIZ);   // max packet size
        if ((type & 3) > 1)
        {
            // if interrupt or bulk EP
            USBx_INEP(num)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
        }
    }
    else
    {
        OutMaxPacketSize[num] = val & USB_OTG_DOEPCTL_MPSIZ;
        OutPacketCnt[num] = 1;

        USBx_DEVICE->DAINTMSK |= (1 << (num + 16)); // unmask OUT EP int

        USBx_OUTEP(num)->DOEPCTL = (type << USB_OTG_DOEPCTL_EPTYP_Pos) | // EP type
                                   (val & USB_OTG_DOEPCTL_MPSIZ); // max packet size

        USBx_OUTEP(num)->DOEPTSIZ = (OutPacketCnt[num] << USB_OTG_DOEPTSIZ_PKTCNT_Pos) | // packet count
                                    (val & USB_OTG_DOEPCTL_MPSIZ); // transfer size
        if (type > 1)
        {
            // if int or bulk EP
            USBx_OUTEP(num)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
        }
    }
}

/*
 *  Set Direction for USB Device Control Endpoint
 *    Parameters:      dir:   Out (dir == 0), In (dir <> 0)
 *    Return Value:    None
 */

void USBD_DirCtrlEP(U32 dir)
{
}

/*
 *  Enable USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_EnableEP(U32 EPNum)
{
    if (EPNum & USB_ENDPOINT_DIRECTION_MASK)
    {
        EPNum &= ~USB_ENDPOINT_DIRECTION_MASK;
        USBx_INEP(EPNum)->DIEPCTL |= USB_OTG_DIEPCTL_USBAEP | // EP active
                                     USB_OTG_DIEPCTL_SNAK;    // set EP NAK
        if (USBx_INEP(EPNum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
        {
            // disable EP
            USBx_INEP(EPNum)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
        }

        InPacketDataReady &= ~(1 << EPNum);
    }
    else
    {
        USBx_OUTEP(EPNum)->DOEPCTL |= USB_OTG_DOEPCTL_USBAEP |  // EP active
                                      USB_OTG_DOEPCTL_EPENA |   // enable EP
                                      USB_OTG_DOEPCTL_CNAK;     // clear EP NAK
    }
}

/*
 *  Disable USB Endpoint
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_DisableEP(U32 EPNum)
{
    uint32_t num, wcnt;

    // Disable IN Endpoint
    if (EPNum & USB_ENDPOINT_DIRECTION_MASK)
    {
        EPNum &= ~USB_ENDPOINT_DIRECTION_MASK;
        InPacketDataReady &= ~(1 << EPNum);

        if (USBx_INEP(EPNum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
        {
            // disable EP
            USBx_INEP(EPNum)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
        }
        // set EP NAK
        USBx_INEP(EPNum)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
        // deactivate EP
        USBx_INEP(EPNum)->DIEPCTL &= ~USB_OTG_DIEPCTL_USBAEP;

    }
    else
    {
        // Disable OUT Endpoint
        // set global out nak
        USBx_DEVICE->DCTL |= USB_OTG_DCTL_SGONAK;

        wcnt = 1000;
        while (!(OTG->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF))
        {
            // wait until global NAK
            if ((wcnt--) == 0)
            {
                break;
            }
        }

        if (USBx_OUTEP(EPNum)->DOEPCTL & USB_OTG_DOEPCTL_EPENA)
        {
            // disable EP
            USBx_OUTEP(EPNum)->DOEPCTL |= USB_OTG_DOEPCTL_EPDIS;
        }
        // set EP NAK
        USBx_OUTEP(EPNum)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
        //  deactivate EP
        USBx_OUTEP(EPNum)->DOEPCTL &= ~USB_OTG_DOEPCTL_USBAEP;

        wcnt = 1000;
        while (!(USBx_OUTEP(EPNum)->DOEPINT & USB_OTG_DOEPINT_EPDISD))
        {
            // wait until EP disabled
            if ((wcnt--) == 0)
            {
                break;
            }
        }
        // clear global nak
        USBx_DEVICE->DCTL |= USB_OTG_DCTL_CGONAK;
    }
}

/*
 *  Reset USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ResetEP(U32 EPNum)
{
    // Reset IN Endpoint
    if (EPNum & USB_ENDPOINT_DIRECTION_MASK)
    {
        EPNum &= ~USB_ENDPOINT_DIRECTION_MASK;
        InPacketDataReady &= ~(1 << EPNum);
        if (USBx_INEP(EPNum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
        {
            // disable EP
            USBx_INEP(EPNum)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
        }
        // set EP NAK
        USBx_INEP(EPNum)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;

        // Flush endpoint fifo
        USB_FlushTxFifo(OTG, EPNum);
    }
}

/*
 *  Set Stall for USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_SetStallEP(U32 EPNum)
{
    uint32_t wcnt;

    if (!(EPNum & USB_ENDPOINT_DIRECTION_MASK))
    {
        // Stall OUT Endpoint

        // set global out nak
        USBx_DEVICE->DCTL |= USB_OTG_DCTL_SGONAK;
        wcnt = 1000;
        while (!(OTG->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF))
        {
            // wait until global NAK
            if ((wcnt--) == 0)
            {
                break;
            }
        }

        if (USBx_OUTEP(EPNum)->DOEPCTL & USB_OTG_DOEPCTL_EPENA)
        {
            // disable EP
            USBx_OUTEP(EPNum)->DOEPCTL |= USB_OTG_DOEPCTL_EPDIS;
        }
        // set stall
        USBx_OUTEP(EPNum)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;

        wcnt = 1000;
        while (!(USBx_OUTEP(EPNum)->DOEPINT & USB_OTG_DOEPINT_EPDISD))
        {
            // wait until EP disabled
            if ((wcnt--) == 0)
            {
                break;
            }
        }

        // clear global nak
        USBx_DEVICE->DCTL |= USB_OTG_DCTL_CGONAK;

    }
    else
    {
        // Stall IN endpoint
        EPNum &= ~USB_ENDPOINT_DIRECTION_MASK;
        if (USBx_INEP(EPNum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
        {
            // disable endpoint
            USBx_INEP(EPNum)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
        }
        // set stall
        USBx_INEP(EPNum)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;

        USB_FlushTxFifo(OTG, EPNum);
    }
}

/*
 *  Clear Stall for USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ClrStallEP(U32 EPNum)
{
    if (!(EPNum & USB_ENDPOINT_DIRECTION_MASK))
    {
        // Clear OUT endpoint Stall
        if (EP_OUT_TYPE(EPNum) > 1)
        {
            // if EP type Bulk or Interrupt
            // Set DATA0 PID
            USBx_OUTEP(EPNum)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
            // Clear stall
            USBx_OUTEP(EPNum)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
        }
    }
    else
    {
        // Clear IN Endpoint Stall
        EPNum &= ~USB_ENDPOINT_DIRECTION_MASK;

        if (USBx_INEP(EPNum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
        {
            // disable endpoint
            USBx_INEP(EPNum)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
        }

        // Flush endpoint fifo
        USB_FlushTxFifo(OTG, EPNum);

        if (EP_IN_TYPE(EPNum) > 1)
        {
            // if Interrupt or bulk EP, Set DATA0 PID
            USBx_INEP(EPNum)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
        }

        // clear Stall
        USBx_INEP(EPNum)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
    }
}

/*
 *  Clear USB Device Endpoint Buffer
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ClearEPBuf(U32 EPNum)
{
    if (EPNum & USB_ENDPOINT_DIRECTION_MASK)
    {
        EPNum &= ~USB_ENDPOINT_DIRECTION_MASK;
        USB_FlushTxFifo(OTG, EPNum);
    }
    else
    {
        USB_FlushRxFifo(OTG);
    }
}

/*
 *  Read USB Device Endpoint Data
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *    Return Value:    Number of bytes read
 */

uint32_t USBD_ReadEP(U32 EPNum, U8 *pData, uint32_t bufsz)
{
    U32 val, sz;

    if ((USBx_OUTEP(EPNum)->DOEPCTL & USB_OTG_DOEPCTL_USBAEP) == 0)
    {
        // if Ep not active
        return (0);
    }

    sz = (OTG->GRXSTSP & USB_OTG_GRXSTSP_BCNT) >> USB_OTG_GRXSTSP_BCNT_Pos; // get available data size

    /* elee: copy from the f103 code, tbd, still required?
      Commit 5ffac262f,  and 3d1a68b768c57cb403bab9d5598f3e9f1d3506a2 (for the
      core) If smaller data is read here, is the rest of the data pulled in
      later? */
    if (sz > bufsz)
    {
        sz = bufsz;
    }

    // copy data from fifo if Isochronous Ep: data is copied to intermediate buffer
    for (val = 0; val < (uint32_t)((sz + 3) / 4); val++)
    {
        __UNALIGNED_UINT32_WRITE(pData, USBx_DFIFO(0U));
        pData += 4;
    }

    // wait RxFIFO non-empty (OUT transfer completed or Setup trans. completed)
    while ((OTG->GINTSTS & USB_OTG_GINTSTS_RXFLVL) == 0)
    {
    }
    OTG->GRXSTSP;                               // pop register
    OTG->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;    // unmask RxFIFO non-empty interrupt

    return (sz);
}

/*
 *  Write USB Device Endpoint Data
 *  If write was requested synchronously from IRQ then data is written to FIFO
 *  directly else data is written to the intermediate buffer and synchronously
 *  transferred to FIFO on next NAK event.
 *    Parameters:      EPNum: Device Endpoint Number
 *                        EPNum.0..3: Address
 *                        EPNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *                     cnt:   Number of bytes to write
 *    Return Value:    Number of bytes written
 */

uint32_t USBD_WriteEP(U32 EPNum, U8 *pData, U32 cnt)
{
    U32 *ptr, val;

    EPNum &= ~USB_ENDPOINT_DIRECTION_MASK;

    if ((USBx_INEP(EPNum)->DIEPCTL & USB_OTG_DIEPCTL_USBAEP) == 0)
    {
        // if Ep not active
        return (0);
    }

    /* Asynchronous write to intermediate buffer */
    if (!SyncWriteEP && InPacketDataPtr[EPNum])
    {
        if (!(InPacketDataReady & (1 << EPNum)))
        {
            InPacketDataCnt[EPNum] = cnt; /* save Data size */
            ptr = InPacketDataPtr[EPNum];
            val = (cnt + 3) / 4;
            if (val)
            {
                while (val--)
                {
                    // save data to intermediate buffer
                    *ptr++ = *((U32 *)pData);
                    pData += 4;
                }
            }
            InPacketDataReady |= 1 << EPNum;
            // Set NAK to enable interrupt on NAK
            USBx_INEP(EPNum)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
            // INEPNEM = 1, IN EP NAK efective msk
            USBx_DEVICE->DIEPMSK |= USB_OTG_DIEPMSK_INEPNEM;
        }
        else
        { /* If packet already loaded to buffer */
            return 0;
        }
    }
    else
    {
        if (cnt)
        {
            while ((USBx_INEP(EPNum)->DTXFSTS * 4) < cnt)
            {
                // get space in Ep TxFIFO
            }
        }

        // set transfer size and packet count
        USBx_INEP(EPNum)->DIEPTSIZ =
            cnt | (InPacketCnt[EPNum] << USB_OTG_DIEPTSIZ_PKTCNT_Pos) | (InPacketCnt[EPNum] << USB_OTG_DIEPTSIZ_MULCNT_Pos);

        // enable ep and clear NAK
        USBx_INEP(EPNum)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
        if (cnt)
        {
            ptr = (uint32_t *)pData;
            val = (cnt + 3) / 4;
            while (val--)
            {
                // copy data to endpoint TxFIFO
                USBx_DFIFO((uint32_t)EPNum) = __UNALIGNED_UINT32_READ(pData);
                pData += 4;
            }
        }
        InPacketDataReady &= ~(1 << EPNum);
    }
    return (cnt);
}

/*
 *  Get USB Device Last Frame Number
 *    Parameters:      None
 *    Return Value:    Frame Number
 */

uint32_t USBD_GetFrame(void)
{
    return ((USBx_DEVICE->DSTS & USB_OTG_DSTS_FNSOF) >> USB_OTG_DSTS_FNSOF_Pos);
}

/*
 *  USB Device Interrupt Service Routine
 */
void OTG_HS_IRQHandler(void)
{
    NVIC_DisableIRQ(OTG_HS_IRQn);
    USBD_SignalHandler();
}

void USBD_Handler(void)
{
    uint32_t istr, val, num, i, msk;

    istr = OTG->GINTSTS & OTG->GINTMSK;

    // reset interrupt
    if (istr & USB_OTG_GINTSTS_USBRST)
    {
        USBD_Reset();
        usbd_reset_core();
        if (USBD_P_Reset_Event)
        {
            USBD_P_Reset_Event();
        }
        OTG->GINTSTS = USB_OTG_GINTSTS_USBRST;
    }

    // suspend interrupt
    if (istr & USB_OTG_GINTSTS_USBSUSP)
    {
        USBD_Suspend();
        if (USBD_P_Suspend_Event)
        {
            USBD_P_Suspend_Event();
        }
        OTG->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
    }

    // resume interrupt
    if (istr & USB_OTG_GINTSTS_WKUINT)
    {
        USBD_Resume();
        if (USBD_P_Resume_Event)
        {
            USBD_P_Resume_Event();
        }
        OTG->GINTSTS = USB_OTG_GINTSTS_WKUINT;
    }

    // speed enumeration completed
    if (istr & USB_OTG_GINTSTS_ENUMDNE)
    {
        if (!((USBx_DEVICE->DSTS & USB_OTG_DSTS_ENUMSPD) >> USB_OTG_DSTS_ENUMSPD_Pos))
        {
            USBD_HighSpeed = 1;
        }
        USBx_INEP(0)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;
        USBx_INEP(0)->DIEPCTL |= OutMaxPacketSize[0]; // EP0 max packet
        USBx_DEVICE->DCTL |= USB_OTG_DCTL_CGINAK;     // clear global IN NAK
        USBx_DEVICE->DCTL |= USB_OTG_DCTL_CGONAK;     // clear global OUT NAK
        OTG->GINTSTS |= USB_OTG_GINTSTS_ENUMDNE;
    }

    // Start Of Frame
    if (istr & USB_OTG_GINTSTS_SOF)
    {
        if (USBD_P_SOF_Event)
        {
            USBD_P_SOF_Event();
        }
        OTG->GINTSTS = USB_OTG_GINTSTS_SOF;
    }

    // RxFIFO non-empty
    if (istr & USB_OTG_GINTSTS_RXFLVL)
    {
        val = OTG->GRXSTSR;
        num = val & USB_OTG_GRXSTSP_EPNUM;

        switch ((val & USB_OTG_GRXSTSP_PKTSTS) >> USB_OTG_GRXSTSP_PKTSTS_Pos)
        {
        // setup packet
        case STS_SETUP_UPDT:
        {
            if (USBD_P_EP[num])
            {
                USBD_P_EP[num](USBD_EVT_SETUP);
            }
            break;
        }

        // OUT packet
        case STS_DATA_UPDT:
        {
            OTG->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;
            if (USBD_P_EP[num])
            {
                USBD_P_EP[num](USBD_EVT_OUT);
            }
            break;
        }
        default:
        {
            OTG->GRXSTSP;
            break;
        }
        } // switch
    }

    // OUT Packet
    if (istr & USB_OTG_GINTSTS_OEPINT)
    {
        msk = ((USBx_DEVICE->DAINT & USBx_DEVICE->DAINTMSK & USB_OTG_DAINTMSK_OEPM) >> USB_OTG_DAINTMSK_OEPM_Pos);
        i = 0;
        while (msk)
        {
            num = 0;
            for (; i < (USBD_EP_NUM + 1); i++)
            {
                if ((msk >> i) & 1)
                {
                    num = i;
                    msk &= ~(1 << i);
                    break;
                }
            }

            // Endpoint disabled
            if (USBx_OUTEP(num)->DOEPINT & USB_OTG_DOEPINT_EPDISD)
            {
                USBx_OUTEP(num)->DOEPINT |= USB_OTG_DOEPINT_EPDISD;
            }

            // Transfer complete interrupt
            if ((USBx_OUTEP(num)->DOEPINT & USB_OTG_DOEPINT_XFRC) | (USBx_OUTEP(num)->DOEPINT & USB_OTG_DOEPINT_STUP))
            {
                USBx_OUTEP(num)->DOEPTSIZ =
                    (OutPacketCnt[num] << USB_OTG_DOEPTSIZ_PKTCNT_Pos) | /* packet count */
                    (OutMaxPacketSize[num]);    /* transfer size */
                if (num == 0)
                {
                    USBx_OUTEP(0)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_STUPCNT_0;
                }
                USBx_OUTEP(num)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
                USBx_OUTEP(num)->DOEPINT |= USB_OTG_DOEPINT_XFRC;
            }
        }
    }

    // IN Packet
    if (istr & USB_OTG_GINTSTS_IEPINT)
    {
        msk = (USBx_DEVICE->DAINT & USBx_DEVICE->DAINTMSK & USB_OTG_DAINTMSK_IEPM);
        i = 0;
        while (msk)
        {
            num = 0;
            for (; i < (USBD_EP_NUM + 1); i++)
            {
                if ((msk >> i) & 1)
                {
                    num = i;
                    msk &= ~(1 << i);
                    break;
                }
            }

            // Endpoint disabled
            if (USBx_INEP(num)->DIEPINT & USB_OTG_DIEPINT_EPDISD)
            {
                USBx_INEP(num)->DIEPINT = USB_OTG_DIEPINT_EPDISD;
            }

            // IN endpoint NAK effective
            if (USBx_INEP(num)->DIEPINT & USB_OTG_DIEPINT_INEPNE)
            {
                if (InPacketDataPtr[num] &&
                    (InPacketDataReady & (1 << num)))
                {
                    SyncWriteEP = 1;
                    USBD_WriteEP(num, (uint8_t *)InPacketDataPtr[num],
                                 InPacketDataCnt[num]);
                    SyncWriteEP = 0;
                    if (!InPacketDataReady)
                    {
                        // No more pending IN transfers and disable IN NAK interrupts
                        USBx_DEVICE->DIEPMSK &= ~USB_OTG_DIEPMSK_INEPNEM;
                    }
                    continue;
                }
                else
                {
                    USBx_INEP(num)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK;
                }
                USBx_INEP(num)->DIEPINT = USB_OTG_DIEPINT_INEPNE;
            }

            // Transmit completed
            if (USBx_INEP(num)->DIEPINT & USB_OTG_DIEPINT_XFRC)
            {
                USBx_INEP(num)->DIEPINT = USB_OTG_DIEPINT_XFRC;
                SyncWriteEP = 1;
                if (USBD_P_EP[num])
                {
                    USBD_P_EP[num](USBD_EVT_IN);
                }
                SyncWriteEP = 0;
            }
        }
    }

    NVIC_EnableIRQ(OTG_HS_IRQn);
}
