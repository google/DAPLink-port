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

#define RX_FIFO_SIZE    1024
#define TX0_FIFO_SIZE   64
#define TX1_FIFO_SIZE   512
#define TX2_FIFO_SIZE   512
#define TX3_FIFO_SIZE   512
#define TX4_FIFO_SIZE   512

#define TX_FIFO(n)      *((__packed volatile uint32_t *)(USB1_OTG_HS + USB_OTG_FIFO_BASE + (n)*USB_OTG_FIFO_SIZE))
#define RX_FIFO         *((__packed volatile uint32_t *)(USB1_OTG_HS + USB_OTG_FIFO_BASE))

// 00: Control, 01: Isochronous, 10: Bulk, 11: Interrupt
#define EP_IN_TYPE(num)     ((USBx_INEP(num)->DIEPCTL >> USB_OTG_DIEPCTL_EPTYP_Pos) & 3)
#define EP_OUT_TYPE(num)    ((USBx_OUTEP(num)->DOEPCTL >> USB_OTG_DIEPCTL_EPTYP_Pos) & 3)

uint32_t OutMaxPacketSize[5] = {USBD_MAX_PACKET0, 0, 0, 0, 0};
uint8_t OutPacketCnt[5] = {1, 0, 0, 0, 0};
uint8_t InPacketCnt[5] = {1, 0, 0, 0, 0};

#if (USBD_HID_ENABLE == 1)
uint32_t HID_IntInPacketData[(USBD_HID_MAX_PACKET + 3) / 4];
#endif

#if (USBD_CDC_ACM_ENABLE == 1)
uint32_t CDC_ACM_IntInPacketData[(USBD_CDC_ACM_MAX_PACKET + 3) / 4];
#endif

uint32_t *InPacketDataPtr[5] =
{
/* endpoint 0 */
    0,
/* endpoint 1 */
#if ((USBD_HID_ENABLE == 1) && (USBD_HID_EP_INTIN == 1))
    HID_IntInPacketData,
#elif ((USBD_CDC_ACM_ENABLE == 1) && (USBD_CDC_ACM_EP_INTIN == 1))
    CDC_ACM_IntInPacketData,
#else
    0,
#endif
/* endpoint 2 */
#if ((USBD_HID_ENABLE == 1) && (USBD_HID_EP_INTIN == 2))
    HID_IntInPacketData,
#elif ((USBD_CDC_ACM_ENABLE == 1) && (USBD_CDC_ACM_EP_INTIN == 2))
    CDC_ACM_IntInPacketData,
#else
    0,
#endif
/* endpoint 3 */
#if ((USBD_HID_ENABLE == 1) && (USBD_HID_EP_INTIN == 3))
    HID_IntInPacketData,
#elif ((USBD_CDC_ACM_ENABLE == 1) && (USBD_CDC_ACM_EP_INTIN == 3))
    CDC_ACM_IntInPacketData,
#else
    0,
#endif
/* endpoint 4 */
#if ((USBD_HID_ENABLE == 1) && (USBD_HID_EP_INTIN == 4))
    HID_IntInPacketData,
#elif ((USBD_CDC_ACM_ENABLE == 1) && (USBD_CDC_ACM_EP_INTIN == 4))
    CDC_ACM_IntInPacketData,
#else
    0,
#endif
};

#if (USBD_ADC_ENABLE == 1)
uint32_t ADC_IsoOutPacketData[(USBD_ADC_WMAXPACKETSIZE + 3) / 4];
uint32_t *IsoOutPacketDataPtr[5] =
{
/* endpoint 0 */
    0,
/* endpoint 1 */
#if (USBD_ADC_EP_ISOOUT == 1)
    ADC_IsoOutPacketData,
#else
    0,
#endif
/* endpoint 2 */
#if (USBD_ADC_EP_ISOOUT == 2)
    ADC_IsoOutPacketData,
#else
    0,
#endif
/* endpoint 3 */
#if (USBD_ADC_EP_ISOOUT == 3)
    ADC_IsoOutPacketData,
#else
    0,
#endif
/* endpoint 4 */
#if (USBD_ADC_EP_ISOOUT == 4)
    ADC_IsoOutPacketData,
#else
    0,
#endif
};
#else
uint32_t *IsoOutPacketDataPtr[5] = {0};
#endif // USBD_ADC_ENABLE

uint32_t InPacketDataCnt[5] = {0};
uint32_t InPacketDataReady = 0;
uint32_t SyncWriteEP = 0;
uint32_t IsoOutPacketDataCnt[5] = {0};
uint32_t IsoOutTokenRead = 0;

/*
 *  usbd_stm32_delay
 *    Parameters:      delay:      Delay in 100 us ticks
 *    Return Value:    None
 */
void usbd_stm32_delay(uint32_t delay)
{
    delay *= SystemCoreClock / 100000;
    while (delay--)
    {
        __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    }
}

/*
 *  USB Device Interrupt enable
 *   Called by USBD_Init to enable the USB Interrupt
 *    Return Value:    None
 */

#ifdef __RTX
void __svc(1) USBD_IntrEna(void);
void __SVC_1(void)
{
#else
void USBD_IntrEna(void)
{
#endif
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

#ifndef __OTG_HS_EMBEDDED_PHY
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN |
                    RCC_AHB4ENR_GPIOCEN | RCC_AHB4ENR_GPIOHEN |
                    RCC_AHB4ENR_GPIOIEN;

    // ULPI data pins
    // PA3 (OTG_HS_ULPI alternate function, DATA0)
    GPIOA->MODER   =  (GPIOA->MODER  & ~(3 << 6)) | (2 << 6);
    GPIOA->OTYPER  &= ~(1 << 3);
    GPIOA->OSPEEDR  |= (3 << 6);
    GPIOA->PUPDR   &= ~(3 << 6);
    GPIOA->AFR[0]  = (GPIOA->AFR[0] & ~(15 << 12)) | (10 << 12); /* Alt Func 10 for ULPI */

    // PB0, PB1 (OTG_HS_ULPI alternate function, DATA1, DATA2)
    GPIOB->MODER = (GPIOB->MODER & ~(15 << 0)) | (10 << 0);
    GPIOB->OTYPER &= ~(3 << 0);
    GPIOB->OSPEEDR |= (15 << 0);
    GPIOB->PUPDR &= ~(15 << 0);
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0xFF << 0)) | (0xAA << 0);

    // PB10..13 (OTG_HS_ULPI alternate function, DATA3 to DATA6)
    GPIOB->MODER = (GPIOB->MODER & ~(0xFF << 20)) | (0xAA << 20);
    GPIOB->OTYPER &= ~(15 << 10);
    GPIOB->OSPEEDR |= (0xFF << 20);
    GPIOB->PUPDR &= ~(0xFF << 20);
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0xFFFF << 8)) | (0xAAAA << 8);

    // PB5 (OTG_HS_ULPI alternate function, DATA7)
    GPIOB->MODER = (GPIOB->MODER & ~(3 << 10)) | (2 << 10);
    GPIOB->OTYPER &= ~(1 << 5);
    GPIOB->OSPEEDR |= (3 << 10);
    GPIOB->PUPDR &= ~(3 << 10);
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(15 << 20)) | (10 << 20);

    // ULPI control pins
    // PC0 (OTG_HS_ULPI alternate function, STP)
    GPIOC->MODER = (GPIOC->MODER & ~(3 << 0)) | (2 << 0);
    GPIOC->OSPEEDR |= (3 << 0);
    GPIOC->AFR[0] = (GPIOC->AFR[0] & ~(15 << 0)) | (10 << 0);

    // PI11 (OTG_HS_ULPI alternate functon, DIR)
    GPIOI->MODER = (GPIOI->MODER & ~(3 << 22)) | (2 << 22);
    GPIOI->OSPEEDR |= (3 << 22);
    GPIOI->AFR[1] = (GPIOI->AFR[1] & ~(15 << 12)) | (10 << 12);

    // PH4 (OTG_HS_ULPI alternate function, NXT)
    GPIOH->MODER = (GPIOH->MODER & ~(3 << 8)) | (2 << 8);
    GPIOH->OSPEEDR |= (3 << 8);
    GPIOH->AFR[0] = (GPIOH->AFR[0] & ~(15 << 16)) | (10 << 16);

    // PA5 (OTG_HS_ULPI alternate function, CLOCK)
    GPIOA->MODER = (GPIOA->MODER & ~(3 << 10)) | (2 << 10);
    GPIOA->OSPEEDR |= (3 << 10);
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(15 << 20)) | (10 << 20);
#else
    RCC->AHB1ENR |= (1 << 1); // Enable clock for Port B

    // Configure PB14 and PB15 as alternate OTG_HS_DM and OTG_HS_DP pins
    GPIOB->MODER = (GPIOB->MODER & ~(0x0FUL << 28)) | (0x0AUL << 28);
    GPIOB->OTYPER &= ~(3 << 14);
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0xFFUL << 24)) |
                    (0xCCUL << 24); // Alt Func 12 for PB14 and PB15
    GPIOB->OSPEEDR |= (15UL << 28);
    GPIOB->PUPDR &= ~(15UL << 28);
#endif

    RCC->AHB1ENR |= RCC_AHB1ENR_USB1OTGHSEN;  // Enable clock for OTG HS
    usbd_stm32_delay(100);      // Wait ~10 ms
    RCC->AHB1RSTR |= RCC_AHB1RSTR_USB1OTGHSRST; // Reset OTG HS clock
    usbd_stm32_delay(100);      // Wait ~10 ms
    RCC->AHB1RSTR &= ~RCC_AHB1RSTR_USB1OTGHSRST;
    usbd_stm32_delay(400); // Wait ~40 ms
#ifndef __OTG_HS_EMBEDDED_PHY
    RCC->AHB1ENR |= RCC_AHB1ENR_USB1OTGHSULPIEN; // Enable clock for OTG HS ULPI
#endif

#ifdef __OTG_HS_EMBEDDED_PHY
    OTG->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;
#endif
    usbd_stm32_delay(200); // Wait ~20 ms
    tout = 1000;           // Wait max 1 s for AHBIDL = 0
    while (!(OTG->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL))
    {
        if (!tout)
        {
            break;
        }
        tout--;
        usbd_stm32_delay(10); // Wait 1 ms
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
        usbd_stm32_delay(10); // Wait 1 ms
    }
    usbd_stm32_delay(30); // Wait 3 ms

    OTG->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT; // Disable interrupts
    // No VBUS sensing
    // ToDo(elee): do we want this disabled?
    OTG->GCCFG &= ~USB_OTG_GCCFG_VBDEN;
    USBx_DEVICE->DCTL |= USB_OTG_DCTL_SDIS; // soft disconnect enabled

    // Force device mode
    OTG->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
    usbd_stm32_delay(1000); // Wait min 25 ms, we wait ~100 ms

#ifndef __OTG_HS_EMBEDDED_PHY
    USBx_DEVICE->DCFG &= ~USB_OTG_DCFG_DSPD; // High speed phy
#else
    OTG->DCFG |= USB_OTG_DCFG_DSPD; // Full speed phy
#endif

    OTG->GINTMSK =
        USB_OTG_GINTMSK_USBSUSPM |  // suspend int unmask
        USB_OTG_GINTMSK_USBRST |    // reset int unmask
        USB_OTG_GINTMSK_ENUMDNEM |  // enumeration done int unmask
        USB_OTG_GINTMSK_RXFLVLM |   // receive fifo non-empty int  unmask
        USB_OTG_GINTMSK_IEPINT |    // IN EP int unmask
        USB_OTG_GINTMSK_OEPINT |    // OUT EP int unmask
        USB_OTG_GINTMSK_WUIM_Msk |  // resume int unmask
#ifdef __RTX /* elee: isn't RTX used for DAPLink?  Curious why this isn't      \
                defined. */
        ((USBD_RTX_DevTask != 0) ? (1 << 3) : 0); // SOF int unmask
#else
        ((USBD_P_SOF_Event != 0) ? (1 << 3) : 0); // SOF int unmask
#endif

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
#ifdef __OTG_HS_EMBEDDED_PHY
        OTG->GCCFG |= USB_OTG_GCCFG_PWRDWN; // power down deactivated
#endif
        USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS; // soft disconnect disabled
    }
    else
    {
        USBx_DEVICE->DCTL |= USB_OTG_DCTL_SDIS; // soft disconnect enabled
#ifdef __OTG_HS_EMBEDDED_PHY
        OTG->GCCFG &= ~USB_OTG_GCCFG_PWRDWN; // power down activated
#endif
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
                        TX2_FIFO_SIZE + TX3_FIFO_SIZE) /
                       4) |
                      ((TX4_FIFO_SIZE / 4) << 16);

    USBx_OUTEP(0)->DOEPTSIZ = (1 << USB_OTG_DOEPTSIZ_STUPCNT_Pos) | // setup count = 1
                              (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos) |  // packet count
                              USBD_MAX_PACKET0;
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
    usbd_stm32_delay(50);                     // Wait ~5 ms
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
 *  USB Device Flush IN Endpoint Transmit Fifo
 *    Parameters:      adr:   USB Device Address
 *    Return Value:    None
 */
static void USBD_FlushInEpFifo(uint32_t EPNum)
{
    uint32_t wcnt;

    EPNum &= ~0x80;
    OTG->GRSTCTL = (OTG->GRSTCTL & ~USB_OTG_GRSTCTL_TXFNUM) |
                   (EPNum << USB_OTG_GRSTCTL_TXFNUM_Pos) |
                   USB_OTG_GRSTCTL_TXFFLSH;
    // wait until fifo is flushed
    wcnt = 10;
    while (OTG->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH)
    {
        if ((wcnt--) == 0)
        {
            break;
        }
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

    num = pEPD->bEndpointAddress & ~(0x80);
    val = pEPD->wMaxPacketSize;
    type = pEPD->bmAttributes & USB_ENDPOINT_TYPE_MASK;

    if (pEPD->bEndpointAddress & USB_ENDPOINT_DIRECTION_MASK)
    {
        if (type == USB_ENDPOINT_TYPE_ISOCHRONOUS)
            InPacketCnt[num] = (val >> 11) & 3;
        else
            InPacketCnt[num] = 1;

        USBx_DEVICE->DAINTMSK |= (1 << num);     // unmask IN EP int
        USBx_INEP(num)->DIEPCTL = (num << 22) |  // fifo number
                                  (type << 18) | // ep type
                                  val & 0x7FF;   // max packet size
        if ((type & 3) > 1)
        {
            // if interrupt or bulk EP
            USBx_INEP(num)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
        }
    }
    else
    {
        OutMaxPacketSize[num] = val & 0x7FF;
        if (type == USB_ENDPOINT_TYPE_ISOCHRONOUS)
            OutPacketCnt[num] = (val >> 11) & 3;
        else
            OutPacketCnt[num] = 1;

        USBx_DEVICE->DAINTMSK |= (1 << (num + 16)); // unmask OUT EP int

        USBx_OUTEP(num)->DOEPCTL = (type << 18) | // EP type
                                   (val & 0x7FF); // max packet size

        USBx_OUTEP(num)->DOEPTSIZ = (OutPacketCnt[num] << 19) | // packet count
                                    (val & 0x7FF); // transfer size
        if ((type & 3) > 1)
        {
            // if int or bulk EP
            USBx_OUTEP(num)->DOEPCTL |= (1 << 28);
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
    if (EPNum & 0x80)
    {
        EPNum &= ~0x80;
        USBx_INEP(EPNum)->DIEPCTL |= USB_OTG_DIEPCTL_USBAEP | // EP active
                                     USB_OTG_DIEPCTL_SNAK;    // set EP NAK
        if (USBx_INEP(EPNum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
        {
            // disable EP
            USBx_INEP(EPNum)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
        }

        InPacketDataReady &= ~(1 << EPNum);
        if (EP_IN_TYPE(EPNum) == USB_ENDPOINT_TYPE_ISOCHRONOUS)
        {
            OTG->GINTMSK |= USB_OTG_GINTMSK_IISOIXFRM;
        }
    }
    else
    {
        if (EP_OUT_TYPE(EPNum) == USB_ENDPOINT_TYPE_ISOCHRONOUS)
        {
            OTG->GINTMSK |= USB_OTG_GINTMSK_EOPFM; // enable end of periodic frame
            USBx_OUTEP(EPNum)->DOEPCTL |= USB_OTG_DOEPCTL_USBAEP; // EP active
        }
        else
        {
            USBx_OUTEP(EPNum)->DOEPCTL |= USB_OTG_DOEPCTL_USBAEP |  // EP active
                                          USB_OTG_DOEPCTL_EPENA |   // enable EP
                                          USB_OTG_DOEPCTL_CNAK;     // clear EP NAK
        }
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
    uint32_t num, wcnt, IsoEpEnCnt;

    IsoEpEnCnt = 0;

    // Disable IN Endpoint
    if (EPNum & 0x80)
    {
        EPNum &= ~0x80;
        for (num = 1; num <= USBD_EP_NUM; num++)
        {
            if (USBx_INEP(num)->DIEPCTL & USB_OTG_DIEPCTL_USBAEP)
            {
                if (EP_IN_TYPE(num) == USB_ENDPOINT_TYPE_ISOCHRONOUS)
                {
                    IsoEpEnCnt++;
                }
            }
        }

        InPacketDataReady &= ~(1 << EPNum);
        if (IsoEpEnCnt == 1)
        {
            // if all iso endpoints disabled
            OTG->GINTMSK &= ~USB_OTG_GINTMSK_IISOIXFRM;
        }
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
        for (num = 1; num <= USBD_EP_NUM; num++)
        {
            if (USBx_OUTEP(num)->DOEPCTL & USB_OTG_DOEPCTL_USBAEP)
            {
                if (EP_OUT_TYPE(num) == USB_ENDPOINT_TYPE_ISOCHRONOUS)
                {
                    IsoEpEnCnt++;
                }
            }
        }
        if (IsoEpEnCnt == 1)
        {
            // if all iso endpoints disabled, disable EOPF
            OTG->GINTMSK &= ~USB_OTG_GINTMSK_EOPFM;
        }

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
    if (EPNum & 0x80)
    {
        EPNum &= ~0x80;
        InPacketDataReady &= ~(1 << EPNum);
        if (USBx_INEP(EPNum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
        {
            // disable EP
            USBx_INEP(EPNum)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
        }
        // set EP NAK
        USBx_INEP(EPNum)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;

        // Flush endpoint fifo
        USBD_FlushInEpFifo(EPNum | 0x80);

        // If endpoint is isochronous, set proper EVEN/ODD frame and enable Ep
        if (EP_IN_TYPE(EPNum) == USB_ENDPOINT_TYPE_ISOCHRONOUS)
        {
            if (USBD_GetFrame() & 1)
            {
                // set even frame
                USBx_INEP(EPNum)->DIEPCTL |= (1 << 28);
            }
            else
            {
                // set odd frame
                USBx_INEP(EPNum)->DIEPCTL |= (1 << 29);
            }
            // enable EP
            USBx_INEP(EPNum)->DIEPCTL |=
                USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
        }
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

    if (!(EPNum & 0x80))
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
        EPNum &= ~0x80;
        if (USBx_INEP(EPNum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
        {
            // disable endpoint
            USBx_INEP(EPNum)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
        }
        // set stall
        USBx_INEP(EPNum)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;

        USBD_FlushInEpFifo(EPNum | 0x80); /* Flush endpoint fifo */
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
    if (!(EPNum & 0x80))
    {
        // Clear OUT endpoint Stall
        if (((USBx_OUTEP(EPNum)->DOEPCTL >> USB_OTG_DOEPCTL_EPTYP_Pos) & 3) > 1)
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
        EPNum &= ~0x80;

        if (USBx_INEP(EPNum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
        {
            // disable endpoint
            USBx_INEP(EPNum)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
        }

        // Flush endpoint fifo
        USBD_FlushInEpFifo(EPNum | 0x80);

        if (((USBx_INEP(EPNum)->DIEPCTL >> USB_OTG_DIEPCTL_EPTYP_Pos) & 3) > 1)
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
    if (EPNum & 0x80)
    {
        USBD_FlushInEpFifo(EPNum | USB_OTG_GRSTCTL_RXFFLSH);
    }
    else
    {
        OTG->GRSTCTL |= USB_OTG_GRSTCTL_RXFFLSH;
        __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
        __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
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
    U32 val, sz, isoEpFlag;

    if ((USBx_OUTEP(EPNum)->DOEPCTL & USB_OTG_DOEPCTL_USBAEP) == 0)
    {
        // if Ep not active
        return (0);
    }
    isoEpFlag = (EP_OUT_TYPE(EPNum) == USB_ENDPOINT_TYPE_ISOCHRONOUS);

    // Isochronous Ep: read data from intermediate buffer
    if (isoEpFlag)
    {
        // if Isochronouos endpoint
        if (IsoOutTokenRead == 0)
        {
            // get data size
            sz = IsoOutPacketDataCnt[EPNum];
            for (val = 0; val < sz; val++)
            {
                // copy data from intermediate buffer
                pData[val] = ((uint8_t *)(IsoOutPacketDataPtr[EPNum]))[val];
            }
            IsoOutPacketDataCnt[EPNum] = 0; // reset data count
            return (sz);
        }
    }

    sz = (OTG->GRXSTSP >> 4) & 0x7FF; // get available data size

    /* elee: copy from the f103 code, tbd, still required?
      Commit 5ffac262f,  and 3d1a68b768c57cb403bab9d5598f3e9f1d3506a2 (for the
      core) If smaller data is read here, is the rest of the data pulled in
      later? */
    if (sz > bufsz)
    {
        sz = bufsz;
    }

    // if isochronous endpoint
    if (isoEpFlag)
    {
        val = OutPacketCnt[EPNum] - ((USBx_OUTEP(EPNum)->DOEPTSIZ >> USB_OTG_DOEPTSIZ_PKTCNT_Pos) & 0x3FF);
        IsoOutPacketDataCnt[EPNum] = 0; // data unvalid

        // check is data is valid
        switch ((USBx_OUTEP(EPNum)->DOEPTSIZ >> USB_OTG_DOEPTSIZ_STUPCNT_Pos) & 3)
        {
        // data0
        case 0:
        {
            if (val == 1)
            {
                IsoOutPacketDataCnt[EPNum] = sz;
            }
            break;
        }
        // data1
        case 2:
        {
            if (val == 2)
            {
                IsoOutPacketDataCnt[EPNum] = sz;
            }
            break;
        }
        // data2
        case 1:
        {
            if (val == 3)
            {
                IsoOutPacketDataCnt[EPNum] = sz;
            }
            break;
        }
        }
    }

    // copy data from fifo if Isochronous Ep: data is copied to intermediate buffer
    for (val = 0; val < (uint32_t)((sz + 3) / 4); val++)
    {
        __UNALIGNED_UINT32_WRITE(pData, USBx_DFIFO(0U));
        pData += 4;
    }

    // wait RxFIFO non-empty (OUT transfer completed or Setup trans. completed)
    while ((OTG->GINTSTS & (1 << 4)) == 0)
    {
    }
    OTG->GRXSTSP;               // pop register
    OTG->GINTMSK |= (1 << 4);   // unmask RxFIFO non-empty interrupt

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
    U32 *ptr, val, isoEpFlag;

    EPNum &= ~(0x80);

    if ((USBx_INEP(EPNum)->DIEPCTL & USB_OTG_DIEPCTL_USBAEP) == 0)
    {
        // if Ep not active
        return (0);
    }
    isoEpFlag = (EP_IN_TYPE(EPNum) == USB_ENDPOINT_TYPE_ISOCHRONOUS);

    /* Asynchronous write to intermediate buffer */
    if (!SyncWriteEP && InPacketDataPtr[EPNum])
    {
        if ((!(InPacketDataReady & (1 << EPNum))) || isoEpFlag)
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
            if (!isoEpFlag)
            {
                InPacketDataReady |= 1 << EPNum;
                // Set NAK to enable interrupt on NAK
                USBx_INEP(EPNum)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
                // INEPNEM = 1, IN EP NAK efective msk
                USBx_DEVICE->DIEPMSK |= USB_OTG_DIEPMSK_INEPNEM;
            }
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
            cnt | (InPacketCnt[EPNum] << 19) | (InPacketCnt[EPNum] << 29);

        if (isoEpFlag)
        {
            // if Isochronous Ep: set packet frame
            if (USBD_GetFrame() & 1)
            {
                // even frame
                USBx_INEP(EPNum)->DIEPCTL |= (1 << 28);
            }
            else
            {
                // odd frame
                USBx_INEP(EPNum)->DIEPCTL |= (1 << 29);
            }
        }

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
        if (isoEpFlag == 0)
        {
            InPacketDataReady &= ~(1 << EPNum);
        }
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
    return ((USBx_DEVICE->DSTS >> USB_OTG_DSTS_FNSOF_Pos) & 0x3FFF);
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
    static uint32_t IsoInIncomplete = 0;

    istr = OTG->GINTSTS & OTG->GINTMSK;

    // reset interrupt
    if (istr & USB_OTG_GINTSTS_USBRST)
    {
        USBD_Reset();
        usbd_reset_core();
#ifdef __RTX
        if (USBD_RTX_DevTask)
        {
            isr_evt_set(USBD_EVT_RESET, USBD_RTX_DevTask);
        }
#else
        if (USBD_P_Reset_Event)
        {
            USBD_P_Reset_Event();
        }
#endif
        OTG->GINTSTS = USB_OTG_GINTSTS_USBRST;
    }

    // suspend interrupt
    if (istr & USB_OTG_GINTSTS_USBSUSP)
    {
        USBD_Suspend();
#ifdef __RTX
        if (USBD_RTX_DevTask)
        {
            isr_evt_set(USBD_EVT_SUSPEND, USBD_RTX_DevTask);
        }
#else
        if (USBD_P_Suspend_Event)
        {
            USBD_P_Suspend_Event();
        }
#endif
        OTG->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
    }

    // resume interrupt
    if (istr & USB_OTG_GINTSTS_WKUINT)
    {
        USBD_Resume();
#ifdef __RTX
        if (USBD_RTX_DevTask)
        {
            isr_evt_set(USBD_EVT_RESUME, USBD_RTX_DevTask);
        }
#else
        if (USBD_P_Resume_Event)
        {
            USBD_P_Resume_Event();
        }
#endif
        OTG->GINTSTS = USB_OTG_GINTSTS_WKUINT;
    }

    // speed enumeration completed
    if (istr & USB_OTG_GINTSTS_ENUMDNE)
    {
        if (!((USBx_DEVICE->DSTS >> USB_OTG_DSTS_ENUMSPD_Pos) & 3))
        {
            USBD_HighSpeed = 1;
        }
        USBx_INEP(0)->DIEPCTL &= ~0x3FF;
        USBx_INEP(0)->DIEPCTL |= OutMaxPacketSize[0]; // EP0 max packet
        USBx_DEVICE->DCTL |= USB_OTG_DCTL_CGINAK;     // clear global IN NAK
        USBx_DEVICE->DCTL |= USB_OTG_DCTL_CGONAK;     // clear global OUT NAK
        OTG->GINTSTS |= USB_OTG_GINTSTS_ENUMDNE;
    }

    // Start Of Frame
    if (istr & USB_OTG_GINTSTS_SOF)
    {
#ifdef __RTX
        if (USBD_RTX_DevTask)
        {
            isr_evt_set(USBD_EVT_SOF, USBD_RTX_DevTask);
        }
#else
        if (USBD_P_SOF_Event)
        {
            USBD_P_SOF_Event();
        }
#endif
        OTG->GINTSTS = USB_OTG_GINTSTS_SOF;
    }

    // RxFIFO non-empty
    if (istr & USB_OTG_GINTSTS_RXFLVL)
    {
        val = OTG->GRXSTSR;
        num = val & 0x0F;

        switch ((val >> 17) & 0x0F)
        {
        // setup packet
        case 6:
        {
#ifdef __RTX
            OTG->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;
            if (USBD_RTX_EPTask[num])
            {
                isr_evt_set(USBD_EVT_SETUP, USBD_RTX_EPTask[num]);
            }
#else
            if (USBD_P_EP[num])
            {
                USBD_P_EP[num](USBD_EVT_SETUP);
            }
#endif
            break;
        }

        // OUT packet
        case 2:
        {
            OTG->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;
            if (EP_OUT_TYPE(num) == USB_ENDPOINT_TYPE_ISOCHRONOUS)
            {
                IsoOutTokenRead = 1;
                USBD_ReadEP(num, (uint8_t *)IsoOutPacketDataPtr[num],
                            OutMaxPacketSize[0]);
                IsoOutTokenRead = 0;
            }
            else
            {
#ifdef __RTX
                if (USBD_RTX_EPTask[num])
                {
                    isr_evt_set(USBD_EVT_OUT, USBD_RTX_EPTask[num]);
                }
#else
                if (USBD_P_EP[num])
                {
                    USBD_P_EP[num](USBD_EVT_OUT);
                }
#endif
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
        msk = (((USBx_DEVICE->DAINT & USBx_DEVICE->DAINTMSK) >> 16) & 0xFFFF);
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
                if (EP_OUT_TYPE(num) == USB_ENDPOINT_TYPE_ISOCHRONOUS)
                {
                    USBx_OUTEP(num)->DOEPTSIZ =
                        (OutPacketCnt[num] << 19) | /* packet count */
                        (OutMaxPacketSize[num]);    /* transfer size */

                    if ((USBD_GetFrame() & 1))
                    {
                        USBx_OUTEP(num)->DOEPCTL |= (1 << 28); /* even frame */
                    }
                    else
                    {
                        USBx_OUTEP(num)->DOEPCTL |= (1 << 29); /* odd frame  */
                    }

                    USBx_OUTEP(num)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
                }
                USBx_OUTEP(num)->DOEPINT |= USB_OTG_DOEPINT_EPDISD;
            }

            // Transfer complete interrupt
            if ((USBx_OUTEP(num)->DOEPINT & USB_OTG_DOEPINT_XFRC) | (USBx_OUTEP(num)->DOEPINT & USB_OTG_DOEPINT_STUP))
            {
                if (EP_OUT_TYPE(num) != USB_ENDPOINT_TYPE_ISOCHRONOUS)
                {

                    USBx_OUTEP(num)->DOEPTSIZ =
                        (OutPacketCnt[num] << 19) | /* packet count */
                        (OutMaxPacketSize[num]);    /* transfer size */
                    if (num == 0)
                    {
                        USBx_OUTEP(0)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_STUPCNT_0;
                    }
                    USBx_OUTEP(num)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
                }
                USBx_OUTEP(num)->DOEPINT |= USB_OTG_DOEPINT_XFRC;
            }
        }
    }

    // IN Packet
    if (istr & USB_OTG_GINTSTS_IEPINT)
    {
        msk = (USBx_DEVICE->DAINT & USBx_DEVICE->DAINTMSK & 0xFFFF);
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

                if (EP_IN_TYPE(num) == USB_ENDPOINT_TYPE_ISOCHRONOUS)
                {
                    if ((IsoInIncomplete & (1 << num)) != 0)
                    {
                        USBD_FlushInEpFifo(num | 0x80);
                        SyncWriteEP = 1;
                        USBD_WriteEP(num, (uint8_t *)InPacketDataPtr[num],
                                     InPacketDataCnt[num]);
                        SyncWriteEP = 0;
                        IsoInIncomplete &= ~(1 << num);
                    }
                }
            }

            // IN endpoint NAK effective
            if (USBx_INEP(num)->DIEPINT & USB_OTG_DIEPINT_INEPNE)
            {
                if (EP_IN_TYPE(num) != USB_ENDPOINT_TYPE_ISOCHRONOUS)
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
            }

            // Transmit completed
            if (USBx_INEP(num)->DIEPINT & 1)
            {
                USBx_INEP(num)->DIEPINT = 1;
                SyncWriteEP = 1;
                if (EP_IN_TYPE(num) == USB_ENDPOINT_TYPE_ISOCHRONOUS)
                {
                    USBD_WriteEP(num, (uint8_t *)InPacketDataPtr[num],
                                 InPacketDataCnt[num]);
                }
                else
                {
#ifdef __RTX
                    if (USBD_RTX_EPTask[num])
                    {
                        isr_evt_set(USBD_EVT_IN, USBD_RTX_EPTask[num]);
                    }
#else
                    if (USBD_P_EP[num])
                    {
                        USBD_P_EP[num](USBD_EVT_IN);
                    }
#endif
                }
                SyncWriteEP = 0;
            }
        }
    }

    // End of periodic frame
    if (istr & USB_OTG_GINTSTS_EOPF)
    {
        for (num = 1; num <= USBD_EP_NUM; num++)
        {
            if (EP_OUT_TYPE(num) != USB_ENDPOINT_TYPE_ISOCHRONOUS)
            {
                continue;
            }
            if (((USBx_OUTEP(num)->DOEPCTL >> USB_OTG_DOEPCTL_USBAEP_Pos) & 1) == 0)
            {
                continue;
            }

            // incomplete isochronous out transfer
            if (OTG->GINTSTS & USB_OTG_GINTSTS_PXFR_INCOMPISOOUT)
            {
                if ((USBD_GetFrame() & 1) == ((USBx_OUTEP(num)->DOEPCTL >> 16) & 1))
                {
                    if (USBx_OUTEP(num)->DOEPCTL & USB_OTG_DOEPCTL_EPENA)
                    {
                        InPacketDataCnt[num] = 0;    /* discard data  */
                        USBx_OUTEP(num)->DOEPCTL |= USB_OTG_DOEPCTL_EPDIS; /* disable endpoint */
                    }
                }

            }
            else
            {
                // prepare for next isohronous transfer
                USBx_OUTEP(num)->DOEPTSIZ = (OutPacketCnt[num] << 19) | /* packet count */
                                            (OutMaxPacketSize[num]);    /* transfer size */

                if ((USBD_GetFrame() & 1))
                {
                    // even frame
                    USBx_OUTEP(num)->DOEPCTL |= (1 << 28);
                }
                else
                {
                    // odd frame
                    USBx_OUTEP(num)->DOEPCTL |= (1 << 29);
                }

                USBx_OUTEP(num)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
            }
        }
        OTG->GINTSTS = USB_OTG_GINTSTS_EOPF | USB_OTG_GINTSTS_PXFR_INCOMPISOOUT;
    }

    // incomplete isochronous IN transfer
    if (istr & USB_OTG_GINTSTS_IISOIXFR)
    {
        OTG->GINTSTS = USB_OTG_GINTSTS_IISOIXFR;
        for (num = 1; num < (USBD_EP_NUM + 1); num++)
        {
            if (EP_IN_TYPE(num) != USB_ENDPOINT_TYPE_ISOCHRONOUS)
            {
                continue;
            }

            if (((USBx_INEP(num)->DIEPCTL >> USB_OTG_DIEPCTL_USBAEP_Pos) & 1) == 0)
            {
                continue;
            }

            // if EP en & packet frame is incorect
            if (USBx_INEP(num)->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
            {
                if ((USBD_GetFrame() & 1) ==
                    ((USBx_INEP(num)->DIEPCTL >> USB_OTG_DIEPCTL_EONUM_DPID_Pos) & 1))
                {
                    IsoInIncomplete |= (1 << num);
                    USBx_INEP(num)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
                }
            }
        }
    }

    NVIC_EnableIRQ(OTG_HS_IRQn);
}
