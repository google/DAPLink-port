/**
 * @file power.c
 * @brief
 */
#include "fsl_smc.h"
#include "fsl_rcm.h"
#include "fsl_llwu.h"
#include "fsl_pmc.h"
#include "fsl_port.h"
#include "fsl_clock.h"

#include "main.h"
#include "power.h"
#include "IO_Config.h"
#include "uart.h"
#include "rl_usb.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void power_set_wakeup_config(app_power_mode_t targetMode);
static void power_mode_switch(smc_power_state_t curPowerState, app_power_mode_t targetPowerMode);
static void power_pre_switch_hook(smc_power_state_t originPowerState, app_power_mode_t targetMode);
static void power_post_switch_hook(smc_power_state_t originPowerState, app_power_mode_t targetMode);
static void power_enter_mode(app_power_mode_t targetPowerMode);

extern volatile uint8_t wake_from_reset;
extern volatile uint8_t wake_from_usb;
extern volatile bool usb_pc_connected;
extern main_usb_connect_t usb_state;

/*******************************************************************************
 * Code
 ******************************************************************************/
void LLWU_IRQHandler(void)
{
    /* If wakeup by external pin BTN_NOT_PRESSED. */
    if (LLWU_GetExternalWakeupPinFlag(LLWU, PIN_SW_RESET_LLWU_PIN))
    {
        LLWU_ClearExternalWakeupPinFlag(LLWU, PIN_SW_RESET_LLWU_PIN);
        wake_from_reset = 1;
    }
    /* If wakeup by external pin WAKE_ON_EDGE. */
    if (LLWU_GetExternalWakeupPinFlag(LLWU, PIN_WAKE_ON_EDGE_LLWU_PIN))
    {
        LLWU_ClearExternalWakeupPinFlag(LLWU, PIN_WAKE_ON_EDGE_LLWU_PIN);
        wake_from_usb = 1;
    }
}

void PORTCD_IRQHandler(void)
{
    if ((1U << PIN_SW_RESET_BIT) & PORT_GetPinsInterruptFlags(PIN_SW_RESET_PORT))
    {
        /* Disable interrupt. */
        PORT_SetPinInterruptConfig(PIN_SW_RESET_PORT, PIN_SW_RESET_BIT, kPORT_InterruptOrDMADisabled);
        PORT_ClearPinsInterruptFlags(PIN_SW_RESET_PORT, (1U << PIN_SW_RESET_BIT));
    }
    if ((1U << PIN_WAKE_ON_EDGE_BIT) & PORT_GetPinsInterruptFlags(PIN_WAKE_ON_EDGE_PORT))
    {
        PORT_ClearPinsInterruptFlags(PIN_WAKE_ON_EDGE_PORT, (1U << PIN_WAKE_ON_EDGE_BIT));

        if ((PIN_WAKE_ON_EDGE_PORT->PCR[PIN_WAKE_ON_EDGE_BIT] & PORT_PCR_IRQC_MASK) == PORT_PCR_IRQC(kPORT_InterruptRisingEdge)) {
            /* Reset USB on cable detach (VBUS falling edge) */
            USBD_Reset();
            usbd_reset_core();
            usb_pc_connected = false;
            usb_state = USB_DISCONNECTED;
        }
        else {
            // Cable inserted
            wake_from_usb = 1;
        }
    }
}

void power_init(void)
{
    // Configure pin as GPIO
    PORT_SetPinMux(PIN_WAKE_ON_EDGE_PORT, PIN_WAKE_ON_EDGE_BIT, kPORT_MuxAsGpio);
    
    /* Power related. */
    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    if (kRCM_SourceWakeup & RCM_GetPreviousResetSources(RCM)) /* Wakeup from VLLS. */
    {
        PMC_ClearPeriphIOIsolationFlag(PMC);
    }

    /* Allow writes to USBREGEN */
    SIM->SOPT1CFG |= SIM_SOPT1CFG_URWE_MASK;
    /* Disable USB voltage regulator */
    SIM->SOPT1 &= ~SIM_SOPT1_USBREGEN_MASK;

    /* Enable rising edge interrupt on WAKE_ON_EDGE pin (VBUS falling edge) to detect USB detach */
    PORT_SetPinInterruptConfig(PIN_WAKE_ON_EDGE_PORT, PIN_WAKE_ON_EDGE_BIT, kPORT_InterruptRisingEdge);

    NVIC_EnableIRQ(LLWU_IRQn);
    NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}

void power_enter_VLLS0()
{
    power_enter_mode(kAPP_PowerModeVlls0);
}

void power_enter_VLPS()
{
    power_enter_mode(kAPP_PowerModeVlps);
}

static void power_enter_mode(app_power_mode_t targetPowerMode)
{
    smc_power_state_t curPowerState = (smc_power_state_t) 0;

    curPowerState = SMC_GetPowerModeState(SMC);

    if ((targetPowerMode > kAPP_PowerModeMin) && (targetPowerMode < kAPP_PowerModeMax))
    {
        power_pre_switch_hook(curPowerState, targetPowerMode);
        power_set_wakeup_config(targetPowerMode);
        power_mode_switch(curPowerState, targetPowerMode);
        power_post_switch_hook(curPowerState, targetPowerMode);
    }
}

static void power_set_wakeup_config(app_power_mode_t targetMode)
{
    PORT_SetPinInterruptConfig(PIN_SW_RESET_PORT, PIN_SW_RESET_BIT, PIN_SW_RESET_PORT_WAKEUP_TYPE);
    PORT_SetPinInterruptConfig(PIN_WAKE_ON_EDGE_PORT, PIN_WAKE_ON_EDGE_BIT, PIN_WAKE_ON_EDGE_PORT_WAKEUP_TYPE);

    /* If targetMode is VLLS/LLS, setup LLWU. */
    if ((kAPP_PowerModeWait != targetMode) && (kAPP_PowerModeVlpw != targetMode) &&
        (kAPP_PowerModeVlps != targetMode) && (kAPP_PowerModeStop != targetMode))
    {
        LLWU_SetExternalWakeupPinMode(LLWU, PIN_SW_RESET_LLWU_PIN, PIN_SW_RESET_LLWU_WAKEUP_TYPE);
        LLWU_SetExternalWakeupPinMode(LLWU, PIN_WAKE_ON_EDGE_LLWU_PIN, PIN_WAKE_ON_EDGE_LLWU_WAKEUP_TYPE);
        NVIC_EnableIRQ(LLWU_IRQn);
    }
}

static void power_mode_switch(smc_power_state_t curPowerState, app_power_mode_t targetPowerMode)
{
    smc_power_mode_vlls_config_t vlls_config;
    vlls_config.enablePorDetectInVlls0 = true;

    switch (targetPowerMode)
    {
        case kAPP_PowerModeVlps:
            SMC_PreEnterStopModes();
            SMC_SetPowerModeVlps(SMC);
            SMC_PostExitStopModes();
            break;

        case kAPP_PowerModeVlls0:
            vlls_config.subMode = kSMC_StopSub0;
            SMC_PreEnterStopModes();
            SMC_SetPowerModeVlls(SMC, &vlls_config);
            SMC_PostExitStopModes();
            break;

        default:
            break;
    }
}

static void power_pre_switch_hook(smc_power_state_t originPowerState, app_power_mode_t targetMode)
{
    /* Wait for debug console output finished. */
    while (!(LPUART_STAT_TC_MASK & UART->STAT))
    {
    }
    uart_uninitialize();

    /* Disable pins to lower current leakage */
    PORT_SetPinMux(UART_PORT, PIN_UART_RX_BIT, kPORT_PinDisabledOrAnalog);
    PORT_SetPinMux(UART_PORT, PIN_UART_TX_BIT, kPORT_PinDisabledOrAnalog);
    PORT_SetPinMux(PIN_HID_LED_PORT, PIN_HID_LED_BIT, kPORT_PinDisabledOrAnalog);
    
    /* Disable I/O pin SWCLK */
    PIN_SWCLK_PORT->PCR[PIN_SWCLK_BIT] = 0;
    
    /* Disable I/O pin SWDIO */
    PIN_SWDIO_PORT->PCR[PIN_SWDIO_BIT] = 0;
    
    /* If targetMode is VLLS0, disable I2C pins */
    if (kAPP_PowerModeVlls0 == targetMode)
    {
        /* PORTC1 is configured as I2C1_SCL */
        PORT_SetPinMux(PORTC, 1U, kPORT_PinDisabledOrAnalog);

        /* PORTC2 is configured as I2C1_SDA */
        PORT_SetPinMux(PORTC, 2U, kPORT_PinDisabledOrAnalog);
    }
}

static void power_post_switch_hook(smc_power_state_t originPowerState, app_power_mode_t targetMode)
{
    
    /* Configure I/O pin SWCLK */
    PIN_SWCLK_PORT->PCR[PIN_SWCLK_BIT] = PORT_PCR_MUX(1)  | /* GPIO */
                                         PORT_PCR_PE_MASK;  /* Pull (Down) enable */
    PIN_SWCLK_GPIO->PSOR  = PIN_SWCLK;                      /* High level */
    PIN_SWCLK_GPIO->PDDR |= PIN_SWCLK;                      /* Output */
    /* Configure I/O pin SWDIO */
    PIN_SWDIO_PORT->PCR[PIN_SWDIO_BIT] = PORT_PCR_MUX(1)  |  /* GPIO */
                                         PORT_PCR_PE_MASK |  /* Pull enable */
                                         PORT_PCR_PS_MASK;   /* Pull-up */
    
    /* re-configure pinmux of disabled pins */
    PORT_SetPinMux(UART_PORT, PIN_UART_RX_BIT, (port_mux_t)PIN_UART_RX_MUX_ALT);
    PORT_SetPinMux(UART_PORT, PIN_UART_TX_BIT, (port_mux_t)PIN_UART_TX_MUX_ALT);

    uart_initialize();

    /* Change to rising edge interrupt on WAKE_ON_EDGE pin (VBUS falling edge) to detect USB detach */
    PORT_SetPinInterruptConfig(PIN_WAKE_ON_EDGE_PORT, PIN_WAKE_ON_EDGE_BIT, kPORT_InterruptRisingEdge);
}
