/*
 * bios.c
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//! \ingroup MODULBIOS
//!  \file bios.c
//!
//!<ul>
//!<li>system init</li>
//!<li>LED handling</li>
//!<li>global timer</li>
//!<li>rx/tx with handshake or DMA</li>
//!</ul>

#include "hw_compiler_specific.h"
#include "bios.h"
#include "../v3_0p.h"
#include "../stream.h"
#include <stdlib.h>
#include <string.h>

#include "USB_API/USB_Common/types.h"
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "USB_config/descriptors.h"
#include "usbConstructs.h"

//! \brief count of LED from uif
#define BIOS_LED_COUNT 2

//! \brief pin (from TUSB) for hardware handshake for TX cannel
//! \details also used as TRUE value for bios_xoff_
#define BIOS_HARD_RTS_BIT 3

#define CORE_VERSION 0x0006// fist core version for ezFET

//! \brief version of bios code
const unsigned short core_version_ @ "COREVERSION" = CORE_VERSION;
#pragma required=core_version_

unsigned short tool_id_;/*= TOOL_ID;*/
#pragma required=tool_id_

//! \brief chars for flow control
const unsigned char BIOS_DLE_CHAR  = 0x10;
//! \brief chars for flow control
const unsigned char BIOS_XON_CHAR  = 0x11;
//! \brief chars for flow control
const unsigned char BIOS_XOFF_CHAR = 0x13;

//**************************** Internal function prototypes

//! \brief clears the TX software buffer
void  bios_UsbTxClear(void);

//! \brief dummy to catch all irq
//! \details biosHalInterfaceClear set all by bios unused irqs to dummy
INTERRUPT_PROTO void dummyIsr (void);

//! \brief receive char form UART
//! \details get chars form hardware RX buffer and stor them in the RX software buffer(s).
//! Messages are checked on crc. and ack/exceptions processed in this function.
void usbRxIsr (unsigned char receivedByte);

//! \brief 10ms timer interrupt
//! \details count all 10ms the timer for RX and TX timeouts.
//! Also the timer for LED drive.
INTERRUPT_PROTO void timerB1Isr (void);

//! \brief function is called by hardware interrupt if TX buffer goes empty
//! \details move a char from the TX software buffer to the hardware TX buffer.
void usbTxIsr (void);

//! \brief Buffer for the RX cannel
//! \details actual two buffers used, one for working, the second to catch char which receivend before
//! the cts signal have an effect.
volatile BiosRxRecord bios_rx_record_ = {0, 0, 0, 0, {0,0, 0,0}, {0,0, 0,0}};
//! \brief Buffer for the TX cannel
//! \details only one buffer in uif. There is no need for a second. The dll play ping pong with the uif
volatile BiosTxRecord bios_tx_record_ = {0, 0, {0}, {0}, {0}, {0}, {0}, NULL, 0};

//! \brief Timer for RX and TX timeouts
//! \details timebase 10ms
volatile BiosGlobalTimer bios_global_timer_[BIOS_TIMER_COUNT];
//! \brief uif is under control of the eg. workbench
//! \details \li 0, not under control, reset by EXCEPTION NULL
//! \li 1, set in v3opRx with the first valid command
char bios_wb_control_ = 0;

//! \brief values for (re)set LEDs
const unsigned char BIOS_LED_OFF   = 0x00;
//! \brief values for (re)set LEDs
const unsigned char BIOS_LED_ON    = 0x01;
//! \brief values for (re)set LEDs
const unsigned char BIOS_LED_BLINK = 0x02;

//! \brief for LED driving
struct _BiosLedTimer_
{
  unsigned char  mode;
  unsigned short load;
  unsigned short counter;
  unsigned char  *addr;
  unsigned char bit;
};
typedef struct _BiosLedTimer_ BiosLedTimer;

//! \brief for LED driving
volatile BiosLedTimer bios_leds_[BIOS_LED_COUNT];

//! \brief the LED from the EASY :-)
//! \details The EASY haven't LEDs, this var is a replacment for the port to catch
//! the LED driving
unsigned char bios_led_dummy_;

//! \brief Hold the crystal frequency
//! \details will set in biosInitSystem depended for the detected hardware
//! will used eg. from timers and UART
unsigned long crystal_;

//! \brief modulation register value
//! \details will set in biosInitSystem depended for the detected hardware
unsigned char baudmod_ = 0;

//! \brief flags for hardware features
//! \details will set in biosInitSystem depended for the detected hardware
//! \li bit 0 -> handshake xon/xoff = 1, hw handshake = 0
//! \li bit 1 -> 4 wire
//! \li bit 2 -> 2 wire
unsigned long bios_device_flags_ = 0;

//! \brief hardware version
//! \details will set in biosInitSystem depended for the detected hardware
unsigned short bios_info_hw_0_;

//! \brief hardware version
//! \details will set in biosInitSystem depended for the detected hardware
unsigned short bios_info_hw_1_;

//! \brief gets the received char
unsigned char bios_rx_char_;

//! \brief dummy to switch of XON/XOFF in easy
const unsigned char bios_rx_char_always_0 = 0;

//! \brief points to bios_rx_char_ if XON/XOFF is active
//! \details on EASY it points to bios_rx_char_always_0 to disable XON/XOFF functionality
//! will set in biosInitSystem depended for the detected hardware
unsigned char *bios_rx_char_ptr_;

//! \brief Flag for XON/XOFF status
//! \li 0, TX can send
//! \li 1, TX must not be send
//! \details between receiving xoff and sending chars is a delay (by program flow). This
//! is not a problem, because the TUSB has enougth reserve in the buffer.
unsigned char bios_xoff_ = 0;

//! \brief pointer to active TX flow control
//! \details \li on EASY it points to the PORT with the handshakesignals
//! \li on uif it points to the (software)flag bios_xoff_
//! will set in biosInitSystem depended for the detected hardware
unsigned char *bios_xoff_ptr_ = &bios_xoff_;

//! \brief Flag for error indication
//! \details \li 1, an error message is to send. Set by every function which can generate an error
//! \li 0, reset in biosmainLoop, if error message was send
volatile char  bios_rx_err_set_ = 0;

//! \brief Message ID from received message which provoke the error.
volatile unsigned char bios_rx_err_id_ = 0;

//! \brief error code
//! \li EXCEPTION_NOT_IMPLEMENT_ERR 	0x8001
//! \li EXCEPTION_MSGID_ERR	        0x8002
//! \li EXCEPTION_CRC_ERR	        0x8003
//! \li EXCEPTION_RX_TIMEOUT_ERR	0x8004
//! \li EXCEPTION_TX_TIMEOUT_ERR	0x8005
//! \li EXCEPTION_RX_OVERFLOW_ERR     	0x8006
//! \li EXCEPTION_TX_NO_BUFFER	        0x8007
//! \li EXCEPTION_COM_RESET	        0x8008
//! \li EXCEPTION_RX_NO_BUFFER	        0x8009
//! \li EXCEPTION_RX_TO_SMALL_BUFFER	0x800A
//! \li EXCEPTION_RX_LENGTH	        0x800B
//! \li HAL specific exceptions	0xFFFF to 0xFF00 (-1 to -255)
volatile unsigned short bios_rx_err_code_;

//! \brief Pointer to additional information
//! \details if an error provide more information about the error
volatile unsigned short *bios_rx_err_payload_ = NULL;

//! \brief target for dummyread form UART
volatile unsigned char usb_read_dummy_;

volatile unsigned short CTS_LINE = 0;

extern unsigned char hilCrcOk();
extern unsigned char halCrcOk();
extern unsigned char dcdcCrcOk();
extern unsigned char comChannelCrcOk();

//**************************** Module Implementation ******************/
void BIOS_InitSystem (void)
{
    bios_device_flags_ = DEVICE_FLAG_SBW4 | DEVICE_FLAG_EZFET;

    //Init Timer A1 (currently used for TX and RX timeouts and LED's)
    //generates interrupt every 10ms
    /*TA1CCR1*/ TA1CCR0 = USB_MCLK_FREQ / (8 * 100) - 1;
    TA1CCTL1 |= CCIE;
    TA1CTL = TASSEL__SMCLK | ID__8 | MC__UP/*MC__CONTINOUS*/ | TACLR | TAIE;

    // LED assignments
    bios_leds_[0].addr = (unsigned char*)&P1OUT;
    bios_leds_[0].bit = BIT2;
    bios_leds_[1].addr = (unsigned char*)&P1OUT;
    bios_leds_[1].bit = BIT3;

    bios_info_hw_0_ = INFO_E2_HW_0;
    bios_info_hw_1_ = INFO_E2_HW_1;

    /// \todo Fix the DLL so that it correctly does not use XON/XOFF when the EZ
    /// is detected!!
    // char set to rx input for xon/xoff
    bios_rx_char_ptr_ = &bios_rx_char_;

    // disalbe BSL protection
    SYSBSLC &= ~SYSBSLPE;
    // get tool id stored in BSL area
    tool_id_ = *(unsigned short*)0x100e;

    // enable BSL protection again
    SYSBSLC |= SYSBSLPE;
}

void BIOS_SuspendSystem(void)
{
     /*if(hal_ptr_ != NULL)
     {
        ((short(*)(void))(*hal_ptr_)[6].function)(); // release JTAG pins
        ((short(*)(void))(*hal_ptr_)[2].function)(); // shutdown target vcc
     }
     dcdcInfos_.dcdcPowerDown();

     _DINT_FET();
     // switch of LEDs
     P1DIR &= ~(BIT2+BIT3);
     P1OUT &= ~(BIT2+BIT3);
     uartInfos_.uartClose();

     // disable Message Timer for Suspend Mode
     TA1CTL = 0;

    __bis_SR_register(LPM4_bits + GIE);*/
}

void BIOS_ResumeSystem(void)
{
    /* P1DIR |= (BIT2+BIT3); // set direction of pins
     _EINT_FET();

     // restart Message Timer
     TA1CTL = TASSEL__SMCLK | ID__8 | MC__UP | TACLR | TAIE;

     BIOS_DcdcInterfaceInit();
     BIOS_HalInterfaceInit();
     BIOS_UartInterfaceInit();
     BIOS_LedOff(BIOS_LED_MODE);
     BIOS_LedOn(BIOS_LED_POWER);*/
}

//*************************************
short BIOS_LedOn(unsigned char no)
{
    if(no < BIOS_LED_COUNT)
    {
        bios_leds_[no].mode = BIOS_LED_ON;
        bios_leds_[no].load = 0;
        bios_leds_[no].counter = 0;
    }
    return(0);
}

//************************************
short BIOS_LedOff(unsigned char no)
{
    if(no < BIOS_LED_COUNT)
    {
        bios_leds_[no].mode = BIOS_LED_OFF;
        bios_leds_[no].load = 0;
        bios_leds_[no].counter = 0;
    }
    return(0);
}

//************************************
short BIOS_LedBlink(unsigned char no,unsigned short time)
{
    if(no < BIOS_LED_COUNT)
    {
        bios_leds_[no].load = time * 2;
        bios_leds_[no].counter = time * 2;
        bios_leds_[no].mode = BIOS_LED_BLINK;
    }
    return(0);
}

//************************************
short BIOS_LedFlash(unsigned char no,unsigned short time)
{
    if((no < BIOS_LED_COUNT) && !bios_leds_[no].counter)
    {
        bios_leds_[no].load = time * 2;
        bios_leds_[no].counter = time * 2;
        bios_leds_[no].mode &= ~BIOS_LED_BLINK;
    }
    return(0);
}

//************************************
short BIOS_LedAlternate(unsigned short time)
{
    bios_leds_[0].load = time * 2;
    bios_leds_[0].counter = time;
    bios_leds_[1].load = time * 2;
    bios_leds_[1].counter = time;
    if(time)
    {
        bios_leds_[0].mode = BIOS_LED_ON | BIOS_LED_BLINK;
        bios_leds_[1].mode = BIOS_LED_OFF | BIOS_LED_BLINK;
    }
    else
    {
        bios_leds_[0].mode = BIOS_LED_OFF;
        bios_leds_[1].mode = BIOS_LED_OFF;
    }
    return(0);
    }

//************************************
void BIOS_GlobalError(void)
{
    BIOS_LedOn(BIOS_LED_MODE);
    BIOS_LedOff(BIOS_LED_POWER);
    while(1);
}

//INTERRUPT(TIMER1_A1_VECTOR) void timerB1Isr (void)



//***********************************************
INTERRUPT(TIMER1_A1_VECTOR) void timerB1Isr (void)
{
    if(TA1IV == 0x02)
    {
        // process global timers
		if((bios_global_timer_[BIOS_TIMER_RX].count > 1) && !(bios_global_timer_[BIOS_TIMER_RX].state & BIOS_TIMER_BREAK))
        {
            bios_global_timer_[BIOS_TIMER_RX].count--;
        }
        else if(bios_global_timer_[BIOS_TIMER_RX].count == 1)
        {
            bios_global_timer_[0].count = 0;
            BIOS_UsbRxError(EXCEPTION_RX_TIMEOUT_ERR);
        }
        if((bios_global_timer_[BIOS_TIMER_TX].count > 1) && !(bios_xoff_))
        {
            bios_global_timer_[BIOS_TIMER_TX].count--;
        }
        // LED mode
        DIAG_SUPPRESS(Pa082)
        if((bios_leds_[BIOS_LED_MODE].mode & BIOS_LED_ON) ^ (bios_leds_[BIOS_LED_MODE].counter > (bios_leds_[BIOS_LED_MODE].load/2)))
        {
            *bios_leds_[BIOS_LED_MODE].addr |= bios_leds_[BIOS_LED_MODE].bit;
        }
        else
        {
            *bios_leds_[BIOS_LED_MODE].addr &= ~bios_leds_[BIOS_LED_MODE].bit;
        }
        DIAG_DEFAULT(Pa082)
        if(bios_leds_[BIOS_LED_MODE].counter > 0)
        {
            bios_leds_[BIOS_LED_MODE].counter--;
        }
        else if(bios_leds_[BIOS_LED_MODE].mode & BIOS_LED_BLINK)
        {
            bios_leds_[BIOS_LED_MODE].counter = bios_leds_[BIOS_LED_MODE].load;
        }

        // LED power
        DIAG_SUPPRESS(Pa082)
        if((bios_leds_[BIOS_LED_POWER].mode & BIOS_LED_ON) ^ (bios_leds_[BIOS_LED_POWER].counter > (bios_leds_[BIOS_LED_POWER].load/2)))
        {
            *bios_leds_[BIOS_LED_POWER].addr |= bios_leds_[BIOS_LED_POWER].bit;
        }
        else
        {
            *bios_leds_[BIOS_LED_POWER].addr &= ~bios_leds_[BIOS_LED_POWER].bit;
        }
        DIAG_DEFAULT(Pa082)
        if(bios_leds_[BIOS_LED_POWER].counter > 0)
        {
            bios_leds_[BIOS_LED_POWER].counter--;
        }
        else if(bios_leds_[BIOS_LED_POWER].mode & BIOS_LED_BLINK)
        {
            bios_leds_[BIOS_LED_POWER].counter = bios_leds_[BIOS_LED_POWER].load;
        }
    }
}

//**********************************************************
INTERRUPT_PROTO void dummyIsr (void)
{
    ;
}

//! \brief Pointer to Bios IRQ vector table
//! \details using of linker file give linker errors
const volatile unsigned short *bios_intvec_ = (unsigned short*)0xFF80;

//! \brief Pointer to HAL IRQ vector table
RO_PLACEMENT_NO_INIT volatile const unsigned short hal_intvec_   @ "HALINTVEC";
RO_PLACEMENT_NO_INIT volatile const unsigned long hal_signature_ @ "HALSIGNATURE";
//! \brief Pointer to irq table in RAM
//! the RAM table is filled form safecore, all irqs are forwarded to the RAM table
// NO_INIT volatile unsigned short irq_forward_[64] @ "IRQ_FORWARD";

RO_PLACEMENT_NO_INIT volatile const unsigned long  dcdc_intvec_   @ "DCDCIVEC";
RO_PLACEMENT_NO_INIT volatile const unsigned long  dcdc_signature_ @ "DCDCSIGNATURE";

RO_PLACEMENT_NO_INIT volatile const unsigned long   hil_signature_ @ "HILSIGNATURE";
RO_PLACEMENT_NO_INIT volatile const unsigned short hil_Start_UP_ @ "HILINIT";

typedef void *(*DcdcInit)(DCDC_INFOS_t* dcdcInfos_Pointer);

DCDC_INFOS_t dcdcInfos_;
#pragma required=dcdcInfos_

short _dummy_getSubMcuVersion(){return 0;};
short _dummy_getLayerVersion(){return 0;};
short _dummy_dcdcCalibrate(unsigned short resistor, unsigned long *ticks, unsigned long *time){return 0;}
short _dummy_dcdcPowerDown(){return 0;}
short _dummy_dcdcSetVcc(unsigned short vcc ){return 0;}
short _dummy_dcdcRestart(unsigned short fetType_){return 0;};

RO_PLACEMENT_NO_INIT volatile const unsigned long  uart_intvec_   @ "UARTINTVEC";
RO_PLACEMENT_NO_INIT volatile const unsigned long  uart_signature_ @ "UARTSIGNATURE";
RO_PLACEMENT_NO_INIT volatile const long  uart_semaphore_ @ "UARTSEMAPHORE";

typedef void *(*UartInit)(UART_INFOS_t* uartInfos_Pointer);

UART_INFOS_t uartInfos_;
#pragma required=uartInfos_

short _dummy_uartGetLayerVersion (void){return 0;};
short  _dummy_uartConfig (unsigned long Baudrate, unsigned long MCLK_Frequency){return 0;};
short _dummy_SetCts (void){return 0;};
short _dummy_GetRts (void){return 0;};
short _dummy_uartNotBusy (void){return 0;};
short _dummy_uartTxEmpty (void){return 0;};
short _dummy_uartRxEmpty (void){return 0;};
short _dummy_uartTransmit(unsigned char to_TxBuffer){return 0;};
unsigned char *_dummy_uartReceive (unsigned short state){return 0;};
short _dummy_ClearCts(void){return 0;};
void _dummy_EnableRtsPullDown(){};
void _dummy_DisableRtsPullDown(){};
void _dummy_EnableRtsPullUp(){};
void _dummy_uartClose(){};

//******************** Module Implementation **************
short BIOS_UartInterfaceInit(void)
{
  // Uart initialisation
    UartInit uart_Init_ = NULL;
    if((uart_intvec_ == 0xFFFF) || (uart_signature_ != 0xACDCACDC)|| ! comChannelCrcOk())
    {
        uartInfos_.uartGetLayerVersion      = _dummy_uartGetLayerVersion;
        uartInfos_.uartConfig               = _dummy_uartConfig;
        uartInfos_.uartSetCts               = _dummy_SetCts;
        uartInfos_.uartGetRts               = _dummy_GetRts;
        uartInfos_.uartNotBusy              = _dummy_uartNotBusy;
        uartInfos_.uartTxEmpty              = _dummy_uartTxEmpty;
        uartInfos_.uartRxEmpty              = _dummy_uartRxEmpty;
        uartInfos_.uartTransmit             = _dummy_uartTransmit;
        uartInfos_.uartReceive              = _dummy_uartReceive;
        uartInfos_.uartClearCts             = _dummy_ClearCts;
        uartInfos_.uartEnableRtsPullDown    = _dummy_EnableRtsPullDown;
        uartInfos_.uartDisableRtsPullDown   = _dummy_DisableRtsPullDown;
        uartInfos_.uartEnableRtsPullUp      = _dummy_EnableRtsPullUp;
        uartInfos_.uartClose                = _dummy_uartClose;
        return -1;
    }
    uart_Init_ = (UartInit)uart_intvec_;
    uart_Init_(&uartInfos_);

    return 0;
}


void BIOS_UartInterfaceClear(void)
{
    uartInfos_.uartGetLayerVersion      = _dummy_uartGetLayerVersion;
    uartInfos_.uartConfig               = _dummy_uartConfig;
    uartInfos_.uartSetCts               = _dummy_SetCts;
    uartInfos_.uartGetRts               = _dummy_GetRts;
    uartInfos_.uartNotBusy              = _dummy_uartNotBusy;
    uartInfos_.uartTxEmpty              = _dummy_uartTxEmpty;
    uartInfos_.uartRxEmpty              = _dummy_uartRxEmpty;
    uartInfos_.uartTransmit             = _dummy_uartTransmit;
    uartInfos_.uartReceive              = _dummy_uartReceive;
    uartInfos_.uartClearCts             = _dummy_ClearCts;
    uartInfos_.uartEnableRtsPullDown    = _dummy_EnableRtsPullDown;
    uartInfos_.uartDisableRtsPullDown   = _dummy_DisableRtsPullDown;
    uartInfos_.uartEnableRtsPullUp      = _dummy_EnableRtsPullUp;
    uartInfos_.uartClose                = _dummy_uartClose;
}

//******************** Module Implementation **************
//#pragma optimize = low
short BIOS_DcdcInterfaceInit(void)
{
    DcdcInit dcdc_Init_ = NULL;
    if(dcdc_intvec_ == 0xFFFF  || dcdc_signature_ != 0xABBAABBA || !dcdcCrcOk())
    {
        dcdcInfos_.getSubMcuVersion = _dummy_getSubMcuVersion;
        dcdcInfos_.getLayerVersion = _dummy_getLayerVersion;
        dcdcInfos_.dcdcCalibrate =  _dummy_dcdcCalibrate;
        dcdcInfos_.dcdcPowerDown = _dummy_dcdcPowerDown;
        dcdcInfos_.dcdcSetVcc = _dummy_dcdcSetVcc;
        dcdcInfos_.dcdcRestart = _dummy_dcdcRestart;
        return -1;
    }
    dcdc_Init_ = (DcdcInit)dcdc_intvec_; // calls the (modified) startup code of HAL
    dcdc_Init_(&dcdcInfos_);

    dcdcInfos_.dcdcRestart(tool_id_);
    return 0;
}

void BIOS_DcdcInterfaceClear(void)
{
    dcdcInfos_.getSubMcuVersion = _dummy_getSubMcuVersion;
    dcdcInfos_.getLayerVersion = _dummy_getLayerVersion;
    dcdcInfos_.dcdcCalibrate =  _dummy_dcdcCalibrate;
    dcdcInfos_.dcdcPowerDown = _dummy_dcdcPowerDown;
    dcdcInfos_.dcdcSetVcc = _dummy_dcdcSetVcc;
    dcdcInfos_.dcdcRestart = _dummy_dcdcRestart;
}

short IccMonitor_Process(unsigned short flags)
{
    if(((P6IN & BIT4) == BIT4) && tool_id_ == eZ_FET_WITH_DCDC)//eZ-FET with DCDC
    {
       //overcurrent detectded -> Sub MCU swith off VCC -> Biso switch on READ LED -> switch of GREEN LED
        BIOS_LedOff(BIOS_LED_POWER);
        BIOS_LedOn(BIOS_LED_MODE);
    }
    else
    {
        // switch on LEDs.
        BIOS_LedOn(BIOS_LED_POWER);
        BIOS_LedOff(BIOS_LED_MODE);
    }
    return 0;
}

extern void _stream_resetSharedVariables();

//**********************************
short BIOS_HalInterfaceInit(void)
{
    HalMainFunc halStartUpCode = NULL;
    unsigned char cmd[6] = {0x05, CMDTYP_EXECUTELOOP, 0, 0, 0, 0};
    unsigned char i;

    BIOS_HalInterfaceClear();

    //set shared mem Variables to 0x00
    _stream_resetSharedVariables();

    // check if rest vector is not FFFF and if a valid Hal/Programm signature was found
    if(hal_intvec_ == 0xFFFF || hal_signature_ != 0xBEEFBEEF || !halCrcOk())
    {
        return -1;
    }
    // if hil is not loaded - did not make sence to init hal
    if(hil_signature_ != 0xF00DF00D || hil_Start_UP_ == 0xFFFF || !hilCrcOk())
    {
          return -1;
    }

    halStartUpCode = (HalMainFunc)hal_intvec_; // calls the (modified) startup code of HAL
    hal_infos_ = halStartUpCode((struct stream_funcs*)&_stream_Funcs, bios_device_flags_, hilCrcOk()); // return HAL sw infos
    hal_ptr_ = (HAL_REC_ARRAY)(*hal_infos_).hal_list_ptr;

   if(hal_ptr_ != NULL)
    {
        //configure ICC monitor process
        (*hal_ptr_)[(*hal_infos_).hal_size-1].id = 0xFFFE;
        (*hal_ptr_)[(*hal_infos_).hal_size-1].function = (void*)IccMonitor_Process;

        for(i=0; i <(*hal_infos_).hal_size; i++)
        {
            if(((*hal_ptr_)[i].id != 0xFFFF) && ((*hal_ptr_)[i].id >= 0xFF00))
            {
                cmd[4] = i;
                cmd[5] = 0;
                V3OP_SetLoop(cmd, 0);
            }
        }
    }
    return 0;
}

//*****************************************
short BIOS_HalInterfaceClear(void)
{
	V3OP_KillAllLoops();

	hal_infos_ = &no_hal_infos_;
    hal_ptr_ = NULL;
    return(0);
}

//******************************************
void BIOS_MainLoop(void)
{
    unsigned char loop_array_counter;
    unsigned char rx_queu_counter = 0;
    unsigned char rx_queu_counter_tmp;
    StreamSafe stream_tmp;
    HalFuncInOut pCallAddr;

    biosResetCts(); // release CTS line
//    while(1)
    {
        // send error messages
        if(bios_rx_err_set_)
        {
            DIAG_SUPPRESS(Pa082)
            V3OP_SendException(bios_rx_err_id_, bios_rx_err_code_, (unsigned short*)bios_rx_err_payload_);
            DIAG_DEFAULT(Pa082)
            bios_rx_err_set_ = 0;
            biosResetCts();
        }
        // search for and execute "commands in loop"

        for(loop_array_counter = 0; loop_array_counter < V3OP_LOOP_ARRAY_COUNT; loop_array_counter++)
        {
            if((v3op_loop_array_[loop_array_counter].active) &&
               (v3op_loop_array_[loop_array_counter].addr <  (*hal_infos_).hal_size))
            {
                pCallAddr = (HalFuncInOut)(*hal_ptr_)[v3op_loop_array_[loop_array_counter].addr].function;
                if(pCallAddr != NULL)
                {
                    if(STREAM_out_init(v3op_loop_array_[loop_array_counter].msg_id, v3op_loop_array_[loop_array_counter].msg_type) >= 0)
                    {
                        STREAM_internal_stream(&v3op_loop_array_[loop_array_counter].indata[MESSAGE_EXECUTE_PAYLOAD_POS], v3op_loop_array_[loop_array_counter].indata[0]-3, (unsigned char*)0x0001, 0, &stream_tmp);
                        if(pCallAddr(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG) == 1)
                        {
                            STREAM_flush();
                            if(v3op_loop_array_[loop_array_counter].flags & V3OP_LOOP_WAIT_FLAG)
                            {
                                V3OP_KillLoop(v3op_loop_array_[loop_array_counter].msg_id);
                            }
                        }
                        STREAM_external_stream(&stream_tmp);
                    }
                }
            }
        }

        // test on new messages from dll and execute
        rx_queu_counter_tmp = rx_queu_counter;
        do
        {
            if(bios_rx_record_.state[rx_queu_counter] & BIOS_RX_RDY)
            {
                BIOS_LedFlash(BIOS_LED_MODE,20);
                STREAM_in_init((BiosRxRecord*)&bios_rx_record_, rx_queu_counter);
                rx_queu_counter_public_ = rx_queu_counter;
                V3OP_Rx(bios_rx_record_.data[rx_queu_counter]);
                rx_queu_counter = rx_queu_counter_public_;
                bios_rx_record_.state[rx_queu_counter] &= ~BIOS_RX_RDY;
                break;
            }
            rx_queu_counter++;
            if(rx_queu_counter >= BIOS_RX_QUEUS)
            {
              rx_queu_counter = 0;
            }
        }
        while(rx_queu_counter_tmp != rx_queu_counter);
    }
}

//! \brief test on overflow and framing error (from TUSB)
/*INLINE(forced)
char biosUsbOverflow(void)
{
    return((U0RCTL & (FE | OE)) > 0);
}*/

//***************************************
unsigned short tempInDataRx[4][BIOS_RX_SIZE]= {0x0,0x0};

//***************************************
void BIOS_InitCom(void)
{
    unsigned char i;

    for(i = 0; i < BIOS_RX_QUEUS; i++)
    {
        bios_rx_record_.datas[i] = tempInDataRx[i];//(unsigned short*)malloc(BIOS_RX_SIZE);
        if(bios_rx_record_.datas[i] == NULL)
        {
            BIOS_GlobalError();
        }
        bios_rx_record_.data[i] = (unsigned char*)bios_rx_record_.datas[i];
    }
    for(i = 0; i < BIOS_TX_QUEUS; i++)
    {
        bios_tx_record_.data[i] = (unsigned char*)bios_tx_record_.datas[i];
    }
}


//*****************************************
void BIOS_PrepareTx(unsigned int size)
{
    bios_tx_record_.send_counter[bios_tx_record_.cannel_to_send] = 0;
    DIAG_SUPPRESS(Pa082)
    bios_tx_record_.count[bios_tx_record_.active] = size;
    bios_tx_record_.state[bios_tx_record_.active] |= BIOS_TX_TO_SEND;
    DIAG_DEFAULT(Pa082)
}

//********************************************
void BIOS_StartTx(void)
{
    bios_global_timer_[BIOS_TIMER_TX].count = BIOS_TX_TIMEOUT;
/*    IFG1 |= UTXIFG0;*/
    // instead of enabling the UART hw TX interrupt for UIFv1...
    // .. we call the F6638-customized usbTxIsr function here direclty
    usbTxIsr();
}

//*********************************************
void BIOS_UsbTxError(void)
{
    bios_UsbTxClear();
}

//*********************************************
void bios_UsbTxClear(void)
{
    unsigned char i = 0;

    memset((unsigned char*)&bios_tx_record_, 0, sizeof(bios_tx_record_));
    bios_global_timer_[BIOS_TIMER_TX].count = 0;
    bios_rx_char_ = 0;
    for(i = 0; i < BIOS_TX_QUEUS; i++)
    {
        bios_tx_record_.data[i] = (unsigned char*)bios_tx_record_.datas[i];
        bios_tx_record_.state[i] = 0;
    }
}

//***************************************
void BIOS_UsbRxClear(void)
{
    unsigned char i;

    bios_rx_record_.active = 0;
    for(i = 0; i < BIOS_RX_QUEUS; i++)
    {
        bios_rx_record_.state[i] = 0;
        bios_rx_record_.last_cmd_typ = 0;
        bios_rx_record_.last_msg_id = 0;
        bios_rx_record_.data[i][0] = 0;
        bios_rx_record_.crc[i] = 0;
        bios_rx_record_.count[i] = 0;
        bios_rx_record_.size[i] = 0;
    }
    bios_global_timer_[0].count = 0;
}

//********************************************
void BIOS_UsbRxError(unsigned short code)
{
    BIOS_UsbRxClear();
    bios_rx_err_code_ = code;
    bios_rx_err_set_ = 1;
    bios_rx_err_payload_ = NULL;
    bios_rx_err_id_ = 0;
}
//************************************************************
void usbTxIsr (void)
{
    unsigned char first_cannel;

    // some thing to send in active send buffer?
    first_cannel = bios_tx_record_.cannel_to_send;
    while(!(bios_tx_record_.state[bios_tx_record_.cannel_to_send] & BIOS_TX_TO_SEND))
    {
        bios_tx_record_.cannel_to_send++;
        if(bios_tx_record_.cannel_to_send >= BIOS_TX_QUEUS)
        {
            bios_tx_record_.cannel_to_send = 0;
        }
        if(bios_tx_record_.cannel_to_send == first_cannel)
        {
            return;
        }
    }
    // test on chars in software TX buffer
    DIAG_SUPPRESS(Pa082)
    if(bios_tx_record_.send_counter[bios_tx_record_.cannel_to_send] < bios_tx_record_.count[bios_tx_record_.cannel_to_send])
    {
        WORD x;

        // send char from software buffer
        if(cdcSendDataInBackground((BYTE*)&bios_tx_record_.data[bios_tx_record_.cannel_to_send][0],bios_tx_record_.count[bios_tx_record_.cannel_to_send],0,20)) //Send the response over USB
        {
            USBCDC_abortSend(&x,0);                                         // It failed for some reason; abort and leave the main loop
        }
        // no char to send ***anymore*** in actual TX software buffer
        // we don't clear TX software buffer, reset only flags. If need we can
        // send the buffer again. (resending in not implemented)
        bios_tx_record_.state[bios_tx_record_.cannel_to_send] &= ~BIOS_TX_TO_SEND;
        bios_tx_record_.send_counter[bios_tx_record_.cannel_to_send] = 0;
        first_cannel = bios_tx_record_.cannel_to_send;
        if(!(bios_tx_record_.state[bios_tx_record_.cannel_to_send] & BIOS_TX_WAIT_ON_ACK))
        {
            bios_global_timer_[BIOS_TIMER_TX].count = 0;
        }
        // search in TX software buffer for data to send. If no data found, it was the
        // last TX buffer empty interrupt.
        do
        {
            bios_tx_record_.cannel_to_send++;
            if(bios_tx_record_.cannel_to_send >= BIOS_TX_QUEUS)
            {
                bios_tx_record_.cannel_to_send = 0;
            }
            if(bios_tx_record_.state[bios_tx_record_.cannel_to_send] & BIOS_TX_TO_SEND)
            {
                if(bios_tx_record_.send_counter[bios_tx_record_.cannel_to_send] <= ((unsigned short)bios_tx_record_.data[bios_tx_record_.cannel_to_send][0])) // add left side +1, if crc active
                {
                    bios_global_timer_[BIOS_TIMER_TX].count = BIOS_TX_TIMEOUT;
                }
            }
        }
        while(!(bios_tx_record_.state[bios_tx_record_.cannel_to_send] & BIOS_TX_TO_SEND) && (bios_tx_record_.cannel_to_send != first_cannel));
    }
    DIAG_DEFAULT(Pa082)
}

static unsigned short * TimerTick = 0;

#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER_B0_ISR_(void)
{
    if(STREAM_getSharedVariable(ID_SHARED_MEMORY_TYPE_TIMER_TICK, &TimerTick))
    {
        (*(unsigned short*)TimerTick)++;
    }
}

extern BYTE USB_disable (VOID);

#pragma vector=SYSNMI_VECTOR
__interrupt void SysNmiHandler(void)
{
    switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG))
    {
    case SYSUNIV_NONE:
        __no_operation();
        break;
    case SYSUNIV_NMIIFG:
        __no_operation();
        break;
    case SYSUNIV_OFIFG:
        UCSCTL7 &= ~(DCOFFG + XT1LFOFFG + XT2OFFG);         //Clear OSC flaut Flags fault flags
        SFRIFG1 &= ~OFIFG;                                  //Clear OFIFG fault flag
        BIOS_LedOn(BIOS_LED_POWER);
        BIOS_LedOn(BIOS_LED_MODE);
        break;
    case SYSUNIV_ACCVIFG:
        __no_operation();
        break;
    case SYSUNIV_BUSIFG:                                      //USB is required.
        SYSBERRIV = 0;                                      //clear bus error flag
        USB_disable();                                      //Disable
        BIOS_LedOn(BIOS_LED_POWER);
        BIOS_LedOn(BIOS_LED_MODE);
        while(1);
        break;
    }

}

static unsigned short * ITick = 0;

#pragma vector=TIMER2_A1_VECTOR
__interrupt void TIMER2_A1_ISR(void)
{
   switch (__even_in_range(TA2IV, TA2IV_TA2IFG))
   {
     case TA2IV_TA2IFG:
            if(STREAM_getSharedVariable(ID_SHARED_MEMORY_TYPE_I_TICK, &ITick))
            {
                ++(*(unsigned short*)ITick);
            }
            break;
     default:
            break;
   }
}

static unsigned short * TimeTick = 0;

#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void)
{
   switch (__even_in_range(TA0IV, TA0IV_TA0IFG))
   {
     case TA0IV_TA0IFG:
            if(STREAM_getSharedVariable(ID_SHARED_MEMORY_TYPE_TIME_TICK, &TimeTick))
            {
                if(*(unsigned short*)TimeTick == 0)
                {
                    ++(*(unsigned short*)TimeTick);
                }
                ++(*(unsigned short*)TimeTick);
            }

            break;
     default:
            break;
   }
}

/****************************************
 * \brief Interrupt function for NMI events
*/
#pragma vector = UNMI_VECTOR
__interrupt VOID UNMI_ISR(VOID)
{
_DINT_FET();
    switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG))
    {
    case SYSUNIV_NONE:
      __no_operation();
      break;
    case SYSUNIV_NMIIFG:
      __no_operation();
      break;
    case SYSUNIV_OFIFG:
        UCSCTL7 &= ~(DCOFFG + XT1LFOFFG + XT2OFFG);         //Clear OSC flaut Flags fault flags
        SFRIFG1 &= ~OFIFG;                                  //Clear OFIFG fault flag
        BIOS_LedOn(BIOS_LED_POWER);
        BIOS_LedOn(BIOS_LED_MODE);
        break;
    case SYSUNIV_ACCVIFG:
      __no_operation();
      break;
    case SYSUNIV_BUSIFG:
      SYSBERRIV = 0;            // clear bus error flag
      USB_disable();            // Disable
      BIOS_LedOn(BIOS_LED_POWER);
      BIOS_LedOn(BIOS_LED_MODE);
      while(1);
      break;
    }

}
