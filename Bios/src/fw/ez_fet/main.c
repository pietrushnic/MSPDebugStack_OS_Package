/*
 * main.c
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
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

#include "USB_config/descriptors.h"

#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/types.h"               //Basic Type declarations
#include "USB_API/USB_Common/usb.h"                 //USB-specific functions

#include "F5xx_F6xx_Core_Lib/HAL_UCS.h"
#include "F5xx_F6xx_Core_Lib/HAL_PMM.h"

#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "usbConstructs.h"
#include "Bios.h"
#include "HAL_FLASH.h"

// Function declarations
extern void USB_MainLoop(void);
extern unsigned char coreCrcOk();
/********************* Module internal function protoypes ********/

/**
 * \brief Initializes the clock system
*/
static void main_InitClock(void);

/**
 * \brief Initializes the ports
*/
static void main_InitPorts(void);

extern volatile WORD USBEVIE;

/********************* Module Implementation *******************/
void main_InitClock(void)
{
     _DINT_FET();
    //Initialization of clock module

     //use REFO for FLL and ACLK
    UCSCTL3 = (UCSCTL3 & ~(SELREF_7)) | (SELREF__REFOCLK);
    UCSCTL4 = (UCSCTL4 & ~(SELA_7)) | (SELA__REFOCLK);

    //MCLK will be driven by the FLL (not by XT2), referenced to the REFO
    Init_FLL_Settle(USB_MCLK_FREQ / 1000, USB_MCLK_FREQ / 32768);   //Start the FLL, at the freq indicated by the config
                                                                    //constant USB_MCLK_FREQ
    XT1_Stop();                 // Stop xt1 to prevent any error
}

/****************************************/
void main_InitPorts(void)
{
	// Ports all back to reset state
	P1DIR = 0;  P2DIR = 0;  P3DIR = 0;    // Reset all ports direction to be inputs
	P4DIR = 0;  P5DIR = 0;  P6DIR = 0;
	//P7DIR = 0; 	P8DIR = 0;
	P1SEL = 0;  P2SEL = 0;  P3SEL = 0;    // Reset all ports alternate function
	P4SEL = 0;  P5SEL = 0;  P6SEL = 0;
	//P7SEL = 0; 	P8SEL = 0;
	P1OUT = 0;  P2OUT = 0;  P3OUT = 0;    // Reset all port output registers
	P4OUT = 0;  P5OUT = 0;  P6OUT = 0;
	//P7OUT = 0;	P8OUT = 0;

    //Port1
    //LED's
    P1DIR = (BIT2+BIT3);

	// Port5
	P5DIR = (BIT0);	// set pins initially to output direction
    P5SEL = (BIT0+BIT6);

	// Port6
	P6DIR = (BIT5);	// set pins initially to output direction
	P6SEL = (BIT0+BIT1+BIT2);

    // set PORT6.4 to input for SUB mcu current reculation
    P6DIR &= ~BIT4;

    P6OUT &= ~(BIT6); // set Reset to 1 -> drive device reset
    __delay_cycles(20000);
    P6OUT = (BIT6);  // release Reset start Sub MCU

    P2REN |= BIT6;   // enable pull down for RTS uart line
}




/****************************************/
void main(void)
{
    WDTCTL = WDTPW + WDTHOLD;	   // Stop watchdog timer
    __delay_cycles(400000);
    if(!coreCrcOk())
    {
        Flash_SegmentErase((unsigned short*)0xFFFE);
        WDTCTL = 0;
    }
    XT2_Stop(); // stop X2 for default state

    // Main module definitions
    main_InitPorts();
    SetVCore(3);
    main_InitClock();
    BIOS_LedOff(BIOS_LED_MODE);
    BIOS_LedOn(BIOS_LED_POWER);

    // Initialize USB and enable various events
    USB_init();
    USB_setEnabledEvents(USBEVIE);

    // See if we're already attached physically to USB, and if so, connect to it
    // Normally applications don't invoke the event handlers, but this is an exception.
    if (USB_connectionInfo() & kUSB_vbusPresent)
    {
        if (USB_enable() == kUSB_succeed)
        {
            USB_reset();
            USB_connect();
        }
        else
        {
            BIOS_LedOff(BIOS_LED_POWER);
            BIOS_LedOn(BIOS_LED_MODE);
            while(1);
        }
    }
    else
    {
        BIOS_LedOff(BIOS_LED_POWER);
        BIOS_LedOn(BIOS_LED_MODE);
        while(1);
    }
    _EINT_FET();

    // init BiosR
    BIOS_InitSystem();
    BIOS_InitCom();
    BIOS_DcdcInterfaceInit();
    BIOS_HalInterfaceInit();
    BIOS_UartInterfaceInit();

    while(1)
    {
        USB_MainLoop();
        BIOS_MainLoop();
    }
}
