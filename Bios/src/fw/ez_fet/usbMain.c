/*
 * usbMain.c
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

#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/types.h"          // Basic Type declarations
#include "F5xx_F6xx_Core_Lib/HAL_UCS.h"
#include "F5xx_F6xx_Core_Lib/HAL_PMM.h"
#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/usb.h"        // USB-specific functions
#include "USB_API/USB_CDC_API/UsbCdc.h"

#include "usbConstructs.h"
#include "bios.h"
// Global flags set by events
volatile BYTE bCDCDataReceived_event = FALSE;          // Flag set by event handler to indicate data has been received into USB buffer

volatile BYTE bNoReceiveOperationOpen = TRUE;

volatile BYTE bDataReceiveCompleted_event = FALSE;

volatile WORD USBEVIE = kUSB_VbusOnEvent+kUSB_VbusOffEvent+kUSB_dataReceivedEvent+kUSB_UsbSuspendEvent+kUSB_UsbResumeEvent+kUSB_UsbResetEvent;
volatile BYTE bCDC_Data_Received_event = FALSE;

BYTE AppRxBuffer[BIOS_RX_SIZE];
BYTE AppRxBuffer2[BIOS_RX_SIZE];

#define MAX_PKT_SIZE 64

static USHORT   iBytesReceived = 0;
static USHORT   UartEjectCounter = 0;
static USHORT   UartGet = 1;
static USHORT   UartPut = 0;

static unsigned short HandshakeOn = 0;

static unsigned short UART_Rx_Marker = 0;
static unsigned int *Semaphore_Add = (unsigned int*)UART_SEMAPHOREADRESS;
static unsigned short RtsReady = 0;

extern unsigned short comDataAvalible;
extern unsigned short FirstUartData;

void USB_MainLoop(void)
{
    BYTE ReceiveError=0, SendError=0;


    // Check the USB state and loop accordingly
    switch(USB_connectionState())
    {
       case ST_USB_DISCONNECTED:
             __bis_SR_register(LPM3_bits + GIE);    // Enter LPM3 until USB is connected
             __no_operation();
            break;

       case ST_USB_CONNECTED_NO_ENUM:
            break;

       case ST_ENUM_ACTIVE:
            break;

       case ST_ENUM_SUSPENDED:
            __bis_SR_register(LPM3_bits + GIE);     // Enter LPM3 until a resume or VBUS-off event
            break;

       case ST_ENUM_IN_PROGRESS:
            break;

       case ST_NOENUM_SUSPENDED:
            __bis_SR_register(LPM3_bits + GIE);
            _NOP();
            break;

       case ST_ERROR:
            _NOP();
            break;

       default:;
    }
    if(ReceiveError || SendError)
    {
      //TO DO: User can place code here to handle error
    }

    // Start USB to UART bridge communication ----------------------------------
    // send only Bytes when Tx buffer is empty, RTS set and UART not busy

    if(comDataAvalible == TRUE)
    {
       if(FirstUartData)
       {
            if(uartInfos_.uartGetRts())
            {
               //handshake is connected use Handshake in Firmware
               HandshakeOn = 1;
               uartInfos_.uartDisableRtsPullDown();
               RtsReady = 1;
               HandshakeOn = 1;
            }
            else
            {
                // Line is floating -> Handsahke is not connected -> do not use Handshake in Firmware
                HandshakeOn = 0;
                RtsReady = 1;
                uartInfos_.uartEnableRtsPullUp();
            }
            FirstUartData = 0;
        }

        if(uartInfos_.uartNotBusy() && uartInfos_.uartTxEmpty() && uartInfos_.uartGetRts() && RtsReady)
        {
            bCDC_Data_Received_event = FALSE;           // reset event
            BYTE BytesInUsbBuffer = USBCDC_bytesInUSBBuffer(1);

            if(BytesInUsbBuffer)
            {
                BYTE uartDataToSend = 0;
                USBCDC_receiveData(&uartDataToSend, 1 , 1);
                uartInfos_.uartTransmit(uartDataToSend);
                if(HandshakeOn == 1)
                {
                    // just check for toggle of RTS line if Handshake is used.
                    RtsReady = 0;
                }
            }
            if(BytesInUsbBuffer == 0)
            {
                comDataAvalible = FALSE;
            }
        }
    }
    // END USB to UART bridge communication ------------------------------------

    // Start clear UART Tx buffer
    if(UART_Rx_Marker)
    {
        uartInfos_.uartReceive(UartPut);
        iBytesReceived++;
        UartEjectCounter=0;
        UART_Rx_Marker = FALSE;
        UCA0IE |= UCRXIE;
    }
    // End clear UART Tx buffer

    // Start UART to USB bridge communication ----------------------------------
    // when UART not busy and Rx buffer is empty

    if(uartInfos_.uartNotBusy() && uartInfos_.uartRxEmpty() && iBytesReceived < MAX_PKT_SIZE)
    {
        *Semaphore_Add = uartInfos_.uartSetCts();
    }
    // End UART to USB bridge communication ------------------------------------

    // Start USB to Host communication (Uart)-----------------------------------
    if(iBytesReceived > 0)
    {
        UartEjectCounter++;
        if(iBytesReceived == MAX_PKT_SIZE || UartEjectCounter >= 64)
        {
            *Semaphore_Add = uartInfos_.uartClearCts();
            if(USBCDC_sendData(uartInfos_.uartReceive(UartGet), iBytesReceived, 1) == kUSBCDC_sendStarted)
            {
              iBytesReceived = UartEjectCounter = 0;
              *Semaphore_Add = uartInfos_.uartSetCts();
            }
        }
    }
    // End USB to Host communication (Uart)-------------------------------------
}

// Interrupt service routines for UART handling
#pragma vector=USCI_A0_VECTOR
__interrupt void UART_RxIsr(void)
{
    *Semaphore_Add = uartInfos_.uartClearCts();
    UART_Rx_Marker = TRUE;
    UCA0IE &=~ UCRXIE;
}

#pragma vector=PORT2_VECTOR
__interrupt void UART_RtsIsr(void)
{
    if((P2IFG & BIT6) == BIT6)
    {
        RtsReady = 1;
        uart_RtsIfgClear();
    }
}


