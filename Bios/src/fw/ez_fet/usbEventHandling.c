/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*
 * ======== usbEventHandling.c ========
 * Event-handling placeholder functions.
 * All functios are called in interrupt context.
 */

#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/types.h"
#include "USB_API/USB_Common/defMSP430USB.h"
#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/usb.h"
#include "F5xx_F6xx_Core_Lib/HAL_UCS.h"
#include "bios.h"
#include "stream.h"

#ifdef _CDC_
#include "USB_API/USB_CDC_API/UsbCdc.h"
#endif

#ifdef _HID_
#include "USB_API/USB_HID_API/UsbHid.h"
#endif

#ifdef _MSC_
#include "USB_API/USB_MSC_API/UsbMsc.h"
#endif

#ifdef _PHDC_
#include "USB_API/USB_PHDC_API/UsbPHDC.h"
#endif

//These variables are only example, they are not needed for stack
extern volatile BYTE bCDCDataReceived_event;    //data received event

/*
 * If this function gets executed, it's a sign that the output of the USB PLL has failed.
 * returns TRUE to keep CPU awake
 */
BYTE USB_handleClockEvent ()
{
    //TO DO: You can place your code here

    return (TRUE);                              //return TRUE to wake the main loop (in the case the CPU slept before interrupt)
}

/*
 * If this function gets executed, it indicates that a valid voltage has just been applied to the VBUS pin.
 * returns TRUE to keep CPU awake
 */
BYTE USB_handleVbusOnEvent ()
{
    //TO DO: You can place your code here

    //We switch on USB and connect to the BUS
    if (USB_enable() == kUSB_succeed){
        USB_reset();
        USB_connect();                          //generate rising edge on DP -> the host enumerates our device as full speed device
    }
    return (TRUE);                              //return TRUE to wake the main loop (in the case the CPU slept before interrupt)
}

/*
 * If this function gets executed, it indicates that a valid voltage has just been removed from the VBUS pin.
 * returns TRUE to keep CPU awake
 */
BYTE USB_handleVbusOffEvent ()
{
    //TO DO: You can place your code here

    XT2_Stop();

    return (TRUE);                              //return TRUE to wake the main loop (in the case the CPU slept before interrupt)
}

/*
 * If this function gets executed, it indicates that the USB host has issued a USB reset event to the device.
 * returns TRUE to keep CPU awake
 */
BYTE USB_handleResetEvent ()
{
    //TO DO: You can place your code here

    return (TRUE);                              //return TRUE to wake the main loop (in the case the CPU slept before interrupt)
}

/*
 * If this function gets executed, it indicates that the USB host has chosen to suspend this device after a period of active
 * operation.
 * returns TRUE to keep CPU awake
 */

extern void BIOS_SuspendSystem(void);
extern void BIOS_ResumeSystem(void);

BYTE USB_handleSuspendEvent ()
{
    BIOS_SuspendSystem();
    return (TRUE);                              //return TRUE to wake the main loop (in the case the CPU slept before interrupt)
}

/*
 * If this function gets executed, it indicates that the USB host has chosen to resume this device after a period of suspended
 * operation.
 * returns TRUE to keep CPU awake
 */
BYTE USB_handleResumeEvent ()
{
    BIOS_ResumeSystem();
    return (TRUE);                              //return TRUE to wake the main loop (in the case the CPU slept before interrupt)
}

/*
 * If this function gets executed, it indicates that the USB host has enumerated this device :
 * after host assigned the address to the device.
 * returns TRUE to keep CPU awake
 */
BYTE USB_handleEnumCompleteEvent ()
{
    //TO DO: You can place your code here

    return (TRUE);                              //return TRUE to wake the main loop (in the case the CPU slept before interrupt)
}

#ifdef _CDC_

/*
 * This event indicates that data has been received for interface intfNum, but no data receive operation is underway.
 * returns TRUE to keep CPU awake
 */

#define COM_CHANNEL 2
#define DEBUG_CHANNEL 0

extern unsigned char bios_rx_char_;
extern volatile char  bios_rx_err_set_;
extern volatile unsigned char bios_rx_err_id_;
extern volatile unsigned short bios_rx_err_code_;
extern volatile unsigned short *bios_rx_err_payload_;
extern volatile unsigned short CTS_LINE;

extern short V3OP_KillLoop(unsigned char msg_id);
extern void bios_UsbTxClear(void);
unsigned short comDataAvalible = 0;
unsigned short FirstUartData = 1;

BYTE USBCDC_handleDataReceived (BYTE intfNum)
{
    //TO DO: You can place your code here
    if (intfNum == DEBUG_CHANNEL) //this is our debug channel
    {
        //static unsigned short crc_tmp = 0;
        BYTE count = 0;
        BYTE usbDataPointer = 0;

        // block TUSB send
        biosSetCts();

        // bCDCDataReceived_event = TRUE;

        count = USBCDC_bytesInUSBBuffer(DEBUG_CHANNEL);

        if(count)
        {
            static BYTE recivedData[128] = {0};

            //static unsigned char last_rx_char;

            USBCDC_receiveData(recivedData, count , DEBUG_CHANNEL);

            while(count--)
            {
                // get char from hardwarebuffer
                bios_rx_char_ = recivedData[usbDataPointer++];
                // test for xon/xoff flow control char

                // test for free buffer
                if(bios_rx_record_.state[bios_rx_record_.active] & BIOS_RX_RDY)
                {
                    BIOS_UsbRxError(EXCEPTION_RX_NO_BUFFER);
                    goto usbRxIsrExit;
                }
                // test for first char in a message
                if(!bios_rx_record_.count[bios_rx_record_.active])
                {
                    // a message must received in 100ms completely
                    // if not a timeout error generate by timerB1Isr
                    bios_global_timer_[0].count = 50; // start timeout timer 500ms
                    bios_rx_record_.crc[1] = 0;
                    // length of messages must be even
                    if(bios_rx_char_ & 0x01)
                    {
                        bios_rx_record_.size[bios_rx_record_.active] = bios_rx_char_ + 0;
                    }
                    else
                    {
                        bios_rx_record_.size[bios_rx_record_.active] = bios_rx_char_ + 1;
                    }
                }
                // test for buffer size
                if(bios_rx_record_.count[bios_rx_record_.active] < (BIOS_RX_SIZE + 2))
                {
                    DIAG_SUPPRESS(Pa082)
                    // store char in RX software buffer
                    bios_rx_record_.data[bios_rx_record_.active][bios_rx_record_.count[bios_rx_record_.active]++] = bios_rx_char_;
                    DIAG_DEFAULT(Pa082)
                }
                else
                {
                    BIOS_UsbRxError(EXCEPTION_RX_TO_SMALL_BUFFER);
                    goto usbRxIsrExit;
                }

                // test for completly message
                DIAG_SUPPRESS(Pa082)
                if(bios_rx_record_.count[bios_rx_record_.active] > (bios_rx_record_.size[bios_rx_record_.active]))
                DIAG_DEFAULT(Pa082)
                {
                    bios_global_timer_[0].count = 0;  // stop timeout timer

                    if(bios_rx_record_.data[bios_rx_record_.active][MESSAGE_CMDTYP_POS] == RESPTYP_ACKNOWLEDGE)
                    {
                        // search for message which are waiting on a ack
                        for(unsigned char i = 0; i < BIOS_TX_QUEUS; i++)
                        {
                            DIAG_SUPPRESS(Pa082)
                            // waits a transmited message for a ACK?
                            if((bios_tx_record_.state[i] & BIOS_TX_WAIT_ON_ACK) &&
                                ((bios_tx_record_.data[i][MESSAGE_MSG_ID_POS] & 0x3F) == bios_rx_record_.data[bios_rx_record_.active][MESSAGE_MSG_ID_POS]))
                            {
                                // if the transmited message are a member of a multi package message, the ack provide the package number
                                if( bios_tx_record_.data[i][3] || (bios_rx_record_.data[bios_rx_record_.active][0] > 3) )
                                {
                                    // package number are in byte 4
                                    if(bios_tx_record_.data[i][3] == bios_rx_record_.data[bios_rx_record_.active][4])
                            DIAG_DEFAULT(Pa082)
                                    {
                                        bios_tx_record_.state[i] &= ~(BIOS_TX_WAIT_ON_ACK | BIOS_TX_TO_SEND | BIOS_TX_NO_SEND);
                                        bios_global_timer_[BIOS_TIMER_TX].count = 0;
                                        break;
                                    }
                                }
                                else
                                {
                                    bios_tx_record_.state[i] &= ~(BIOS_TX_WAIT_ON_ACK | BIOS_TX_TO_SEND | BIOS_TX_NO_SEND);
                                    break;
                                }
                            }
                        }
                    }
                    else if(bios_rx_record_.data[bios_rx_record_.active][MESSAGE_CMDTYP_POS] ==  RESPTYP_EXCEPTION)
                    {
                        // exception with message ID = 0 always reset the communication
                        // also all user macros in the loop are terminated
                        if(bios_rx_record_.data[bios_rx_record_.active][MESSAGE_MSG_ID_POS] == 0)
                        {
                            BIOS_LedOn(BIOS_LED_POWER);
                            BIOS_LedOff(BIOS_LED_MODE);
                            bios_wb_control_ = 0;
                            V3OP_KillLoop(0);
                            BIOS_UsbRxClear();
                            bios_UsbTxClear();
                        }
                        else
                        {
                            // search for message which are waiting on a ack
                            for(unsigned char i = 0; i < BIOS_TX_QUEUS; i++)
                            {
                                DIAG_SUPPRESS(Pa082)
                                if(((bios_tx_record_.state[i] & (BIOS_TX_WAIT_ON_ACK | BIOS_TX_TO_SEND)) == (BIOS_TX_WAIT_ON_ACK | BIOS_TX_TO_SEND)) &&
                                    (bios_tx_record_.data[i][MESSAGE_MSG_ID_POS] == bios_rx_record_.data[bios_rx_record_.active][MESSAGE_MSG_ID_POS]) &&
                                    (bios_tx_record_.data[i][3] == bios_rx_record_.data[bios_rx_record_.active][4]))
                                DIAG_DEFAULT(Pa082)
                                {
                                    // only reset the wait_on_ack flag,
                                    //! \todo add a resend function
                                    bios_tx_record_.state[i] &= ~BIOS_TX_WAIT_ON_ACK;
                                    break;
                                }
                            }
                        }
                    }
                    else if((bios_rx_record_.data[bios_rx_record_.active][MESSAGE_MSG_ID_POS] == 0) && (bios_rx_record_.data[bios_rx_record_.active][MESSAGE_CMDTYP_POS] ==CMDTYP_COMRESET))
                    {
                        // clear RX cannel
                        BIOS_UsbRxClear();
                        goto usbRxIsrExit;
                    }
                    else
                    {
                        // mark message as ready
                        // BIOS_MainLoop pass the message to v3opRx for macro execution
                        DIAG_SUPPRESS(Pa082)
                        bios_rx_record_.state[bios_rx_record_.active] |= BIOS_RX_RDY;
                        DIAG_DEFAULT(Pa082)
                        // switch to the next Rx buffer
                        bios_rx_record_.active++;
                        if(bios_rx_record_.active >= BIOS_RX_QUEUS)
                        {
                            bios_rx_record_.active = 0;
                        }
                    }

                // no size error
                bios_rx_record_.count[bios_rx_record_.active] = 0;
                bios_rx_record_.crc[1] = 0;
                }
                usbRxIsrExit:
            }
        }
        biosResetCts(); // release TUSB RX
        return (TRUE);
    }

    if(intfNum == 1)
    {
       comDataAvalible = TRUE;
       return (TRUE);
    }
    //return FALSE to go asleep after interrupt (in the case the CPU slept before
    //interrupt)
    return (TRUE);
}

/*
 * This event indicates that a send operation on interface intfNum has just been completed.
 * returns TRUE to keep CPU awake
 */
BYTE USBCDC_handleSendCompleted (BYTE intfNum)
{
    //TO DO: You can place your code here

    return (FALSE);                             //return FALSE to go asleep after interrupt (in the case the CPU slept before
                                                //interrupt)
}

/*
 * This event indicates that a receive operation on interface intfNum has just been completed.
 */
BYTE USBCDC_handleReceiveCompleted (BYTE intfNum)
{
    //TO DO: You can place your code here

    return (FALSE);                             //return FALSE to go asleep after interrupt (in the case the CPU slept before
                                                //interrupt)
}
/*
 * This event indicates that new line coding params have been received from the host
 */
BYTE USBCDC_handleSetLineCoding (BYTE intfNum, ULONG lBaudrate)
{
    //TO DO: You can place your code here
    if(intfNum == COM_CHANNEL)
    {
        FirstUartData = 1;
        uartInfos_.uartConfig(lBaudrate, USB_MCLK_FREQ);

        // Enable pull down for Handshake detection
        uartInfos_.uartEnableRtsPullDown();
        /*P2REN |=  BIT6;
        P2OUT &= ~BIT6;
        P2DIR &= ~BIT6;*/
    }
    return (FALSE);                             //return FALSE to go asleep after interrupt (in the case the CPU slept before
                                                //interrupt)
}

/*
 * This event indicates that new line state has been received from the host
 */
BYTE USBCDC_handleSetControlLineState (BYTE intfNum, BYTE lineState)
{
	return FALSE;
}

#endif //_CDC_

#ifdef _HID_
/*
 * This event indicates that data has been received for interface intfNum, but no data receive operation is underway.
 * returns TRUE to keep CPU awake
 */
BYTE USBHID_handleDataReceived (BYTE intfNum)
{
    //TO DO: You can place your code here

    return (FALSE);                             //return FALSE to go asleep after interrupt (in the case the CPU slept before
                                                //interrupt)
}

/*
 * This event indicates that a send operation on interface intfNum has just been completed.
 * returns TRUE to keep CPU awake
 */
BYTE USBHID_handleSendCompleted (BYTE intfNum)
{
    //TO DO: You can place your code here

    return (FALSE);                             //return FALSE to go asleep after interrupt (in the case the CPU slept before
                                                //interrupt)
}

/*
 * This event indicates that a receive operation on interface intfNum has just been completed.
 */
BYTE USBHID_handleReceiveCompleted (BYTE intfNum)
{
    //TO DO: You can place your code here

    return (FALSE);                             //return FALSE to go asleep after interrupt (in the case the CPU slept before
                                                //interrupt)
}

/*
 * This event indicates that a request to switch to or from Boot protocol
 * was received from the host
 */
BYTE USBHID_handleBootProtocol (BYTE protocol, BYTE intfnum)
{
    return (FALSE);
}

#endif //_HID_

#ifdef _MSC_
BYTE USBMSC_handleBufferEvent (VOID)
{
    return (FALSE);                             //return FALSE to go asleep after interrupt (in the case the CPU slept before
                                                //interrupt)
}

#endif //_MSC_


#ifdef _PHDC_

/*
 * This event indicates that data has been received for interface intfNum, but no data receive operation is underway.
 * returns TRUE to keep CPU awake
 */
BYTE USBPHDC_handleDataReceived (BYTE intfNum)
{
    //TO DO: You can place your code here

    return (TRUE);                              //return FALSE to go asleep after interrupt (in the case the CPU slept before
                                                //interrupt)
}

/*
 * This event indicates that a send operation on interface intfNum has just been completed.
 * returns TRUE to keep CPU awake
 */
BYTE USBPHDC_handleSendCompleted (BYTE intfNum)
{
    //TO DO: You can place your code here

    return (FALSE);                             //return FALSE to go asleep after interrupt (in the case the CPU slept before
                                                //interrupt)
}

/*
 * This event indicates that a receive operation on interface intfNum has just been completed.
 */
BYTE USBPHDC_handleReceiveCompleted (BYTE intfNum)
{
    //TO DO: You can place your code here

    return (FALSE);                             //return FALSE to go asleep after interrupt (in the case the CPU slept before
                                                //interrupt)
}

#endif //_PHDC_

/*----------------------------------------------------------------------------+
 | End of source file                                                          |
 +----------------------------------------------------------------------------*/
/*------------------------ Nothing Below This Line --------------------------*/
