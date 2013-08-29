/*
* FetUart.c
*
* Base class for all memory classes.
*
* Copyright (C) 2007 - 2011 Texas Instruments Incorporated - http://www.ti.com/
*
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the
* distribution.
*
* Neither the name of Texas Instruments Incorporated nor the names of
* its contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*----- FetUart.c -----*/

#include "msp430.h"
#include "FetUart.h"

#define UART_LAYER_VERSION 0x0005
#define UART_SIGNATURE     0xACDCACDCul
#define UART_FIFOSIZE 128

static unsigned char *UartRxFifoCase, *UartRxFifoEnd;
static unsigned char UartRxBuffer[UART_FIFOSIZE]={0};
static unsigned long UartPreviousBaudrate=0;

static unsigned char *UartTxFifoCase, *UartTxFifoEnd;
static unsigned char UartTxBuffer[UART_FIFOSIZE]={0};

__no_init static short uart_Semaphore_ @ "UARTSEMAPHORE";
#pragma required = uart_Semaphore_

const unsigned long uart_Signature_ @ "UARTSIGNATURE" = UART_SIGNATURE;
#pragma required = uart_Signature_

const unsigned int uart_LayerVersion_ @ "UARTLAYERVERSION" = UART_LAYER_VERSION;
#pragma required = uart_LayerVersion_

UART_INFOS_t uartInfos_;
unsigned short uartLayerVersion =0;
unsigned short closed = 0;


// initilaize UART functions
void uart_Init(UART_INFOS_t* uartInfos_Pointer)
{
    // map function pointers in FET UART layer
    // get Infos about UART-module
    uartInfos_.uartGetLayerVersion      = uart_GetLayerVersion;
    uartInfos_.uartConfig               = uart_Config; 
    uartInfos_.uartSetCts               = uart_SetCts;
    uartInfos_.uartGetRts               = uart_GetRts;
    uartInfos_.uartClearCts             = uart_ClearCts;
    uartInfos_.uartNotBusy              = uart_NotBusy;
    uartInfos_.uartTxEmpty              = uart_TxEmpty;
    uartInfos_.uartRxEmpty              = uart_RxEmpty;
    uartInfos_.uartTransmit             = uart_Transmit;
    uartInfos_.uartReceive              = uart_Receive;
    uartInfos_.uartEnableRtsPullDown    = uart_EnableRtsPullDown;
    uartInfos_.uartDisableRtsPullDown   = uart_DisableRtsPullDown;
    uartInfos_.uartEnableRtsPullUp      = uart_EnableRtsPullUp;
    uartInfos_.uartClose                = uart_Close;
       
    // clear UART semaphore
    uart_Semaphore_ = 0;
    
    // Ports configuration
    uartPortsHds();                             
    
    // init Tx FIFO pointer
    UartTxFifoCase = &UartTxBuffer[0];            
    UartTxFifoEnd = &UartTxBuffer[UART_FIFOSIZE];
    
    // init Rx FIFO pointer
    UartRxFifoCase = &UartRxBuffer[0];            
    UartRxFifoEnd = &UartRxBuffer[UART_FIFOSIZE];
    
    // now copy getLayerVersion and retrun it to uper core layer
    *uartInfos_Pointer =  uartInfos_;
}

void uart_Close()
{  
   
    // set RTS as input  
   P2OUT &= ~BIT7;
   P2DIR &= ~BIT7;
   
   // set CTS as input
   P2OUT &= ~BIT6;
   P2DIR &= ~BIT6;   
   P2REN &= ~BIT6;   
   
   uart_RtsIfgClear();
      
   P3SEL &= ~BIT3;   // deassign P3.3 to UCA0TXD          
   P3SEL &= ~BIT4;   // deassign P3.4 to UCA0RXD    
   P3DIR &= ~BIT3;             
   P3DIR &= ~BIT4;   
      
   // I2C   
   //P3SEL &= ~BIT0;   // deassign P3.0 to UCA0TXD          
   //P3SEL &= ~BIT1;   // deassign P3.1 to UCA0RXD 
   
   //P3DIR &= ~BIT0;             
   //P3DIR &= ~BIT1;   
}

// return UART layer version
short uart_GetLayerVersion()
{
    return (UART_LAYER_VERSION);
}

// init function for UART port pins
// reset all portpins 
void uartPortsHds()         // with handshake 
{   
    uart_CtsOut();          // CTS output
    uart_ClearCtsBit();     // clear CTS
    uart_RtsIn();           // RTS input

    uart_RtsRen();          // enable pull up resistor on RTS
    
    uart_RtsIes();          // RTS Interrupt edge select
    uart_RtsIe();           // RTS Interrupt enable
    uart_RtsIfgClear();     // RTS Interrupt flag clear
    uart_RtsIfgSet();       // RTS Interrupt flag set (required for sending first byte)

    uart_TxdSel();               
    uart_RxdSel();
}
 
void uart_EnableRtsPullDown() 
{
    P2REN |=  BIT6;
    P2OUT &= ~BIT6;
    P2DIR &= ~BIT6;
}

void uart_DisableRtsPullDown()
{
    P2REN &= ~BIT6;
    P2OUT &= ~BIT6;
    P2DIR &= ~BIT6;
}

void uart_EnableRtsPullUp()
{
    P2REN |=  BIT6;
    P2OUT |=  BIT6;
    P2DIR &= ~BIT6;
}

// configuration function for UART settings
// UART will be configure automatically about COM channel configuration
short uart_Config(unsigned long UartBaudrate, unsigned long MCLK_Frequency)
{
    uart_ClearCtsBit();  

    if(UartBaudrate == 9620)
    {
        uart_Close();
        closed = 1;
        return (0);
    }

    if(closed)
    {
        // clear UART semaphore
        uart_Semaphore_ = 0;

        // Ports configuration
        uartPortsHds();

        // init Tx FIFO pointer
        UartTxFifoCase = &UartTxBuffer[0];
        UartTxFifoEnd = &UartTxBuffer[UART_FIFOSIZE];

        // init Rx FIFO pointer
        UartRxFifoCase = &UartRxBuffer[0];
        UartRxFifoEnd = &UartRxBuffer[UART_FIFOSIZE];
    }

    if(UartBaudrate != UartPreviousBaudrate)    // baudrate is changed ?
    {
        static double NonIntergerValue = 0;
    
        UartPreviousBaudrate = UartBaudrate;
        NonIntergerValue = (MCLK_Frequency/ (double)UartBaudrate);
        
        // calculate register settings for UART registers for MSP430F5528
#ifdef __MSP430F5528 
        UCA0CTL1 |= UCSWRST;            // Put state maschine reset
        UCA0CTL1 |= UCSSEL__SMCLK;      // clock source: SMCLK  
        
        // different register settings due to UART calculation
        // use UCBRFO and UC0S16 when (SMCLK / baudrate) >= 16
        if(NonIntergerValue >=16)
        {
            UCA0BRW = (unsigned int)(NonIntergerValue/ 16); 
            UCA0MCTL = UCOS16 + (UCBRF0 * 
                                ((unsigned int)(0.5+
                                (((NonIntergerValue/16)-
                                ((unsigned int)(NonIntergerValue/16)))*16))));
        }
        
        // use only UCBRS0 when (SMCLK / baudrate) < 16 
        else
        {
            UCA0BRW = (unsigned int)(NonIntergerValue);
            UCA0MCTL |= (UCBRS0 * 
                        ((unsigned int)(0.5+
                        (((NonIntergerValue)-
                        ((unsigned int)(NonIntergerValue)))*8))));
        }
        UCA0CTL1 &=~ UCSWRST;    // Initialize USCI state machine
        UCA0IE |= UCRXIE;        // enable USCIA0 interrupt
#endif

        // calculate register settings for Uart registers for MSP430F6638
#ifdef __MSP4306638
        UCA1CTL1 |= UCSWRST;         // Put state maschine reset
        UCA1CTL1 |= UCSSEL__SMCLK;   // clock source: SMCLK
        
        // different register settings due to UART calculation
        // use UCBRFO and UC0S16 when (SMCLK / baudrate) >= 16
        if(NonIntergerValue >= 16)
        {
            UCA1BRW = (unsigned int)(NonIntergerValue/ 16); 
            UCA1MCTL = UCOS16 + (UCBRF0 * 
                                ((unsigned int)(0.5+
                                (((NonIntergerValue/16)-
                                ((unsigned int)(NonIntergerValue/16)))*16))));
        }
        
        // use only UCBRS0 when (SMCLK / baudrate) < 16 
        else
        {
            UCA1BRW = (unsigned int)(NonIntergerValue);
            UCA1MCTL |= (UCBRS0 * 
                        ((unsigned int)(0.5+
                        (((NonIntergerValue)-
                        ((unsigned int)(NonIntergerValue)))*8))));
        }
        UCA1CTL1 &=~ UCSWRST;    // Initialize USCI state machine
        UCA1IE |= UCRXIE;        // enable USCIA1 interrupt  
#endif 
    }
    return (0);
}

short uart_SetCts()
{
    uart_SetCtsBit();       
    return (1);
}



short uart_GetRts()
{
    if(uart_RtsStatus())    // get RTS status from Target    
    {
        return (1);
    }
    else
    {
        return (0);
    }
}  

short uart_ClearCts()
{
    uart_ClearCtsBit();     // clear CTS
    return (0);
}

short uart_NotBusy()
{
    if((UCA0STAT & UCBUSY) == UCBUSY)   // when busy:
    {
        return (0);                     // return 0
    }
    return (1);                         // else return 1
}

short uart_TxEmpty()
{
    // UCTXIFG is set when UCAxTXBUF empty
    if(UCTXIFG == (UCTXIFG & UCA0IFG)) 
    {
        return (1);     // empty
    }
    return (0);         // not empty
}

short uart_RxEmpty()
{
    // UCRXIFG is set when  UCAxRXBUF has received a complete character
    if(UCRXIFG == (UCRXIFG & UCA0IFG)) 
    {
        return (0);     // empty
    }
    return (1);         // not empty
}

// functions copies byte by byte from USB buffer to UART buffer
short uart_Transmit(unsigned char uartDataToTxBuffer)
{
    UCA0TXBUF = uartDataToTxBuffer;
    return (0);
}

unsigned char *uart_Receive(unsigned short UartRxFifoState)
{
    // put target datas into UARt Rx buffer
    if((UartRxFifoCase != UartRxFifoEnd) && !UartRxFifoState)
    {
        if(UartRxFifoCase == UartRxFifoEnd)
        {
            uart_ClearCts();
        }
        *UartRxFifoCase++ = UCA0RXBUF;
    }
    // return Uart Rx buffer to USB
    if(UartRxFifoState)
    {
        UartRxFifoCase = &UartRxBuffer[0];
        return(UartRxBuffer);
    }
    return (0);
}
