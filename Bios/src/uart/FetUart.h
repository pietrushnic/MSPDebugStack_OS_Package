/*
* FetUart.h
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

/*----- FetUart.h -----*/
#ifndef FetUART_H
#define FetUART_H

#ifdef __MSP430F5528

// UART port direction
#define uart_CtsOut()       {P2DIR |= BIT7;}    // CTS output                   
#define uart_RtsIn()        {P2DIR &=~ BIT6;}   // RTS input                     
     
// UART port
#define uart_SetCtsBit()    {P2OUT |= BIT7;}    // Output                       
#define uart_ClearCtsBit()  {P2OUT &=~ BIT7;}   // Output clear                 
#define uart_RtsRen()       {P2REN |= BIT6;}    // RTS resistor                 
#define uart_CtsRen()       {P2REN |= BIT7;}    // CTS resistor

#define uart_RtsIfgSet()    {P2IFG |= BIT6;}    // RTS Interrupt flag set (required for sending first byte)
#define uart_RtsIfgClear()  {P2IFG &=~ BIT6;}   // RTS Interrupt flag clear
#define uart_RtsIe()        {P2IE  |= BIT6;}    // RTS Interrupt enable
#define uart_RtsIes()       {P2IES &=~ BIT6;}   // RTS Interrupt edge select (is set when a low-high transition)

// UART status bits
#define uart_RtsStatus()    ((P2IN & BIT6)==BIT6)                                                      
           

// UART port select
#define uart_TxdSel()       {P3SEL |= BIT3;}    // assign P3.3 to UCA0TXD          
#define uart_RxdSel()       {P3SEL |= BIT4;}    // assign P3.4 to UCA0RXD 

#endif 

/*
#ifdef __MSP430F6638
// UART port direction
    #define UART_CTSOUT  {P9DIR |= BIT5;}   // CTS output
    #define UART_RTSIN   {P2DIR &=~ BIT1;}  // RTS input

// UART status bits
    #define UART_RTSSTATUS {((P2IN &BIT1)==BIT1);} // Input status

// UART port
    #define UART_CTS     {P9OUT |= BIT5;}   // Output
    #define UART_RTS     {P2IN & BIT1;}     // Input

    #define UART_RTSREN  {P2REN |= BIT1;}   // RTS resistor
    #define UART_CTSREN  {P9REN |= BIT5;}   // CTS resistor  

// UART port select
    #define UART_TXDSEL  {P8SEL |= BIT3;}   // assign P8.3 to UCA0TXD
    #define UART_RXDSEL  {P8SEL |= BIT2;}   // assign P8.2 to UCA0RXD
#endif
*/
  
struct UART_INFOS
{
  short(*uartGetLayerVersion)(void);
  short(*uartConfig)(unsigned long Baudrate, unsigned long MCLK_Frequency);
  short(*uartSetCts)(void);
  short(*uartGetRts)(void);
  short(*uartNotBusy)(void);
  short(*uartTxEmpty)(void);
  short(*uartRxEmpty)(void);
  short(*uartTransmit)(unsigned char uartDataToTxBuffer);
  unsigned char *(*uartReceive)(unsigned short state);
  short(*uartClearCts)(void);
  void (*uartEnableRtsPullDown)(void); 
  void (*uartDisableRtsPullDown)(void); 
  void (*uartEnableRtsPullUp)(void); 
  void (*uartClose)(void); 
};
typedef struct UART_INFOS UART_INFOS_t;

short uart_GetLayerVersion();
short uart_Config(unsigned long Baudrate, unsigned long MCLK_Frequency);
short uart_SetCts();
short uart_GetRts();
short uart_ClearCts();
short uart_NotBusy();
short uart_TxEmpty();
short uart_RxEmpty();
short uart_Transmit(unsigned char uartDataToTxBuffer);
unsigned char *uart_Receive(unsigned short state);
void uartPortsHds(void);
void uart_Init(UART_INFOS_t* uartInfos_Pointer);
void uart_EnableRtsPullDown();
void uart_DisableRtsPullDown();
void uart_EnableRtsPullUp();
void uart_Close();

#endif
