/*
 * FetDcdc.c
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

#include "msp430.h"
#include "FetDcdc.h"
#include "hw_compiler_specific.h"

#define DCDC_LAYER_VERSION 0x0002
#define DCDC_SIGNATURE     0xABBAABBAul

#define COUNT 16
#define DELAYCOUNT 16384
#define ABMOD_LONG0CALIB

const unsigned long dcdc_Signature_ @ "DCDCSIGNATURE" = DCDC_SIGNATURE;
#pragma required = dcdc_Signature_

const unsigned short dcdc_LayerVersion_ @ "DCDCLAYERVERSION" = DCDC_LAYER_VERSION;
#pragma required = dcdc_LayerVersion_

DCDC_INFOS_t dcdcInfos_;
unsigned short dcdcLayerVersion =0;

void dcdc_Init(DCDC_INFOS_t* dcdcInfos_Pointer)
{
    P6DIR &= ~BIT4;

    P6DIR |= BIT5;
    P6OUT &= ~BIT5;

    // map funciotn pointers in FET dcdc layer
    dcdcInfos_.dcdcCalibrate        = dcdc_Calibrate;
    dcdcInfos_.getSubMcuVersion     = dcdc_getSubMcuVersion;
    dcdcInfos_.getLayerVersion      = dcdc_getLayerVersion;
    dcdcInfos_.dcdcPowerDown        = dcdc_PowerDown;
    dcdcInfos_.dcdcSetVcc           = dcdc_SetVcc;
    dcdcInfos_.dcdcRestart          = dcdc_Restart;
    // now copy getLayerVersion and retrun it to uper core layer
    *dcdcInfos_Pointer =  dcdcInfos_;
}

short dcdc_Restart(unsigned short fetType_)
{

    if(fetType_ == eZ_FET_WITH_DCDC) // this is the eZ-FET tool id
    {
         // TEST pin
         P6DIR &= ~BIT7;
         P6OUT &= ~BIT7;
         //togle RST pin to restart sub mcu
         P6DIR |= BIT6;
         __delay_cycles(600000);
         P6OUT &= ~BIT6;
         __delay_cycles(600000);
         P6OUT |= BIT6;
         P6DIR &= ~BIT6;
        // Port3
        //  P3.0 <-> HOST_SDA
        //  P3.1 -> HOST_SCL
        //  P3.2 -> N/C
        //  P3.3 -> UART_RXD N/C
        //  P3.4 <- UART_TXD N/C
        //  P3.5 -> n/c
        //  P3.6 -> n/c
        //  P3.7 -> n/c
        P3SEL = (BIT0+BIT1);
        // Configure I2C
        UCB0CTL1 |= UCSWRST;                      // Enable SW reset
        UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
        UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
        UCB0BR0 = 0xC8;//25;//12;                        // fSCL = SMCLK/25 = ~100kHz
        UCB0BR1 = 0;
        UCB0I2CSA = 0x48;                         // Slave Address is 048h
        UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
         __delay_cycles(600000);
        return eZ_FET_WITH_DCDC;
    }
    else if(fetType_ == eZ_FET_NO_DCDC)
    {
        return eZ_FET_NO_DCDC;
    }
    return -1;
}

//static const unsigned short supportedVoltage[] = {3300, 3600};
//#define ABMOD_LDO
short dcdc_SetVcc(unsigned short vcc)
{
#ifdef ABMOD_LDO
    short success = dcdc_Send(CMD_POWERDOWN, SDIO_POWERDOWN_KEY);
#else
    short success = dcdc_Send(CMD_CONFIGURE, vcc);
#endif
    if(!success)
    {
        return -1;
    }
    return 0;
}


void dcdc_RunCalibrate(unsigned long long *ticks, unsigned long long *time, unsigned short count, unsigned short resistor)
{
    unsigned long long _ticks = 0;
    unsigned long long _time = 0;
    unsigned short i0 = 0, i1 = 0;
    unsigned short t0 = 0, t1 = 0;
    unsigned short i = 0;

    _DINT_FET();
    for(i = 0; i < count; ++i)
    {
        TA0CTL |= TACLR + MC__CONTINOUS + TAIE; // START the timer in free-running mode
        TA2CTL = TASSEL__TACLK + MC_2 + TAIE;   // Timer_A2 source

        i0 = TA2R;
        t0 = TA0R;

        // Sample VCCout
        ADC12CTL0  &= ~ADC12ENC;              // Disable conversion, write controls
        ADC12MCTL0  = ADC12SREF_1 + 1;        // select Vref and analog channel Ax
        ADC12CTL0  |= ADC12ENC;               // Enable conversions
        ADC12CTL0  |= ADC12SC;                // Start conversions
        while ((ADC12IFG & BIT0) == 0);       // wait until conversion is done

        // Wait for some time
        for(volatile unsigned long j = 0; j < DELAYCOUNT; ++j);

        i1 = TA2R;
        t1 = TA0R;

        _time += (t1 - t0);
        _ticks += (i1 - i0);
    }
    *ticks = _ticks;
    *time  = _time;
    _EINT_FET();
}

short dcdc_Calibrate(unsigned short resistor, unsigned long *ticks, unsigned long *time)
{
    unsigned long long _ticks = 0;
    unsigned long long _time = 0;
    short success = 0;
    unsigned int count = COUNT;

    if (resistor == 0)
    {
        count = COUNT*5;
    }

    success = dcdc_Send(CMD_CALLOAD, resistor);

    if(!success)
    {
        return -1;
    }
    // Wait for some time
    for(volatile unsigned long j = 0; j < DELAYCOUNT; ++j);
    for(volatile unsigned long j = 0; j < DELAYCOUNT; ++j);
    for(volatile unsigned long j = 0; j < DELAYCOUNT; ++j);

    dcdc_RunCalibrate(&_ticks, &_time, count, resistor);
    *ticks = _ticks;
    *time  = _time;

    // Reset the calibration resistor
    success = dcdc_Send(CMD_CALLOAD, 0);
    for(volatile unsigned long j = 0; j < DELAYCOUNT; ++j);
    if(!success)
    {
        return -1;
    }
    return 0;
}

short dcdc_getSubMcuVersion()
{
    unsigned int success = 0, version = 0;
    success = dcdc_Receive(&version);
    if (!success)
    {
        return -1;
    }
    return version;
}

short dcdc_getLayerVersion()
{
    return DCDC_LAYER_VERSION;
}

short dcdc_PowerDown()
{
    if(!dcdc_Send(CMD_POWERDOWN, SDIO_POWERDOWN_KEY))
    {
        return -1;
    }
    return 0;
}

// Generic send function
short dcdc_Send(unsigned int cmd, unsigned int data)
{
    unsigned char TxData[3];
    unsigned int timeout;

    TxData[0] = cmd;
    TxData[1] = (unsigned char)(data>>8);       // High byte
    TxData[2] = data&0xFF;                      // Low bytes

    // Send start condition and slave address
    UCB0CTL1 |= UCTR + UCTXSTT;                 // Master Tx and Start Condition
    __delay_cycles(5000);
    if(UCB0IFG & UCNACKIFG)
    {
        // Nack received
        return (0);
    }

    // Send byte 0
    UCB0TXBUF = TxData[0];
    __delay_cycles(5000);
    if(UCB0IFG & UCNACKIFG)
    {
        // Nack received
        return (0);
    }

    // Send byte 1
    UCB0TXBUF = TxData[1];
    __delay_cycles(5000);
    if(UCB0IFG & UCNACKIFG)
    {
        // Nack received
        return (0);
    }

    // Send byte 3
    UCB0TXBUF = TxData[2];

    //Poll for transmit interrupt flag.
    timeout = 10000;
    while ((!(UCB0IFG & UCTXIFG)) && (--timeout>0));
    if (timeout==0)
    {
        // Error
        return (0);
    }

    //Send stop condition.
    UCB0CTL1 |= UCTXSTP;

    return (1);

}

// Generic receive function
short dcdc_Receive(unsigned int *data_read)
{
    unsigned char RxBuffer[2];
    unsigned int timeout;
    unsigned int val;

    timeout = 10000;
    while ((UCB0CTL1 & UCTXSTP) && (--timeout>0));             // Ensure stop condition got sent
    if (timeout==0)
    {
        // Slave did not send in time
        return (0);
    }

    UCB0CTL1 &= ~UCTR;                      // Master receives
    UCB0CTL1 |= UCTXSTT;	                  // Start transmit enable

    // Receive byte 0
    timeout = 10000;
    while ((((UCB0IFG&UCRXIFG)==0)) && (--timeout>0));
    if (timeout==0)
    {
        // Slave did not send in time
        return (0);
    }
    UCB0IFG &= ~UCRXIFG;
    RxBuffer[0] = (unsigned char)UCB0RXBUF;	// Read RX buffer byte

    // Receive byte 1
    UCB0CTL1 |= UCTXSTP;            // Generate I2C stop condition
    timeout = 10000;
    while ((UCB0CTL1& UCTXSTP) && (--timeout>0));      // Wait for Stop
    if (timeout==0)
    {
        // Slave did not send in time
        return (0);
    }
    timeout = 10000;
    while ((((UCB0IFG&UCRXIFG)==0)) && (--timeout>0));  // Wait for byte RX
    if (timeout==0)
    {
        // Slave did not send in time
        return (0);
    }
    RxBuffer[1] = (unsigned char)UCB0RXBUF;	// Read RX buffer byte

    val  = ((unsigned int) RxBuffer[0])<<8 ;
    val += ((unsigned int) RxBuffer[1]);
    *data_read=val;

    return (1);
}
