/**
* \ingroup MODULHIL
*
* \file hil.c
*
* \brief Hardware Independent Layer Interface
*
*/
/*
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
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

//! \page hil_page1  (HIL)
//! \brief The HIL ...
//! \author  Detlef Fink (03/10/2011)
//!
//! <b>files</b>\n
//! \li hil.c
//! \li hil_2w.c
//! \li hil_4w.c
//! \li hil_delays.s43
//! \li hil_delays.h
//! \li arch.h

#include "hw_compiler_specific.h"
#include "arch.h"
#include "edt.h"
#include "hilDelays.h"
#include "HalGlobalVars.h"
#include "stream.h"

unsigned short gprotocol_id;
unsigned short gTclkHighWhilePsa;
unsigned short bVccOn;  // Target Vcc is switched off by default
                         // dependant on calls to _hil_SetVcc()
unsigned char *tstctrl_port_;
unsigned char entdi2tdo_;
unsigned short jtagReleased;

// function prototypes for map initialization
// common HIL configuration methods
short _hil_Init( void );
short _hil_SetVcc(unsigned short Vcc);
short _hil_GetVcc(double* Vcc, double* ExtVcc);
short _hil_SetProtocol(unsigned short protocol_id);
void  _hil_SetPsaSetup(unsigned short enhanced);
void  _hil_SetPsaTCLK(unsigned short tclkValue);
short _hil_Open( unsigned char state );
short _hil_Close( void );
short IccMonitor_Process(unsigned short flags); // flags: to be compatible with HIL calls
void  _hil_EntrySequences(unsigned char states);


void _hil_SetJtagSpeed(unsigned short speed, unsigned short sbwSpeed);
void _hil_ConfigureSetPc(unsigned short x) {}


void _hil_SetReset(unsigned char);
void _hil_SetTest(unsigned char);
void _hil_SetTMS(unsigned char);
void _hil_SetTCK(unsigned char);
void _hil_SetTDI(unsigned char);
void _hil_initTimerB0(void);
void _hil_BSL_EntrySequence(unsigned short dummy);
void _hil_SwitchVccFET(unsigned short SwitchVccFET){};

edt_common_methods_t _edt_Common_Methods =
{
    _hil_Init,
    _hil_SetVcc,
    _hil_SwitchVccFET,
    _hil_GetVcc,
    _hil_SetProtocol,
    _hil_SetPsaTCLK,
    _hil_Open,
    _hil_Close,
    _hil_Delay_1us,
    _hil_Delay_1ms,
    IccMonitor_Process,
    _hil_EntrySequences,
    _hil_SetReset,
    _hil_SetTest,
    _hil_SetJtagSpeed,
    _hil_ConfigureSetPc,
    _hil_initTimerB0,
    _hil_BSL_EntrySequence,
    _hil_SetTMS,
    _hil_SetTCK,
    _hil_SetTDI,
};
// init by HIL init function
VAR_AT(edt_distinct_methods_t _edt_Distinct_Methods, HAL_ADDR_VAR_EDT_DISTINCT_METHODS);

// protocol specific methods
// (must be implemeted in 4-wire JTAG and 2-wire Spy-Bi-Wire mode)
extern short _hil_4w_TapReset(void);
extern short _hil_4w_CheckJtagFuse(void);
extern unsigned char _hil_4w_Instr(unsigned char Instruction);
//extern unsigned long _hil_4w_SetReg_XBits(unsigned long Data, short Bits);
extern unsigned char _hil_4w_SetReg_XBits08(unsigned char Data);
extern unsigned short _hil_4w_SetReg_XBits16(unsigned short Data);
extern unsigned long _hil_4w_SetReg_XBits20(unsigned long Data);
extern unsigned long _hil_4w_SetReg_XBits32(unsigned long Data);
extern unsigned long long _hil_4w_SetReg_XBits64(unsigned long long Data);
extern unsigned long long _hil_4w_SetReg_XBits8_64(unsigned long long Data, unsigned short loopCount, unsigned short JStateVersion);

extern void _hil_4w_Tclk(unsigned char state);
extern void _hil_4w_StepPsa(unsigned long length);
extern void _hil_4w_StepPsaTclkHigh(unsigned long length);
extern short _hil_4w_BlowFuse(unsigned char targetHasTestVpp);

extern short _hil_2w_TapReset(void);
extern short _hil_2w_CheckJtagFuse(void);
extern unsigned char _hil_2w_Instr(unsigned char Instruction);
//extern unsigned long _hil_2w_SetReg_XBits(unsigned long Data, short Bits);
extern unsigned char _hil_2w_SetReg_XBits08(unsigned char Data);
extern unsigned short _hil_2w_SetReg_XBits16(unsigned short Data);
extern unsigned long _hil_2w_SetReg_XBits20(unsigned long Data);
extern unsigned long _hil_2w_SetReg_XBits32(unsigned long Data);
extern unsigned long long _hil_2w_SetReg_XBits64(unsigned long long Data);
extern unsigned long long _hil_2w_SetReg_XBits8_64(unsigned long long Data, unsigned short loopCount, unsigned short JStateVersion);
extern void _hil_2w_Tclk(unsigned char state);
extern void _hil_2w_StepPsa(unsigned long length);
extern void _hil_2w_StepPsaTclkHigh(unsigned long length);
extern short _hil_2w_BlowFuse(unsigned char targetHasTestVpp);
void _hil_2w_StepPsa_Xv2(unsigned long length);

extern unsigned char _hil_2w_GetPrevInstruction();
extern unsigned char _hil_4w_GetPrevInstruction();

short _hil_dummy_TapReset(void) {return 0;}
short _hil_dummy_CheckJtagFuse(void){return 0;}
unsigned char _hil_dummy_Instr(unsigned char Instruction){return 0;}
unsigned char _hil_dummy_SetReg_XBits08(unsigned char Data){return 0;}
unsigned short _hil_dummy_SetReg_XBits16(unsigned short Data){return 0;}
unsigned long _hil_dummy_SetReg_XBits20(unsigned long Data){return 0;}
unsigned long _hil_dummy_SetReg_XBits32(unsigned long Data){return 0;}
unsigned long long _hil_dummy_SetReg_XBits64(unsigned long long Data){return 0;}
unsigned long long _hil_dummy_SetReg_XBits8_64(unsigned long long Data, unsigned short loopCount, unsigned short JStateVersion){return 0;}
void _hil_dummy_Tclk(unsigned char state){return;}
void _hil_dummy_StepPsa(unsigned long length){return;}
short _hil_dummy_BlowFuse(unsigned char targetHasTestVpp){return 0;}
unsigned char _hil_dummy_GetPrevInstruction(){return 0;}

// PSA distinct methods
void _hil_EnhancedPsaSetup(unsigned long address);
void _hil_PsaSetup(unsigned long address);
void _hil_EnhancedPsaEnd(void);
void _hil_PsaEnd(void);

#define  ExtLimit        1700            // a level higher than this means external voltage available
#define  ConvRange       4095.0f         // 12bit conversion range both for ADC and DAC
#define  VCCTmin         ConvRange       // 0xFFF, write this value to the DAC register to setup minimum VCCT
#define  Res1            60.4f           // FB2VCCT
#define  Res2            30.1f           // FB2GND
#define  Res3            39.2f           // FB2SETVCCT
#define  Res4            30.1f           // VCCT2ADC1
#define  Res5            22.1f           // ADC12GND
#define  Vref            1224.0f         // the reference voltage of the voltage regulator
#define  VCref           2500.0f         // conversion reference voltage
#define  minVCCT         ((Res1*((Vref/Res2)-((VCref-Vref)/Res3)))+Vref)
#define  maxVCCT         (((Res1*((Res2+Res3)/(Res2*Res3)))+1)*Vref)
#define  VCCICHN         0
#define  VCCTCHN         1
#define  VCCRCHN         3

const unsigned short ADC_MV[4] = { 5905, 5905, 6934, 5905 };  // input voltages included the scale by resitors
const unsigned short ADC_CONV_RANGE = 4096;                   // 1 Bit error, because div is with 4096 faster
const unsigned short ADC_AVERAGES = 4;
const unsigned short DAC_CONV_RANGE = 4095;                   //

extern const unsigned char DMA_TMSL_TDIL[];

static volatile unsigned char VFSettleFlag = 0;
static volatile unsigned char useTDI = 0;

unsigned short ConvertAD(unsigned short channel)
{
    unsigned long tmp;
    unsigned short ret_mv = 0;

    if(channel < 4 )
    {
        tmp = *((unsigned short*)&ADC12MEM0+channel) +
        *((unsigned short*)&ADC12MEM4+channel) +
        *((unsigned short*)&ADC12MEM8+channel) +
        *((unsigned short*)&ADC12MEM12+channel);
        tmp *= ADC_MV[channel];
        tmp /= ADC_CONV_RANGE * ADC_AVERAGES;
        ret_mv = tmp;
    }
    return(ret_mv);
}

/*----------------------------------------------------------------------------
   This function performs a single AD conversion on the selected pin.
   Uses internal reference 2.5V (VR+ = VREF+, VR- = AVSS).
   Only used for SetTargetVcc() and Selftest().
   Arguments: word pinmask (bit position of selected analog input Ax)
   Result:    word (12 bit ADC conversion result)
*/
short ConvertAD_(short channel)
{
// select channel and do conversion
  ADC12CTL0  &= ~ENC;                   // Disable conversion, write controls
  ADC12MCTL0  = 0x10 + channel;         // select Vref and analog channel Ax
  ADC12CTL0  |= ENC;                    // Enable conversions
  ADC12CTL0  |= ADC12SC;                // Start conversions
  while ((ADC12IFG & BIT0) == 0);       // wait until conversion is done
  return(ADC12MEM0);                    // return conversion result from MEM0
}

unsigned short last_vcc; // Initialization in HalGlobalVars.c


#pragma optimize = none
void SetDac(unsigned short vcc)
{
    signed short nomDac;
    signed short corr;

    if(last_vcc != vcc)
    {
        nomDac = (unsigned short)(ConvRange - (vcc - minVCCT));      // calculate approximated value
        nomDac += (85 - (vcc/20));                 // error reduction of approximated value
        if(nomDac > ConvRange)
        {
            nomDac = ConvRange;
        }
        if(nomDac < 0)
        {
            nomDac = 0;
        }
        DAC12_0DAT = nomDac;                  // set voltage
        DAC12_0CTL |= DAC12ENC;               // enable DAC12
        last_vcc = vcc;
    }
    else
    {
        corr = ConvertAD(VCCTCHN);
        if(corr > (vcc+10))
        {
            if(DAC12_0DAT < 4095)
            {
                DAC12_0DAT++;
            }
        }
        else if(corr < (vcc-10))
        {
            if(DAC12_0DAT)
            {
                DAC12_0DAT--;
            }
        }
    }
}

// -----------------------------------------------------------------------------
short _hil_Init( void )
{
    // Setup ADC12
    ADC12CTL0  &= ~ENC;					  // Disable conversions, write controls
    ADC12CTL0  = ADC12ON | REFON | REF2_5V | MSC | SHT0_2 | SHT1_2;  // Turn on ADC12, VREF = 2.5v, set samp. time
    ADC12CTL1  = SHP | CONSEQ_3;					  // Use sampling timer, CstartAddr = MEM0
    // four time averaging on each channel
    ADC12MCTL0  = 0x10 | 0;           // select Vref and cannel
    ADC12MCTL1  = 0x10 | 1;           // select Vref and cannel
    ADC12MCTL2  = 0x10 | 2;           // select Vref and cannel
    ADC12MCTL3  = 0x10 | 3;           // select Vref and cannel
    ADC12MCTL4  = 0x10 | 0;           // select Vref and cannel
    ADC12MCTL5  = 0x10 | 1;           // select Vref and cannel
    ADC12MCTL6  = 0x10 | 2;           // select Vref and cannel
    ADC12MCTL7  = 0x10 | 3;           // select Vref and cannel
    ADC12MCTL8  = 0x10 | 0;           // select Vref and cannel
    ADC12MCTL9  = 0x10 | 1;           // select Vref and cannel
    ADC12MCTL10 = 0x10 | 2;           // select Vref and cannel
    ADC12MCTL11 = 0x10 | 3;           // select Vref and cannel
    ADC12MCTL12 = 0x10 | 0;           // select Vref and cannel
    ADC12MCTL13 = 0x10 | 1;           // select Vref and cannel
    ADC12MCTL14 = 0x10 | 2;           // select Vref and cannel
    ADC12MCTL15 = 0x10 | 3;           // select Vref and cannel

    ADC12CTL0  |= ENC;                    // Enable conversions
    // ADCs run free
    ADC12CTL0  |= ADC12SC;                // Start conversions

    DAC12_0DAT = VCCTmin;                           // set voltage to MIN ~0
    DAC12_0CTL = DAC12CALON + DAC12IR + DAC12AMP_5; // Internal ref gain 1

    // Init SPI for JTAG
    UCTL1 |= SWRST;
    UCTL1 = CHAR | SYNC | MM | SWRST;
    UTCTL1 = CKPL | SSEL0 | STC | TXEPT;
    UBR01  = 2; //SPI_DIV_JTAG;				    // Load SPI frequency divider
    UBR11  = 0x00;
    UMCTL1 = 0x00;						// Clear Modulation Register
    UCTL1 &= ~SWRST;
    ME2   |= USPIE1;					// Enable SPI module

    // DMA init
    DMACTL0 = 0;                  // URXIFG0 trigger for DMS channel0, channel1 and channel2 are software triggered
    DMACTL1 = 0;                           // DAM immediatly, no round-robin, no NMI interrupt

    DMA1CTL = ( DMADT0 | DMASRCINCR1 | DMASRCINCR0 | DMASRCBYTE | DMADSTBYTE);
    DMA1DA = (unsigned int)_Jtag.Out; //JTAGOUT;       // set destination address
    DMA2CTL = ( DMADT0 | DMASRCINCR1 | DMASRCINCR0 | DMASRCBYTE | DMADSTBYTE);
    DMA2DA = (unsigned int)_Jtag.Out; //JTAGOUT;       // set destination address
    DMA2SZ = 4;                            // JTAG test/idle sequence is always 4 transisions

    // set default debug protocol to JTAG
    gprotocol_id = JTAG;
    // set default to TCLK low when doing PSA
    gTclkHighWhilePsa = 0;
    // initialize function pointers to distinct functions
    _hil_SetProtocol(gprotocol_id);
    jtagReleased = 1;

    return 0;
}

void _hil_SetJtagSpeed(unsigned short speed, unsigned short sbwSpeed)
{
    if(speed)
    {
        UBR01  = speed; //SPI_DIV_JTAG;				    // Load SPI frequency divider
    }
}
// -----------------------------------------------------------------------------
#pragma optimize = medium
short _hil_SetVcc(unsigned short Vcc)
{
  static unsigned char bPowerUp = 1;

    if(Vcc)
    {
          //This workaround is necessary to set TCK to a defined low level before
          //applying Vcc to the target device
          //otherwise the target will not power up correctly and the UIF can't
          //establish a SBW communication

          if(bPowerUp)
          {
            P5OUT &= ~BIT3;
            TGTCTRLOUT &= ~SELT;
            bVccOn = Vcc;
            SetDac(Vcc+50);
            VCCTon();
            _hil_Delay_1ms(1);
            bPowerUp = 0;
          }
          else
          {
            bVccOn = Vcc;
            SetDac(Vcc+50);
            VCCTon();
          }
    }
    else
    {
        VCCToff();
        _hil_Close();
        bVccOn = 0;
        bPowerUp = 1;
    }
    return 0;
}
// -----------------------------------------------------------------------------
short _hil_GetVcc(double* Vcc, double* ExtVcc)
{
    if(P4OUT & BIT3)
    {
        *Vcc = ConvertAD(VCCTCHN);
    }
    else
    {
        *Vcc = 0;
    }
    *ExtVcc = ConvertAD(VCCICHN);
    return 0;
}
// -----------------------------------------------------------------------------
short _hil_SetProtocol(unsigned short protocol_id)
{
    short ret_value = 0;

    if((protocol_id == JTAG))
    {
        gprotocol_id = JTAG;
    }
    else if((protocol_id == SPYBIWIRE))
    {
        gprotocol_id = SPYBIWIRE;
    }
    else if((protocol_id == SPYBIWIREJTAG))
    {
        gprotocol_id = SPYBIWIREJTAG;
    }
    else
    {
        ret_value = -1;
    }

    tstctrl_port_ = (unsigned char*)&P4OUT;
    entdi2tdo_ = BIT2;

    if(gprotocol_id == SPYBIWIRE)
    {
        _edt_Distinct_Methods.TapReset = _hil_2w_TapReset;
        _edt_Distinct_Methods.CheckJtagFuse = _hil_2w_CheckJtagFuse;
        _edt_Distinct_Methods.Instr = _hil_2w_Instr;
        _edt_Distinct_Methods.SetReg_XBits08 = _hil_2w_SetReg_XBits08;
        _edt_Distinct_Methods.SetReg_XBits16 = _hil_2w_SetReg_XBits16;
        _edt_Distinct_Methods.SetReg_XBits20 = _hil_2w_SetReg_XBits20;
        _edt_Distinct_Methods.SetReg_XBits32 = _hil_2w_SetReg_XBits32;
        _edt_Distinct_Methods.SetReg_XBits64 = _hil_2w_SetReg_XBits64;
        _edt_Distinct_Methods.SetReg_XBits8_64 = _hil_2w_SetReg_XBits8_64;
        _edt_Distinct_Methods.Tclk = _hil_2w_Tclk;
        _edt_Distinct_Methods.GetPrevInstruction = _hil_2w_GetPrevInstruction;

        if(gTclkHighWhilePsa == 1)
        {
            _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsaTclkHigh;
        }
        else if(gTclkHighWhilePsa == 2)
        {
          _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsa_Xv2;
        }
        else
        {
            _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsa;
        }

        _edt_Distinct_Methods.BlowFuse = _hil_2w_BlowFuse;
        DMA1SZ = 6;                         // load DMA1 with size
        DMA2SZ = 6;                         // load DMA1 with size
        DMA2SA = (unsigned int)DMA_TMSL_TDIL;
    }
    else
    {
        _edt_Distinct_Methods.TapReset = _hil_4w_TapReset;
        _edt_Distinct_Methods.CheckJtagFuse = _hil_4w_CheckJtagFuse;
        _edt_Distinct_Methods.Instr = _hil_4w_Instr;
        _edt_Distinct_Methods.SetReg_XBits08 = _hil_4w_SetReg_XBits08;
        _edt_Distinct_Methods.SetReg_XBits16 = _hil_4w_SetReg_XBits16;
        _edt_Distinct_Methods.SetReg_XBits20 = _hil_4w_SetReg_XBits20;
        _edt_Distinct_Methods.SetReg_XBits32 = _hil_4w_SetReg_XBits32;
        _edt_Distinct_Methods.SetReg_XBits64 = _hil_4w_SetReg_XBits64;
        _edt_Distinct_Methods.SetReg_XBits8_64 = _hil_4w_SetReg_XBits8_64;
        _edt_Distinct_Methods.Tclk = _hil_4w_Tclk;
        _edt_Distinct_Methods.GetPrevInstruction = _hil_4w_GetPrevInstruction;
        if(gTclkHighWhilePsa == 0 || gTclkHighWhilePsa == 2)
        {
            _edt_Distinct_Methods.StepPsa = _hil_4w_StepPsa;
        }
        else
        {
            _edt_Distinct_Methods.StepPsa = _hil_4w_StepPsaTclkHigh;
        }
        _edt_Distinct_Methods.BlowFuse = _hil_4w_BlowFuse;
        DMA2SZ = 4;                         // load DMA1 with size
    }
    return(ret_value);
}

void _hil_SetPsaTCLK(unsigned short tclkValue)
{
    gTclkHighWhilePsa = tclkValue;
    if(gprotocol_id == SPYBIWIRE)
    {
        if(gTclkHighWhilePsa == 1)
        {
            _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsaTclkHigh;
        }
        else if(gTclkHighWhilePsa == 2)
        {
          _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsa_Xv2;
        }
        else
        {
            _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsa;
        }
    }
    else
    {
        if(gTclkHighWhilePsa == 1)
        {
            _edt_Distinct_Methods.StepPsa = _hil_4w_StepPsaTclkHigh;
        }
        else
        {
            _edt_Distinct_Methods.StepPsa = _hil_4w_StepPsa;
        }
    }
}

static void _hil_Release(void)
{
    if(!jtagReleased)
    {
        if(gprotocol_id == SPYBIWIRE)
        {
            // drive target RST/SBWTDIO pin high
            JTAGOUT |=  sbwdato;                // TDI drives target RST high
            TSTCTRLOUT |= ENTDI2TDO;            // disable TDI2TDO
            IHIL_Delay_1ms(1);
            // drive target TEST/SBWTCK pin low
            JTAGOUT &= ~sbwclk;                 // TCK drives target TEST low - release Spy-Bi-Wire logic
        }
        else
        {
            // hands off from target RST pin  -> will be pulled up
            TSTCTRLOUT |= ENTDI2TDO;          // disable TDI2TDO
            TGTCTRLOUT |=  SELT;               // disable JTAG & RST pin drivers
            IHIL_Delay_1ms(1);
            // hands off from target TEST pin -> will be pulled down
            TSTCTRLOUT |=  TEST;               // reset target TEST pin low
        }
        _edt_Distinct_Methods.TapReset  = _hil_dummy_TapReset;
        _edt_Distinct_Methods.CheckJtagFuse = _hil_dummy_CheckJtagFuse;
        _edt_Distinct_Methods.Instr = _hil_dummy_Instr;
        _edt_Distinct_Methods.SetReg_XBits08 = _hil_dummy_SetReg_XBits08;
        _edt_Distinct_Methods.SetReg_XBits16 = _hil_dummy_SetReg_XBits16;
        _edt_Distinct_Methods.SetReg_XBits20 = _hil_dummy_SetReg_XBits20;
        _edt_Distinct_Methods.SetReg_XBits32 = _hil_dummy_SetReg_XBits32;
        _edt_Distinct_Methods.SetReg_XBits64 = _hil_dummy_SetReg_XBits64;
        _edt_Distinct_Methods.SetReg_XBits8_64 = _hil_dummy_SetReg_XBits8_64;
        _edt_Distinct_Methods.Tclk = _hil_dummy_Tclk;
        _edt_Distinct_Methods.GetPrevInstruction = _hil_dummy_GetPrevInstruction;
        _edt_Distinct_Methods.StepPsa = _hil_dummy_StepPsa;
        _edt_Distinct_Methods.BlowFuse = _hil_dummy_BlowFuse;

        jtagReleased = 1;
    }
    IHIL_Delay_1ms(5);
}

#define RSTLOW_SBW   0
#define RSTLOW_JTAG  1
#define RSTHIGH_SBW  2
#define RSTHIGH_JTAG 3

#define qDriveSignals() { *_Jtag.Out |= (_Jtag.TCK + _Jtag.TMS + _Jtag.TDI);   \
                          TSTCTRLOUT |=  ENTDI2TDO;                            \
                          TGTCTRLOUT |=  TGTRST;                               \
                          TGTCTRLOUT &= ~SELT;                                 \
                        }

#define qDriveSbw()     { TSTCTRLOUT |=  TEST;                                 \
                          JTAGOUT |=  sbwdato;                                 \
                          TSTCTRLOUT &= ~ENTDI2TDO;                            \
                          IHIL_Delay_1ms(1);                                    \
                          JTAGOUT &= ~sbwclk;                                  \
                          TGTCTRLOUT &= ~SELT;                                 \
                        }

/*-------------RstLow_JTAG----------------
            ________           __________
Test ______|        |_________|
                          _______________
Rst_____________________|
----------------------------------------*/

INLINE(forced)
static void _hil_EntrySequences_RstLow_JTAG()
{
    __disable_interrupt();
    TSTset0;                    //1
    IHIL_Delay_1ms(4);           //reset TEST logic

    RSTset0;                    //2

    TSTset1;                    //3
    IHIL_Delay_1ms(50);         //activate TEST logic

    RSTset0;                    //4
    IHIL_Delay_1us(40);

     // for 4-wire JTAG clear Test pin Test(0)
    ((TSTCTRLOUT) |= (TEST));   //5
    IHIL_Delay_1us(2);

    // for 4-wire JTAG -dry  Reset(1)
    TGTCTRLOUT &= ~SELT;
    TGTCTRLOUT |=  TGTRST;
    IHIL_Delay_1us(2);

    // 4-wire JTAG - Test (1)
    ((TSTCTRLOUT) &= (~TEST));  //7
    IHIL_Delay_1ms(5);
    __enable_interrupt();
}

/*-------------RstHigh_JTAG--------------
            ________           __________
Test ______|        |_________|
         _______                   ______
Rst____|       |_________________|
----------------------------------------*/

INLINE(forced)
static void _hil_EntrySequences_RstHigh_JTAG()
{
    TSTset0;                    //1
    IHIL_Delay_1ms(1);           //reset TEST logic

    RSTset1;                    //2

    TSTset1;                    //3
    IHIL_Delay_1ms(100);         //activate TEST logic

    RSTset0;                    //4
    IHIL_Delay_1us(40);

    // for 4-wire JTAG clear Test pin Test(0)
    ((TSTCTRLOUT) |= (TEST));   //5
    IHIL_Delay_1us(1);

    // for 4-wire JTAG - Test (1)
    ((TSTCTRLOUT) &= (~TEST));  //7
    IHIL_Delay_1us(40);

    // phase 5 Reset(1)
    TGTCTRLOUT &= ~SELT; TGTCTRLOUT |=  TGTRST;
    IHIL_Delay_1ms(5);
}

/*-------------RstHigh_SBW---------------
            ________           __________
Test ______|        |_________|
        _________________________________
Rst____|
----------------------------------------*/
INLINE(forced)
static void _hil_EntrySequences_RstHigh_SBW()
{
    TSTset0;                //1
    IHIL_Delay_1ms(1);       // reset TEST logic

    RSTset1;                //2

    TSTset1;                //3
    IHIL_Delay_1ms(100);     // activate TEST logic

    // phase 1
    RSTset1;                //4
    IHIL_Delay_1us(40);

    // phase 2 -> TEST pin to 0, no change on RST pin
    // for Spy-Bi-Wire
    __disable_interrupt();
    JTAGOUT &= ~sbwclk;     //5
    IHIL_Delay_1us(1);

    // phase 4 -> TEST pin to 1, no change on RST pin
    // for Spy-Bi-Wire
    JTAGOUT |= sbwclk;      //7
    __enable_interrupt();
    IHIL_Delay_1us(40);

    IHIL_Delay_1ms(5);
}

/*-------------RstLow_SBW----------------
            ________           __________
Test ______|        |_________|
               __________________________
Rst__________|
----------------------------------------*/
INLINE(forced)
static void _hil_EntrySequences_RstLow_SBW()
{
    TSTset0;                //1
    IHIL_Delay_1ms(1);       // reset TEST logic

    RSTset0;                //2

    TSTset1;                //3
    IHIL_Delay_1ms(100);     // activate TEST logic

    // phase 1
    RSTset1;                //4
    IHIL_Delay_1us(40);

    // phase 2 -> TEST pin to 0, no change on RST pin
    // for Spy-Bi-Wire
    __disable_interrupt();
    JTAGOUT &= ~sbwclk;     //5
    IHIL_Delay_1us(1);

    // phase 4 -> TEST pin to 1, no change on RST pin
    // for Spy-Bi-Wire
    JTAGOUT |= sbwclk;      //7
    __enable_interrupt();
    IHIL_Delay_1us(40);
    IHIL_Delay_1ms(5);
}

void _hil_EntrySequences(unsigned char states)
{
    switch(gprotocol_id)
    {
    case SPYBIWIRE:
        if (states == RSTLOW)
        {
            _hil_EntrySequences_RstLow_SBW();
        }
        if (states == RSTHIGH)
        {
            _hil_EntrySequences_RstHigh_SBW();
        }
        break;

    case SPYBIWIREJTAG:
        if (states == RSTLOW)
        {
            _hil_EntrySequences_RstLow_JTAG();
        }
        if (states == RSTHIGH)
        {
            _hil_EntrySequences_RstHigh_JTAG();
        }
        break;

    default:
        TSTset1
        break;
    }
}

// -----------------------------------------------------------------------------
static void _hil_Connect(unsigned char state)
{
    if(jtagReleased)
    {
        _hil_SetProtocol(gprotocol_id);
    }

    if(state == RSTHIGH)
    {
        if(gprotocol_id == SPYBIWIRE)
        {
            qDriveSbw();
            IHIL_Delay_1ms(1);
            _hil_EntrySequences_RstHigh_SBW();
        }
        else
        {
            qDriveSignals();
            IHIL_Delay_1ms(1);
            if(gprotocol_id == SPYBIWIREJTAG)
            {
                _hil_EntrySequences_RstHigh_JTAG();
            }
            else
            {
                TSTset1;
            }
        }
    }
    else // state  == RSTLOW
    {
        if(gprotocol_id == SPYBIWIRE)
        {
            qDriveSbw();
            IHIL_Delay_1ms(1);
            _hil_EntrySequences_RstLow_SBW();
        }
        else
        {
            qDriveSignals();
            IHIL_Delay_1ms(1);
            if(gprotocol_id == SPYBIWIREJTAG)
            {
                _hil_EntrySequences_RstLow_JTAG();

            }
            else
            {
                TSTset1;
            }
        }
    }
    jtagReleased = 0;
}

// -----------------------------------------------------------------------------
short _hil_Open( unsigned char state)
{
    _hil_Connect(state);
    return 0;
}
// -----------------------------------------------------------------------------
short _hil_Close( void )
{
    _hil_Release();
    return 0;
}

// *****************************************************************************
// Icc Monitor
char bIccMonitorOn;                 // Initialization in HalGlobalVars.c
char bHighCurrent;                  // Initialization in HalGlobalVars.c
signed short last_ext_vcc;          // Initialization in HalGlobalVars.c
unsigned short over_current_count;   // Initialization in HalGlobalVars.c

short IccMonitor_Process(unsigned short flags)
{
    signed short ext_vcc;
    signed short int_vcc;
    signed short rint_vcc;

    ext_vcc = ConvertAD(VCCICHN);          // convert channel
    int_vcc = ConvertAD(VCCTCHN);          // convert channel
    rint_vcc = ConvertAD(VCCRCHN);

    if(ext_vcc >= ExtLimit) // external voltage in the range 1.7V >=
    {
        if((ext_vcc < (last_ext_vcc-10)) || ext_vcc > (last_ext_vcc+10))
        {
            last_ext_vcc = ext_vcc;
        }
        SetDac(last_ext_vcc);               // let's hace a look at the external supply pin
    }
    else
    {
        if(bVccOn != 0)
        {
          _hil_SetVcc(bVccOn);
        }
    }

    if(bVccOn && bIccMonitorOn)
    {
        if((rint_vcc - int_vcc) > 150) // eqivalent to > 100mA
        {
            over_current_count++;
            if(over_current_count > 400)
            {
                bHighCurrent = 1;
                IHIL_Close();
                P4OUT &= ~BIT3;                             // turn off target VCC
                bVccOn = 0;
                STREAM_biosLedOff(1);
            }
        }
        else
        {
            over_current_count = 0;
        }
    }
    else
    {
        over_current_count = 0;
    }
    return 0;
}

// -----------------------------------------------------------------------------
#pragma vector=TIMERA0_VECTOR
__interrupt void TA_CCR0_ISR (void)
{
    static unsigned char cnt  = 0;

    short x;
    cnt++;
    if(cnt == 10)
    {
        cnt   = 0;     // reset pulse counter
        TACTL = 0;     // stop Timer A
        x = ConvertAD_(VFCHN);
        if(x < 0xFFF)   // 12-bit A/D thus value must exceed maximum measureable value (+-6.6V)
        {
            TACTL = (TASSEL_1 + TACLR + MC_1);      // start Timer A again and produce pulses
        } else {
            VFSettleFlag = 0;        // send message to main program
        }
    }
}

// -----------------------------------------------------------------------------
void SetVFuse(void)
{
    long TimeOut = DEF_TIMEOUT;
    VFSettleFlag = 1;
    P2DIR |= BIT7;
    P2SEL |= BIT7;
    CCR0  = TA_SETVF_DIV;
    TACCTL0 = OUTMOD_4 + CCIE;        // outmode toggle
    TACTL = (TASSEL_1 + TACLR + MC_1);// clear TAR and start Timer A in up-mode
    while(VFSettleFlag && TimeOut)
    {
        TimeOut--;
    }
    TACCTL0 = 0;                      // disable CCR0 interrupt
    P2SEL &= ~BIT7;
    P2DIR &= ~BIT7;
}

// -----------------------------------------------------------------------------
void SetVpp(long voltage)
{
    if(voltage)
    {
        SetVFuse();
        if(useTDI)
        {
            VPPon(VPP_ON_TDI);
        }
        else
        {
            VPPon(VPP_ON_TEST);
        }
        IHIL_Delay_1ms(10);
    }
    else
    {
        VPPoff();
    }
}

// -----------------------------------------------------------------------------
void testVpp(unsigned char mode)
{
    if(mode)
    {
        SETTDOOUT;
        SETTDION;
    }
    else
    {
        SETTDIOFF;
        SETTDOIN;
    }

    useTDI = !mode;
}

// -----------------------------------------------------------------------------
void _hil_SetReset(unsigned char value)
{
    if(value)
    {
        RSTset1
    }
    else
    {
        RSTset0
    }
}

// -----------------------------------------------------------------------------
void _hil_SetTMS(unsigned char value)
{
    if(value)
    {
        TMSset1
    }
    else
    {
        TMSset0
    }
}

// -----------------------------------------------------------------------------
void _hil_SetTDI(unsigned char value)
{
    if(value)
    {
        TDIset1
    }
    else
    {
        TDIset0
    }
}

// -----------------------------------------------------------------------------
void _hil_SetTCK(unsigned char value)
{
    if(value)
    {
        TCKset1
    }
    else
    {
        TCKset0
    }
}

// -----------------------------------------------------------------------------
void _hil_SetTest(unsigned char value)
{
    if(value)
    {
        TSTset1
    }
    else
    {
        TSTset0
    }
}
#ifdef uController
    #pragma optimize = low
#endif

// -----------------------------------------------------------------------------
void _hil_initTimerB0(void)
{
    //Empty for UIF
}

/*------------------------------------------------------------------------*/
/*                      __________     <15us    __________                */
/* Test/SBWTCK ________|          |____________|          |___            */
/*                                                  __________            */
/* RST/SBWTDIO ____________________________________|                      */
/*                                                                        */
/*                                                 BSL Enabled here!!     */
/*------------------------------------------------------------------------*/
void _hil_BSL_EntrySequence(unsigned short dummy)
{
    if(gprotocol_id == SPYBIWIRE)
    {
        // set Default state of RST and TST
        JTAGOUT |= sbwclk;            // set test to 1
        IHIL_Delay_1ms(1);
        RSTset1;                      // set RST 1;

        // INIT phase
        JTAGOUT &= ~sbwclk;           // set test 0;
        IHIL_Delay_1ms(100);

        JTAGOUT &= ~sbwdato;          // set RST low
        IHIL_Delay_1ms(100);

        JTAGOUT |= sbwclk;            // set test to 1
        IHIL_Delay_1ms(100);

        /*this is the first pulse keep it low for less than 15us */
        JTAGOUT &= ~sbwclk;          // set test 0;
        IHIL_Delay_1us(10);
        JTAGOUT |= sbwclk;           // set test 1;

        IHIL_Delay_1ms(100);
        RSTset1;
        IHIL_Delay_1ms(100);
        JTAGOUT &= ~sbwclk;         // set test 0;
        IHIL_Delay_1ms(100);
    }
    else
    {
        // set Default state of RST and TST
        ((TSTCTRLOUT) &= (~TEST));    // set test to 1
        IHIL_Delay_1ms(1);
        RSTset1;                      // set RST 1;

        // INIT phase
        ((TSTCTRLOUT) |= (TEST));     // set test 0;
        IHIL_Delay_1ms(10);

        { TGTCTRLOUT &= ~SELT; TGTCTRLOUT &= ~TGTRST; }// set RST 0
        IHIL_Delay_1ms(10);

        ((TSTCTRLOUT) &= (~TEST));    // set test to 1
        IHIL_Delay_1ms(10);

        /*this is the first pulse keep it low for less than 15us */
        ((TSTCTRLOUT) |= (TEST));     // set test 0;
        IHIL_Delay_1us(10);
        ((TSTCTRLOUT) &= ~(TEST));    // set test 1;

        IHIL_Delay_1ms(10);
        RSTset1;                      // set RST 1;
        IHIL_Delay_1ms(10);
        ((TSTCTRLOUT) |= (TEST));     // set test 0 ;
        IHIL_Delay_1ms(10);
    }
 }

// -----------------------------------------------------------------------------
#ifdef uController
    #pragma optimize = none
#endif
void JSBW_EntrySequences(unsigned char states)
{

    qDriveSignals();
    IHIL_Delay_1ms(10);

    TSTset0;
    IHIL_Delay_1ms(1); // reset TEST logic

    if(states == 0 || states == 1)
    {
        TGTCTRLOUT &=  ~TGTRST;//RST 0
    }
    else
    {
        RSTset1;
    }
    IHIL_Delay_1us(10);

    TSTset1;
    IHIL_Delay_1ms(25); // activate TEST logic

    // phase 1
    if(states == 1 || states == 3)
    {
        RSTset0;
    }
    else
    {
        RSTset1;
    }
    IHIL_Delay_1us(40);


    // phase 2 -> TEST pin to 0, no change on RST pin
    if(states == 0 || states == 2)
    { // for Spy-Bi-Wire
        TSTCTRLOUT |= JSBsbwclk;
    }
    else
    { // for 4-wire JTAG
        ((TSTCTRLOUT) |= (TEST));   //5 clear the bastard test
    }

    // phase 3
    if(states == 1)
    {
         TGTCTRLOUT &= ~SELT;    // set the reset 1
         TGTCTRLOUT |=  TGTRST;  //6
    }
    IHIL_Delay_1us(1);

    // phase 4 -> TEST pin to 1, no change on RST pin
    if(states == 0 || states == 2)
    { // for Spy-Bi-Wire
        TSTCTRLOUT &=  ~JSBsbwclk;
    }
    else
    { // for 4-wire JTAG set Test pin
        ((TSTCTRLOUT) &= (~TEST));  //5
    }
    IHIL_Delay_1us(40);

    // phase 5
    if(states == 3)
    {
        TGTCTRLOUT &= ~SELT;
        TGTCTRLOUT |=  TGTRST;
    }
    IHIL_Delay_1ms(5);
}
/* EOF */
