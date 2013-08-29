/*
 * hil.c
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

//! \ingroup MODULHIL
//! \file hil.c
//! \brief
//!

//! \page hil_page1  (HIL)
//! \brief The HIL ...
//! \author  Berenbrinker Florian (01/01/2012)
//!
//! <b>files</b>\n
//! \li hil.c
//! \li hil_2w.c
//! \li hil_4w.c
//! \li hil_delays.s43
//! \li hil_delays.h
//! \li arch.h
#include <stdint.h>
#include "hw_compiler_specific.h"
#include "arch.h"
#include "edt.h"
#include "hilDelays.h"
//#include "HilGlobalVars.h"
#include "JTAG_defs.h"

#define HIL_VERSION        0x000B             // fist hil version for ezFET
#define HIL_SIGNATURE      0xF00DF00Dul


//! \brief version of bios code
const unsigned short hil_version_ @ "HILVERSION" = HIL_VERSION;
#pragma required=hil_version_
const unsigned long hil_signature_ @ "HILSIGNATURE" = HIL_SIGNATURE;
#pragma required=hil_signature_

unsigned short gTclkHighWhilePsa;
unsigned short bVccOn;  // Target Vcc is switched off by default
                        // dependant on calls to _hil_SetVcc()

unsigned short setPCclockBeforeCapture;

// function prototypes for map initialization
// common HIL configuration methods
short _hil_Init( void );
short _hil_SetVcc(unsigned short Vcc);
short _hil_GetVcc(double* Vcc, double* ExtVcc);
short _hil_SetProtocol(unsigned short protocol_id);
void  _hil_SetPsaSetup(unsigned short enhanced);
void  _hil_SetPsaTCLK(unsigned short tclkValue);
/*unsigned short _hil_GetTimeStamp(void);

unsigned short _hil_GetIMeasure(void);   // Get Current measurement ticks
unsigned char _hil_GetDCDCStatus(void); // Get the status of the DCDC Sub MCU*/

short _hil_Open( unsigned char state );
short _hil_Close( void );
short IccMonitor_Process(unsigned short flags); // flags: to be compatible with HIL calls
void  _hil_EntrySequences(unsigned char states);

void  _hil_SetJtagBits(unsigned char output);
unsigned char _hil_GetJtagBits(void);

void  _hil_SetTgtCtrlBits(unsigned char);
unsigned char _hil_GetTgtCtrlBits(void);

void  _hil_SetTestBits(unsigned char);
unsigned char _hil_GetTestBits(void);

void _hil_SetReset(unsigned char value);
void _hil_SetTest(unsigned char value);

void _hil_SetJtagSpeed(unsigned short jtagSpeed, unsigned short sbwSpeed);
void _hil_ConfigureSetPc (unsigned short PCclockBeforeCapture);

// protocol specific methods
/*extern short _hil_4w_TapReset(void);
extern short _hil_4w_CheckJtagFuse(void);
extern unsigned short _hil_4w_EnumChain(void);
extern unsigned char _hil_4w_Instr(unsigned char Instruction);
extern unsigned char _hil_4w_SetReg_XBits08(unsigned char Data);
extern unsigned short _hil_4w_SetReg_XBits16(unsigned short Data);
extern unsigned long _hil_4w_SetReg_XBits20(unsigned long Data);
extern unsigned long _hil_4w_SetReg_XBits32(unsigned long Data);
extern unsigned long long _hil_4w_SetReg_XBits64(unsigned long long Data);
extern void  _hil_4w_Tclk(unsigned char state);
extern void  _hil_4w_StepPsa(unsigned long length);
extern void  _hil_4w_StepPsaTclkHigh(unsigned long length);
extern short _hil_4w_BlowFuse(unsigned char targetHasTestVpp);*/
//extern void  _hil_4w_ConfigureSpeed(unsigned short speed);

//SBW2 DMA
extern short _hil_2w_TapReset_Dma(void);
extern short _hil_2w_CheckJtagFuse_Dma(void);
extern unsigned short _hil_2w_EnumChain_Dma(void);
extern unsigned char _hil_2w_Instr_Dma(unsigned char Instruction);
extern unsigned char _hil_2w_SetReg_XBits08_Dma(unsigned char Data);
extern unsigned short _hil_2w_SetReg_XBits16_Dma(unsigned short Data);
extern unsigned long _hil_2w_SetReg_XBits20_Dma(unsigned long Data);
extern unsigned long _hil_2w_SetReg_XBits32_Dma(unsigned long Data);
extern unsigned long long _hil_2w_SetReg_XBits64_Dma(unsigned long long Data);
extern unsigned long long _hil_2w_SetReg_XBits8_64(unsigned long long Data, unsigned short loopCount);
extern void  _hil_2w_Tclk_Dma(unsigned char state);
extern void  _hil_2w_StepPsa_Dma(unsigned long length);
extern void  _hil_2w_StepPsaTclkHigh_Dma(unsigned long length);
extern short _hil_2w_BlowFuse_Dma(unsigned char targetHasTestVpp);
extern void  _hil_2w_ConfigureSpeed_Dma(unsigned short speed);
extern unsigned char _hil_2w_GetPrevInstruction_Dma();
extern unsigned char DMA_TMSL_TDIL[];

// PSA distinct methods
void _hil_EnhancedPsaSetup(unsigned long address);
void _hil_PsaSetup(unsigned long address);
void _hil_EnhancedPsaEnd(void);
void _hil_PsaEnd(void);
void _hil_Release(void);
void _hil_initTimerB0(void);
void _hil_BSL_EntrySequence(void);
static void _hil_Connect(unsigned char state);

extern void initJtagSbw2Dma(struct jtag tmp);
//extern void initJtagSbw4(struct jtag tmp);


edt_common_methods_t   _edt_Common_Methods;
edt_distinct_methods_t _edt_Distinct_Methods;

static struct jtag _Jtag =
{
  0,  // TCK, P4.4 (out) (high)
  0,  // TMS, P4.5 (out) (high)
  0,  // TDI, P4.6 (out) (high)
  0,  // TDO, P4.7 (in)
  0,
  0,
  0, //RST
  0, //TST
  0
};

unsigned short gprotocol_id;
static unsigned short hil_sbw2Speed_;

#pragma inline=forced
void RSTset1()
{
    { (*_Jtag.Out) |= _Jtag.RST; }
    _hil_Delay_1ms(5);
}
#pragma inline=forced
void RSTset0()
{
    { (*_Jtag.Out) &= ~_Jtag.RST; }
     _hil_Delay_1ms(5);
}
#pragma inline=forced
void TSTset1()
{
    { (*_Jtag.Out) |= _Jtag.TST;}
    _hil_Delay_1ms(5);
}
#pragma inline=forced
void TSTset0()
{
    {(*_Jtag.Out) &= ~_Jtag.TST; }
    _hil_Delay_1ms(5);
}
//#pragma inline=forced
void TCLKset1()
{
   _edt_Distinct_Methods.Tclk(1);
}

//#pragma inline=forced
void TCLKset0()
{
   _edt_Distinct_Methods.Tclk(0);
}
#pragma inline=forced
void TCLK()
{
    TCLKset0();
    TCLKset1();
}
/*----------------------------------------------------------------------------
   This function performs a single AD conversion on the selected pin.
   Uses internal reference 2500mV (VR+ = VREF+).
   Arguments: word pinmask (bit position of selected analog input Ax)
   Result:    word (12 bit ADC conversion result)
*/
static float ConvertAD(short channel)
{
    ADC12CTL0  &= ~ADC12ENC;              // Disable conversion, write controls
    ADC12MCTL0  = ADC12SREF_1 + channel;  // select Vref and analog channel Ax
    ADC12CTL0  |= ADC12ENC;               // Enable conversions
    ADC12CTL0  |= ADC12SC;                // Start conversions
    while ((ADC12IFG & BIT0) == 0);       // wait until conversion is done
    return(((float)ADC12MEM0* A_VREFPLUS) / ADC_CONV_RANGE);      // return conversion result from MEM0 value in mv
}

static int SetVCoreUp (unsigned short level)
{
        // Open PMM registers for write access
    PMMCTL0_H = 0xA5;
        // Set SVS/SVM high side new level
    SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
        // Set SVM low side to new level
    SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
        // Wait till SVM is settled
    while ((PMMIFG & SVSMLDLYIFG) == 0);
        // Clear already set flags
    PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
        // Set VCore to new level
    PMMCTL0_L = PMMCOREV0 * level;
        // Wait till new level reached
    if ((PMMIFG & SVMLIFG))
      while ((PMMIFG & SVMLVLRIFG) == 0);
        // Set SVS/SVM low side to new level
    SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
        // Lock PMM registers for write access
    PMMCTL0_H = 0x00;

    return 0;
}

void _hil_Init_Stand_Alone( void )
{
    // Stop watchdog timer to prevent time out reset

    SetVCoreUp(2);
     __delay_cycles(30000);

    //****** WE ARE NOT USING XT1 top source the clock **********

    UCSCTL5 = 0;            // DIVPA, DIVA, DIVS, DIVM -> all direct (DIV=1)
    P5SEL = BIT2+BIT3;      // XT2 SELECT BITS
    UCSCTL6 &=~XT2OFF;      // XT2 ON
    UCSCTL3 = SELREF_2;     // DCO SELECTION

    //Loop until XT1,XT2 & DCO stabilizes
    UCSCTL4 |= SELA__REFOCLK + SELA__XT2CLK; // SELECT ACLK_XT1 AND SMCLK_XT2 TO AVOID OSCILLATOR FAULT FLAG XT1OIFG
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG /*+ XT1HFOFFG*/ + DCOFFG); //Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                                      //Clear fault flags
    }
    while( SFRIFG1 & OFIFG );


    __bis_SR_register(SCG0);    // disable the FLL for config
    UCSCTL1 = DCORSEL_6;       // DCO-freq range up to min 39MHz (must be higher then 18MHz*2 = 36 MHz)
    UCSCTL2 = FLLD0 + 9*FLLN0;  //DCO-DIV/2 + PLL MULTI*(9+1), freq = 10*2 = 20 MHz
    UCSCTL3 = SELREF_5+FLLREFDIV_2; //Reference - XT2-CLK, XT2/2 = 2MHz
    __bic_SR_register(SCG0);  // Enable the FLL control loop after config

    //Loop until XT1,XT2 & DCO stabilizes
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG /*+ XT1HFOFFG*/ + DCOFFG);     //Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                                              //Clear fault flags
    }
    while( SFRIFG1 & OFIFG );
    __delay_cycles(60000);

    UCSCTL4 = SELA__XT2CLK + SELS__DCOCLK + SELM__DCOCLK ;//SELM__DCOCLKDIV; //SELM__DCOCLK;//  + SELM__DCOCLKDIV;

    // Enable Vref=2.5V for ADC
    REFCTL0 |= REFMSTR;
    REFCTL0 |= REFVSEL1;
    REFCTL0 |= REFON;

   // Configure ADC12
   // select channel and do conversion
   ADC12CTL0 = ADC12ON + ADC12SHT02;     	// Turn on ADC12, set sampling time
   ADC12CTL1 = ADC12SHP;                        // Use sampling timer
   ADC12MCTL0  = ADC12SREF_1 + 0;               // select Vref and analog channel A0
   __delay_cycles((15*90));
   ADC12CTL0 |= ADC12ENC;                       // Enable conversion

    // set default debug protocol to JTAG
    gprotocol_id = SPYBIWIRE;
    // set default to TCLK low when doing PSA
    gTclkHighWhilePsa = 0;
    // initialize function pointers to distinct functions
    _hil_SetProtocol(SPYBIWIRE);

    //hil_device_flags_ = 0;
}

static void _hil_initEdtCommenMethods()
{
    _edt_Common_Methods.Init = _hil_Init;

    _edt_Common_Methods.SetVcc = _hil_SetVcc;
    _edt_Common_Methods.GetVcc = _hil_GetVcc;

    _edt_Common_Methods.SetProtocol = _hil_SetProtocol;
    _edt_Common_Methods.SetPsaSetup = _hil_SetPsaSetup;
    _edt_Common_Methods.SetPsaTCLK = _hil_SetPsaTCLK;

    _edt_Common_Methods.Open = _hil_Open;
    _edt_Common_Methods.Close = _hil_Close;

    _edt_Common_Methods.Delay_1us = _hil_Delay_1us;
    _edt_Common_Methods.Delay_1ms = _hil_Delay_1ms;

    _edt_Common_Methods.Loop = 0;

    _edt_Common_Methods.EntrySequences = _hil_EntrySequences;

    _edt_Common_Methods.SetJtagBits = _hil_SetJtagBits;
    _edt_Common_Methods.GetJtagBits = _hil_GetJtagBits;

    _edt_Common_Methods.SetTgtCtrlBits = _hil_SetTgtCtrlBits;
    _edt_Common_Methods.GetTgtCtrlBits = _hil_GetTgtCtrlBits;

    _edt_Common_Methods.GetTestBits = _hil_GetTestBits;
    _edt_Common_Methods.SetTestBits = _hil_SetTestBits;

    _edt_Common_Methods.SetReset = _hil_SetReset;
    _edt_Common_Methods.SetTest  = _hil_SetTest;

    _edt_Common_Methods.SetJtagSpeed = _hil_SetJtagSpeed;

    _edt_Common_Methods.ConfigureSetPc = _hil_ConfigureSetPc;
    _edt_Common_Methods.initTimerB0 = _hil_initTimerB0;

    _edt_Common_Methods.BSL_EntrySequence = _hil_BSL_EntrySequence;
}

void _hil_getEdtDistinct(edt_distinct_methods_t* edt_distinct);
void _hil_getEdtCommen(edt_common_methods_t* edt_commen);

#pragma required=_hil_getEdtDistinct
#pragma required=_hil_getEdtCommen

void _hil_startUp()
{
    _hil_initEdtCommenMethods();
    _edt_Common_Methods.Init();
    return;
}
#pragma location="INFO_C_DISTINCT"
void _hil_getEdtDistinct(edt_distinct_methods_t* edt_distinct)
{
    *edt_distinct = _edt_Distinct_Methods;
    if(edt_distinct!= 0)
    {
        return;
    }
}

#pragma location="INFO_C_COMMEN"
void _hil_getEdtCommen(edt_common_methods_t* edt_commen)
{
    *edt_commen = _edt_Common_Methods;
    if(edt_commen != 0)
    {
        return;
    }
}

void _hil_initTimerB0(void)
{
    // Setup timer_A for hardware delay
    TB0CTL = 0;                                          // STOP Timer
    TB0CTL =  ID__8 +TBSSEL__SMCLK;                      // Timer_B source:SMCLK ,SMCLK/0 = 20
    TB0CCR0 = 0x9cf;                                     // Load CCR0 with delay... (1ms delay)
}

// -----------------------------------------------------------------------------
void _hil_BSL_EntrySequence(void)
{
    // set Default state of RST and TST
    (*_Jtag.Out) |= _Jtag.TST;
    _hil_Delay_1ms(1);
    RSTset1();

    // INIT phase
    TSTset0();
    _hil_Delay_1ms(100);

    RSTset0();    // set RST 0
    _hil_Delay_1ms(100);

    TSTset1();    // set test to 1
    _hil_Delay_1ms(100);

    /*this is the first pulse keep it low for less than 15us */
    (*_Jtag.Out) &= ~_Jtag.TST;
    _hil_Delay_1us(10);
    TSTset1();     // set test 1;

    _hil_Delay_1ms(100);
    RSTset1();     // set RST 1;
    _hil_Delay_1ms(100);
    TSTset0();     // set test 0;
    _hil_Delay_1ms(100);
}

// -----------------------------------------------------------------------------
void hil_initTimerA0(void)
{
    // Setup time_A0 for timestamping
    TA0CTL = MC__STOP;                                  // STOP Timer
    TA0CCR0 = 0xFFFF;                                   // TimerA2 Period
    TA0CTL = ID__8 + TASSEL__SMCLK;                     // Timer_A0 source:SMCLK/8 = 25 / 8 = 3.125 MHz = 1/320ns
    TA0EX0 = 1;                                         // Timer_A0 set additional divider to /2
    TA0CTL |= TACLR + MC__CONTINOUS + TAIE;             // START the timer in free-running mode
}

// -----------------------------------------------------------------------------
void hil_initTimerA2(void)
{
    // Setup timer_A2 for current pulse measurement
    P2DIR &= ~(1<<2);                                   // P2.2 input
    P2SEL |=  (1<<2);                                   // Select pin
    TA2CTL = MC__STOP;                                  // STOP Timer
    TA2CCR0 = 0xFFFF;                                   // TimerA2 Period
    TA2CTL = TASSEL__TACLK + MC_2 + TAIE;               // Timer_A2 source
}

// -----------------------------------------------------------------------------

short _hil_Init( void )
{
    hil_initTimerA0();              // TimeStamp
    hil_initTimerA2();              // Current pulse counter
    _hil_initTimerB0();              // Delay loop timer

    // Enable Vref=2.5V for ADC
    REFCTL0 |= REFMSTR;
    REFCTL0 |= REFVSEL1;
    REFCTL0 |= REFON;

    // Configure ADC12
    // select channel and do conversion
    ADC12CTL0 = ADC12ON + ADC12SHT02;     	// Turn on ADC12, set sampling time
    ADC12CTL1 = ADC12SHP;                        // Use sampling timer
    ADC12MCTL0  = ADC12SREF_1 + 0;               // select Vref and analog channel A0
    __delay_cycles((15*90));
    ADC12CTL0 |= ADC12ENC;                       // Enable conversion

    DMACTL2 = 0;
    DMACTL1 = 0;

    /********************* Temporary Hack *********************/
    // Turn on the power switch
    P2DIR |= (BIT0|BIT1);
    P2OUT |= BIT0;
    /********************* Temporary Hack *********************/

    // set default debug protocol to JTAG
    gprotocol_id = SPYBIWIRE;
    // set default to TCLK low when doing PSA
    gTclkHighWhilePsa = 0;
    // initialize function pointers to distinct functions
    _hil_SetProtocol(SPYBIWIRE);

    return 0;
}

// -----------------------------------------------------------------------------
short _hil_SetVcc(unsigned short Vcc)
{
    if(Vcc)
    { // Turn on the power switch
        P2DIR |= (BIT0|BIT1);
        P2OUT |= BIT0;
    }
    else
    { // Turn off the Power switch
        P2DIR &= ~(BIT0|BIT1);
        P2OUT &= ~(BIT0|BIT1);
        _hil_Release();
    }
    return 0;
}
// -----------------------------------------------------------------------------

// Calculate ExtVcc based on Vref+=2.5V, R27=250kOhm, R28=250kOhm
// Calculate Vcc    based on Vref+=2.5V, R29=500kOhm, R30=500kOhm
// Calculate VBus   based on Vref+=2.5V, R31=250kOhm, R32=150kOhm

#define  R31            250.0f           // 250kOhm
#define  R32            150.0f           // 150kOhm
short _hil_GetVBus(float* VccVBus)
{
    unsigned short i = 0;
    float vbus_mv = 0, vbus_mv0 = 0;
    for (i=0; i<ADC_AVERAGE; i++)
    {
        vbus_mv0 = ConvertAD(A_VBUS);
        vbus_mv0 = (vbus_mv0 * (R31 + R32)) / R32;
        vbus_mv0 = vbus_mv0*1000;

        // Low pass filter
        if (vbus_mv == 0)
        {
            vbus_mv = vbus_mv0;
        }
        vbus_mv = ((vbus_mv0 * 3) + (vbus_mv * 7)) / 10;
    }
    *VccVBus = vbus_mv;
    return 0;
}

#pragma optimize = low
short _hil_GetVcc(double* Vcc, double* ExtVcc)
{
    float VccTmp = 0;
    // check if external target voltage is applied
    //P6DIR |= BIT0;
/*    VccTmp = ConvertAD(A_VCCTARGET);
    VccTmp = (VccTmp * (R27 + R28)) / R28;
    VccTmp = VccTmp * 1000; // change to mV
    if(VccTmp > 600)// more than 600mV
    {
        // if we detect external voltage do not start debugging
        *ExtVcc = VccTmp;
        *Vcc = 0;
    }
    else*/
    //{
        // With the ezFET we do not have the option of external power!
        // if no voltage is applied check if the tool supplies the device with
        // correct voltage 3.6 v
        VccTmp = ((ConvertAD(A_VCCOUT)) * (R29 + R30)) / R30;
        VccTmp = VccTmp * 1000; // change to mV
        *Vcc = VccTmp;
        *ExtVcc = 0;
    //}
    return 0;
}
// -----------------------------------------------------------------------------
short _hil_SetProtocol(unsigned short protocol_id)
{
    short ret_value = 0;

    if(protocol_id == SPYBIWIRE || protocol_id == SPYBIWIREJTAG)
    {
        gprotocol_id = SPYBIWIRE;
        _Jtag = _Jtag_Target;
        initJtagSbw2Dma(_Jtag);
        _hil_2w_ConfigureSpeed_Dma(hil_sbw2Speed_);
    }
    /*else if(protocol_id == SPYBIWIREJTAG) // DEBUG
    {
         gprotocol_id = SPYBIWIREJTAG;
        _Jtag = _Jtag_Target;
        initJtagSbw4(_Jtag);              // DEBUG
    }*/
    else if(protocol_id == SPYBIWIRE_SUBMCU)
    {
        gprotocol_id = 1;
        _Jtag = _Jtag_SubMcu;
        initJtagSbw2Dma(_Jtag);
        _hil_2w_ConfigureSpeed_Dma(SBW400KHz);
    }
    else
    {
        ret_value = -1;
    }
    //if(protocol_id == SPYBIWIRE || protocol_id == SPYBIWIRE_SUBMCU )
    //{
        // load DMA1 with size just default
        DMA1CTL = ( DMADT0 | DMASRCINCR1 | DMASRCINCR0 | DMASRCBYTE | DMADSTBYTE);
        DMA1DA =  (_Jtag.Out); //JTAGOUT;       // set destination address
        DMA2CTL = ( DMADT0 | DMASRCINCR1 | DMASRCINCR0 | DMASRCBYTE | DMADSTBYTE);
        DMA2DA =  (_Jtag.Out); //JTAGOUT;       // set destination address

        _edt_Distinct_Methods.TapReset =            _hil_2w_TapReset_Dma;
        _edt_Distinct_Methods.CheckJtagFuse =       _hil_2w_CheckJtagFuse_Dma;
        _edt_Distinct_Methods.EnumChain =           _hil_2w_EnumChain_Dma;
        _edt_Distinct_Methods.Instr =               _hil_2w_Instr_Dma;
        _edt_Distinct_Methods.SetReg_XBits08 =      _hil_2w_SetReg_XBits08_Dma;
        _edt_Distinct_Methods.SetReg_XBits16 =      _hil_2w_SetReg_XBits16_Dma;
        _edt_Distinct_Methods.SetReg_XBits20 =      _hil_2w_SetReg_XBits20_Dma;
        _edt_Distinct_Methods.SetReg_XBits32 =      _hil_2w_SetReg_XBits32_Dma;
        _edt_Distinct_Methods.SetReg_XBits64 =      _hil_2w_SetReg_XBits64_Dma;
        _edt_Distinct_Methods.SetReg_XBits8_64 =    _hil_2w_SetReg_XBits8_64;
        _edt_Distinct_Methods.Tclk =                _hil_2w_Tclk_Dma;
        _edt_Distinct_Methods.GetPrevInstruction =  _hil_2w_GetPrevInstruction_Dma;

        if(!gTclkHighWhilePsa)
        {
            _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsa_Dma;
        }
        else
        {
            _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsaTclkHigh_Dma;
        }
        _edt_Distinct_Methods.BlowFuse = _hil_2w_BlowFuse_Dma;

        DMA2SA = (unsigned char*)DMA_TMSL_TDIL;
    /*}
    else // DEBUG
    {
        _edt_Distinct_Methods.TapReset = _hil_4w_TapReset;
        _edt_Distinct_Methods.CheckJtagFuse = _hil_4w_CheckJtagFuse;
        _edt_Distinct_Methods.EnumChain = _hil_4w_EnumChain;
        _edt_Distinct_Methods.Instr = _hil_4w_Instr;
        _edt_Distinct_Methods.SetReg_XBits08 = _hil_4w_SetReg_XBits08;
        _edt_Distinct_Methods.SetReg_XBits16 = _hil_4w_SetReg_XBits16;
        _edt_Distinct_Methods.SetReg_XBits20 = _hil_4w_SetReg_XBits20;
        _edt_Distinct_Methods.SetReg_XBits32 = _hil_4w_SetReg_XBits32;
        _edt_Distinct_Methods.SetReg_XBits64 = _hil_4w_SetReg_XBits64;
        _edt_Distinct_Methods.Tclk = _hil_4w_Tclk;
        if(!gTclkHighWhilePsa)
        {
            _edt_Distinct_Methods.StepPsa = _hil_4w_StepPsa;
        }
        else
        {
            _edt_Distinct_Methods.StepPsa = _hil_4w_StepPsaTclkHigh;
        }
        _edt_Distinct_Methods.BlowFuse = _hil_4w_BlowFuse;

    }// DEBUG*/

    return(ret_value);
}

void _hil_SetPsaSetup(unsigned short enhancedPsa)
{
    if(enhancedPsa)
    {
        _edt_Distinct_Methods.SetupPsa = _hil_EnhancedPsaSetup;
        _edt_Distinct_Methods.EndPsa = _hil_EnhancedPsaEnd;
    }
    else
    {
        _edt_Distinct_Methods.SetupPsa = _hil_PsaSetup;
        _edt_Distinct_Methods.EndPsa = _hil_PsaEnd;
    }
}

void _hil_SetPsaTCLK(unsigned short tclkValue)
{
    if(tclkValue)
    {
        gTclkHighWhilePsa = 1;
    }
    else
    {
        gTclkHighWhilePsa = 0;
    }

    if(gprotocol_id == SPYBIWIRE)
    {
        if(gTclkHighWhilePsa)
        {
            _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsaTclkHigh_Dma;
        }
        else
        {
            _edt_Distinct_Methods.StepPsa = _hil_2w_StepPsa_Dma;
        }
    }
}

void _hil_Release(void)
{
    (*_Jtag.Out) |= _Jtag.RST;
    (*_Jtag.Out) &= ~_Jtag.TST;
    _hil_Delay_1ms(50);

    (*_Jtag.DIRECTION) &= ( ~_Jtag.TST );
    (*_Jtag.DIRECTION) &= ( ~_Jtag.RST );

    _hil_Delay_1ms(5);
}

#define qDriveJTAG(){    (*_Jtag.Out) |= (_Jtag.TCK + _Jtag.TMS + _Jtag.TDI + _Jtag.RST + _Jtag.TST);\
                         (*_Jtag.DIRECTION) |= (_Jtag.TCK + _Jtag.TMS + _Jtag.TDI);                  \
                         (*_Jtag.DIRECTION) |= (_Jtag.TST + _Jtag.RST);                              \
                         (*_Jtag.DIRECTION) &= (~_Jtag.TDO);                                         \
                     }

#define qDriveSbw(){    (*_Jtag.Out) |= _Jtag.RST;                                                   \
                        (*_Jtag.Out) &= ~_Jtag.TST;                                                  \
                        (*_Jtag.DIRECTION) |= ( _Jtag.TST +  _Jtag.RST);                             \
                    }

void _hil_EntrySequences(unsigned char states)
{
    TSTset0();    //1
    _hil_Delay_1ms(4); // reset TEST logic

    if(states == RSTLOW_SBW || states == RSTLOW_JTAG)
    {
        RSTset0();    //2
    }
    else
    {
        RSTset1();    //2
    }

    TSTset1();    //3
    _hil_Delay_1ms(20); // activate TEST logic

    // phase 1
    if(states == RSTLOW_JTAG || states == RSTHIGH_JTAG)
    {
        RSTset0();    //4
    }
    else
    {
        RSTset1();    //4
    }
    _hil_Delay_1us(60);

    // phase 2 -> TEST pin to 0, no change on RST pin
    if(states == RSTLOW_SBW || states == RSTHIGH_SBW)
    { // for Spy-Bi-Wire
        _DINT_FET();
        (*_Jtag.Out) &= ~_Jtag.TST; //5
    }
    else
    {   // for 4-wire JTAG clear Test pin
        TSTset0();  //5
    }

    // phase 3
    if(states == RSTLOW_JTAG)
    {
        RSTset0();  //6
    }
    _hil_Delay_1us(1);

    // phase 4 -> TEST pin to 1, no change on RST pin
    if(states == RSTLOW_SBW || states == RSTHIGH_SBW)
    { // for Spy-Bi-Wire
        (*_Jtag.Out) |= _Jtag.TST;  //7
        _EINT_FET();
    }
    else
    { // for 4-wire JTAG
        TSTset1();//7
    }
    _hil_Delay_1us(60);

    // phase 5
    if(states == RSTHIGH_JTAG)
    {
         RSTset1();
    }
    _hil_Delay_1ms(5);
}

// -----------------------------------------------------------------------------
void _hil_PsaSetup(unsigned long StartAddr)
{
    data_16bit
    TCLKset1();
    SetReg_16Bits_(MOV_IMM_PC)
    TCLKset0();
    TCLKset1();
    SetReg_16Bits_(StartAddr - 2)
    TCLKset0();
    TCLKset1();
    TCLKset0();
    TCLKset1();
    TCLKset0();
    addr_capture
    SetReg_16Bits_(0x0000);
}

// -----------------------------------------------------------------------------
void _hil_EnhancedPsaSetup(unsigned long StartAddr )
{
    SetPc(StartAddr - 4);
    halt_cpu;
    TCLKset0();
    data_16bit
    SetReg_16Bits_(StartAddr - 2)
}

// -----------------------------------------------------------------------------
void _hil_PsaEnd(void)
{
    // Intentionally does nothing
}

// -----------------------------------------------------------------------------
void _hil_EnhancedPsaEnd(void)
{
    decl_out
    decl_isInstrLoad

    release_cpu;
    isInstrLoad;
}

// -----------------------------------------------------------------------------
static void _hil_Connect(unsigned char state)
{
    if(state == RSTHIGH)
    {
        if(gprotocol_id == SPYBIWIRE)
        {
            qDriveSbw();
            _hil_Delay_1ms(1);
            _hil_EntrySequences(RSTHIGH_SBW);
        }
        else
        {
            qDriveSbw();
            _hil_Delay_1ms(1);
            if(gprotocol_id == SPYBIWIREJTAG)
            {
                _hil_EntrySequences(RSTHIGH_JTAG);
            }
            else
            {
                TSTset1();
            }
        }
    }
    else // state  == RSTLOW
    {
        if(gprotocol_id == SPYBIWIRE)
        {
            qDriveSbw();
            _hil_Delay_1ms(1);
            _hil_EntrySequences(RSTLOW_SBW);
        }
        else
        {
            qDriveSbw();
            _hil_Delay_1ms(1);
            if(gprotocol_id == SPYBIWIREJTAG)
            {
                _hil_EntrySequences(RSTLOW_JTAG);
            }
            else
            {
                TSTset1();
            }
        }
    }
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

// -----------------------------------------------------------------------------

void _hil_SetJtagSpeed(unsigned short jtagSpeed, unsigned short sbwSpeed)
{
    if(sbwSpeed)
    {
        hil_sbw2Speed_ =  sbwSpeed;
       _hil_2w_ConfigureSpeed_Dma(sbwSpeed);
    }
}

// -----------------------------------------------------------------------------

void _hil_ConfigureSetPc (unsigned short PCclockBeforeCapture)
{
    setPCclockBeforeCapture = PCclockBeforeCapture;
}


// -----------------------------------------------------------------------------
void SetVFuse(void)
{
    return;
}

// -----------------------------------------------------------------------------
void SetVpp(long voltage)
{
    return;
}

// -----------------------------------------------------------------------------
void testVpp(unsigned char mode)
{
    return;
}

// -----------------------------------------------------------------------------
void _hil_SetJtagBits(unsigned char output)
{
    //Only RST pin exists and won't be set through this function
    return;
}

// -----------------------------------------------------------------------------
unsigned char _hil_GetJtagBits(void)
{
    return ((*_Jtag.Out) & _Jtag.RST) ? 0x40 : 0x00;
}

// -----------------------------------------------------------------------------
void _hil_SetTgtCtrlBits(unsigned char output)
{
    (*_Jtag.Out) = output;
}

// -----------------------------------------------------------------------------
unsigned char _hil_GetTgtCtrlBits(void)
{
    return (*_Jtag.Out);
}

// -----------------------------------------------------------------------------
void _hil_SetTestBits(unsigned char output)
{
    //Only TST pin exists and won't be set through this function
    return;
}

// -----------------------------------------------------------------------------
unsigned char _hil_GetTestBits(void)
{
    return ((*_Jtag.Out) & _Jtag.TST) ? 0x01 : 0x00;
}

// -----------------------------------------------------------------------------
void _hil_SetReset(unsigned char value)
{
    if(value)
    {
        RSTset1();
    }
    else
    {
        RSTset0();
    }
}

// -----------------------------------------------------------------------------
void _hil_SetTest(unsigned char value)
{
    if(value)
    {
        TSTset1();
    }
    else
    {
        TSTset0();
    }
}
/* EOF */
