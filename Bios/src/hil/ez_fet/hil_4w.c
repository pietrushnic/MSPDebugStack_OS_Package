/*
 * hil_4w.c
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
//! \file hil_4w.c
//! \brief 
//!

//#include "hw_compiler_specific.h"
//#include "HalGlobalVars.h"
#include "arch.h"
#include "hilDelays.h"
#include "JTAG_defs.h"
//#include "hil.c"

//! \ingroup MODULHIL
//! \file hil_4w.c
//! \brief 
//!

#define uController_uif

extern void TCLKset1();
extern void TCLKset0(); 

static struct jtag _Jtag = {0};
//#define JTAG_DELAY   { _NOP();}
void _hil_4w_Tclk(unsigned char state);

// 4-wire JTAG: low level signal access
#pragma inline=forced 
void TMSset1()    
{ (*_Jtag.Out) |=  _Jtag.TMS;}

#pragma inline=forced 
void TMSset0()    
{ (*_Jtag.Out) &= ~_Jtag.TMS;}

#pragma inline=forced 
void TDIset1()    
{ (*_Jtag.Out) |=  _Jtag.TDI;}

#pragma inline=forced 
void TDIset0()    
{ (*_Jtag.Out) &= ~_Jtag.TDI;}

#pragma inline=forced 
void TCKset1()    
{ (*_Jtag.Out) |=  _Jtag.TCK;}

#pragma inline=forced 
static void TCKset0()    
{ (*_Jtag.Out) &= ~_Jtag.TCK;}

#pragma inline=forced 
void TDIset1TMSset1()    
{ (*_Jtag.Out) |=  _Jtag.TDI | _Jtag.TMS; }

#pragma inline=forced 
void TDIset0TMSset1()   
{ (*_Jtag.Out) &= ~_Jtag.TDI; *_Jtag.Out |= _Jtag.TMS;}

#pragma inline=forced 
unsigned char StoreTTDI() 
{
    return *_Jtag.Out;
}
#pragma inline=forced 
void RestoreTTDI(unsigned long long x)  
{
  if(x &_Jtag.TDI)
  {
    *_Jtag.Out |= _Jtag.TDI; 
    //JTAG_DELAY;
  }
  else
  {
    *_Jtag.Out &=  ~_Jtag.TDI;
  }
}
#pragma inline=forced 
unsigned char ScanTDO()      
{
    if((*_Jtag.In   &  _Jtag.TDO)!= 0)
    { 
        return 1;
    }
    else
    {
        return 0;
    }
}

void initJtagSbw4(struct jtag tmp)
{  
    _Jtag = tmp;  
}
#pragma inline=forced 
unsigned long long streamSbw4WireShift(unsigned long long Data, short Bits)
{   
    unsigned short tclk = 0; 
    unsigned long long TDOvalue = 0x0000000000000000;
    unsigned long long MSB = 0x0000000000000000;
    tclk = StoreTTDI();  // Store TCLK state;
        
    // JTAG FSM state = Select DR-Scan
    TMSset0();
    TCKset0();
    TCKset1();
    // JTAG FSM state = Capture-DR
    TCKset0();
    TCKset1();    
   
    switch(Bits)
    {
        case F_BYTE: MSB = 0x00000080;
            break;
        case F_WORD: MSB = 0x00008000;
            break;
        case F_ADDR: MSB = 0x00080000;
            break;
        case F_LONG: MSB = 0x80000000;
            break;
        case F_LONG_LONG: MSB = 0x8000000000000000;
            break;
        default: // this is an unsupported format, function will just return 0
            return TDOvalue;
    }    
    do
    {
        if((Data & MSB) == 0)
        {
            TDIset0();
        }
        else
        {
            TDIset1(); 
        }
        
        if ((MSB & 1) == 1)
        {                       // Last bit requires TMS=1
            TMSset1(); 
        }
        
        TCKset0();
        TCKset1(); 
          
        TDOvalue <<= 1;          // TDO could be any port pin
        
        if (ScanTDO() != 0)
        {
            TDOvalue++;	
        }
    }
    while(MSB >>= 1);
    
    // common exit
    RestoreTTDI(tclk);                  // restore TCLK state

    // JTAG FSM = Exit-DR
    //TMSset1
    TCKset0();
    TCKset1();
      
    // JTAG FSM = Update-DR
    TMSset1();
    TCKset0();
    TCKset1();
    
    if(Bits == F_ADDR)
    {
        TDOvalue = ((TDOvalue << 16) + (TDOvalue >> 4)) & 0x000FFFFF;
    }    
    // JTAG FSM = Run-Test/Idle
     return(TDOvalue);  
}
#pragma inline=forced 
///#pragma optimize = low
unsigned long long Sbw4WireShift(unsigned long long Data, unsigned short Bits)
{   
    unsigned long long tclk = 0; 
    unsigned long long TDOvalue = 0x0000000000000000;
    unsigned long long MSB = 0x0000000000000000;
    tclk = StoreTTDI();  // Store TCLK state;
    
    switch(Bits)
    {
        case F_BYTE: MSB = 0x00000080;
            break;
        case F_WORD: MSB = 0x00008000;
            break;
        case F_ADDR: MSB = 0x00080000;
            break;
        case F_LONG: MSB = 0x80000000;
            break;
        case F_LONG_LONG: MSB = 0x8000000000000000;                               
            break;
        default: // this is an unsupported format, function will just return 0
            return TDOvalue;
    }    
    do
    {
        if((Data & MSB) == 0)
        {
            TDIset0();
        }
        else
        {
            TDIset1(); 
        }
        
        if ((MSB & 1) == 1)
        {                       // Last bit requires TMS=1
            TMSset1(); 
        }
        
        TCKset0();
        TCKset1(); 
          
        TDOvalue <<= 1;          // TDO could be any port pin
        
        if (ScanTDO() != 0)
        {
            TDOvalue++;	
        }
    }
    while(MSB >>= 1);
    
    // common exit
    RestoreTTDI(tclk);                  // restore TCLK state

    // JTAG FSM = Exit-DR
    TMSset1();
    TCKset0();
    TCKset1();
      
    // JTAG FSM = Update-DR
    TMSset0();
    TCKset0();
    TCKset1();
    
    if(Bits == F_ADDR)
    {
        TDOvalue = ((TDOvalue << 16) + (TDOvalue >> 4)) & 0x000FFFFF;
    }    
    // JTAG FSM = Run-Test/Idle
     return(TDOvalue);  
}

// -----------------------------------------------------------------------------
short _hil_4w_CheckJtagFuse(void)
{
    // perform a JTAG fuse check
    TMSset1();
    TMSset0();
    _hil_Delay_1us(15);
    TMSset1();
    TMSset0();
    _hil_Delay_1us(15);
    TMSset1();
    return 0;
}

// -----------------------------------------------------------------------------
short _hil_4w_TapReset(void)
{
    // Reset TAP Controller State Machine
    // Set default state for JTAG signals (TDI = TMS = TCK = 1)
    TDIset1();
    TMSset1();
    TCKset1();
    // Clock TCK six (6) times
    TCKset0();
    TCKset1();
    TCKset0();
    TCKset1();
    TCKset0();
    TCKset1();
    TCKset0();
    TCKset1();
    TCKset0();
    TCKset1();
    TCKset0();
    TCKset1();
    // TAP Controller State Machine is now in "Test-Logic Reset" state
    // Clock TCK one more time with TMS = 0
    TMSset0();
    TCKset0();
    TCKset1();
    return 0;
}



// defined in hil.c
//extern void testVpp(unsigned char mode);
//extern void SetVpp(long voltage);

unsigned short _hil_4w_EnumChain(void)
{
    /*unsigned long TDOval = 0x00000000;    // Initialize shifted-in word
    unsigned long MSB = 0x80000000;
    unsigned long LSB = 0x00000001;
    unsigned long DataIn = Invalid_Manufactor_IdCode | LSB;
    unsigned char i = 0;
    unsigned char detIdCode = 0;
    unsigned short numOfDevices = 0;
    
    _hil_4w_TapReset();
    
    // JTAG FSM state = Run-Test/Idle
    TMSset1;
    _hil_Delay_1us(10);
    TCKset0;
    _hil_Delay_1us(10);
    TCKset1;
    _hil_Delay_1us(10);
    // JTAG FSM state = Select DR-Scan
    TMSset0;
    _hil_Delay_1us(10);
    TCKset0;
    _hil_Delay_1us(10);
    TCKset1;
    _hil_Delay_1us(10);
    // JTAG FSM state = Capture-DR
    TCKset0;
    _hil_Delay_1us(10);
    TCKset1;
    _hil_Delay_1us(10);
    // JTAG FSM state = Shift-IR
  
    while(1)
    {
        if((DataIn & LSB) == 0)
        {
            TDIset0;
        }
        else
        {
            TDIset1;
        }
        DataIn >>= 1;
        TCKset0;
        _hil_Delay_1us(10);
        TCKset1;
        TDOval >>= 1;			    // TDO could be any port pin
        if (ScanTDO())
        {
            TDOval |= MSB;
            if(0 == detIdCode)                // Test if LSB of IdCode
            {
                i = 0;
                detIdCode = 1;
            }
        }
        else if(0 == detIdCode)
        {
            numOfDevices++;
        }
    
        i += detIdCode;
        if(32 == i)
        {
            detIdCode = 0;
            i = 0;
            if(Invalid_Manufactor_IdCode == (TDOval & Mask_Manufactor_IdCode))
            {
                // End of chain detected
                break; 
            }
            // Device with valid IdCode detected
            numOfDevices++;
        }
        if(0xFF == numOfDevices)
        {
            // Only 254 devices are supported
            // Probably hardware connection is broken
            numOfDevices = 0;
            break;
        }
    }  
    
    TMSset1;
    _hil_Delay_1us(10);
    TCKset0;
    _hil_Delay_1us(10);
    TCKset1;  
    // JTAG FSM = Exit-DR
    TCKset0;
    _hil_Delay_1us(10);
    TCKset1;
    _hil_Delay_1us(10);
    // JTAG FSM = Update-DR
    TMSset0;
    _hil_Delay_1us(10);
    TCKset0;
    _hil_Delay_1us(10);
    TCKset1;
    _hil_Delay_1us(10);
    // JTAG FSM = Run-Test/Idle
    TMSset1; // to save power during debugging
    _hil_Delay_1us(10);*/
    return 1;
}


unsigned char _hil_4w_Instr(unsigned char Instruction)
{
    // JTAG FSM state = Run-Test/Idle
    // JTAG FSM state = Select DR-Scan
    TMSset1();
    TCKset0();
    TCKset1();
    
    // JTAG FSM state = Select IR-Scan
    TCKset0();
    TCKset1();
    
    // JTAG FSM state = Capture-IR
    TMSset0();
    TCKset0();
    TCKset1();
    
    // JTAG FSM state = Shift-IR, Shift in TDI (8-bit)
    TCKset0();
    TCKset1();
    
    return(Sbw4WireShift(Instruction,F_BYTE));
}

unsigned char _hil_4w_SetReg_XBits08(unsigned char Data)
{
    // JTAG FSM state = Run-Test/Idle
    TMSset1();
    TCKset0();
    TCKset1();

    // JTAG FSM state = Select DR-Scan
    TMSset0();
    TCKset0();
    TCKset1();
    // JTAG FSM state = Capture-DR
    TCKset0();
    TCKset1();
    // JTAG FSM state = Shift-DR, Shift in TDI (8-bit)
    return(Sbw4WireShift(Data,F_BYTE));
}
unsigned short _hil_4w_SetReg_XBits16(unsigned short Data)
{
    // JTAG FSM state = Run-Test/Idle
    TMSset1();
    TCKset0();
    TCKset1();

    // JTAG FSM state = Select DR-Scan
    TMSset0();
    TCKset0();
    TCKset1();
    
    // JTAG FSM state = Capture-DR
    TCKset0();
    TCKset1();
    // JTAG FSM state = Shift-DR, Shift in TDI (16-bit)
    return(Sbw4WireShift(Data,F_WORD)); 
}
unsigned long _hil_4w_SetReg_XBits20(unsigned long Data)
{
        // JTAG FSM state = Run-Test/Idle
    TMSset1();
    TCKset0();
    TCKset1();

    // JTAG FSM state = Select DR-Scan
    TMSset0();
    TCKset0();
    TCKset1();
    // JTAG FSM state = Capture-DR
    TCKset0();
    TCKset1();
    // JTAG FSM state = Shift-DR, Shift in TDI (20-bit)
    return(Sbw4WireShift(Data,F_ADDR)); 
}

// -----------------------------------------------------------------------------
unsigned long _hil_4w_SetReg_XBits32(unsigned long Data)
{
            // JTAG FSM state = Run-Test/Idle
    TMSset1();
    TCKset0();
    TCKset1();

    // JTAG FSM state = Select DR-Scan
    TMSset0();
    TCKset0();
    TCKset1();
    // JTAG FSM state = Capture-DR
    TCKset0();
    TCKset1();
    // JTAG FSM state = Shift-DR, Shift in TDI (20-bit)
    return(Sbw4WireShift(Data,F_LONG)); 
}
// -----------------------------------------------------------------------------

unsigned long long _hil_4w_SetReg_XBits64(unsigned long long Data)
{
           // JTAG FSM state = Run-Test/Idle
    TMSset1();
    TCKset0();
    TCKset1();

    // JTAG FSM state = Select DR-Scan
    TMSset0();
    TCKset0();
    TCKset1();
    // JTAG FSM state = Capture-DR
    TCKset0();
    TCKset1();
    // JTAG FSM state = Shift-DR, Shift in TDI (20-bit)
    return(Sbw4WireShift(Data,F_LONG_LONG)); 
}
// -----------------------------------------------------------------------------
void _hil_4w_Tclk(unsigned char state)
{
    if(state)
    {
        TDIset1();
    }
    else
    {
        TDIset0();
    }
}

// -----------------------------------------------------------------------------
void _hil_4w_StepPsa(unsigned long length)
{
    while(length-- > 0)
    {
        TCLKset1(); _NOP();
        TCLKset0(); _NOP();

        TCKset0(); _NOP();
        TMSset1(); _NOP();
        TCKset1(); _NOP(); // select DR scan
        TCKset0(); _NOP();
        TMSset0(); _NOP();
        
        TCKset1(); _NOP(); // capture DR
        TCKset0(); _NOP();
        TCKset1(); _NOP(); // shift DR
        TCKset0(); _NOP();
        
        TMSset1(); _NOP();
        TCKset1(); _NOP();// exit DR
        TCKset0(); _NOP();
        
        // Set JTAG FSM back into Run-Test/Idle
        TCKset1(); _NOP();
        TMSset0(); _NOP();
        TCKset0(); _NOP();
        TCKset1(); _NOP();
        _NOP(); _NOP(); _NOP(); _NOP(); _NOP();
    }
}
// -----------------------------------------------------------------------------
void _hil_4w_StepPsaTclkHigh(unsigned long length)
{
    while(length--)
    {
        TCLKset1(); _NOP();

        TCKset0(); _NOP();
        TMSset1(); _NOP();
        TCKset1(); _NOP();// select DR scan
        TCKset0(); _NOP();
        TMSset0(); _NOP();
        
        TCKset1(); _NOP();// capture DR
        TCKset0(); _NOP();
        TCKset1(); _NOP();// shift DR
        TCKset0(); _NOP();
        
        TMSset1(); _NOP();
        TCKset1(); _NOP(); // exit DR
        TCKset0(); _NOP();
        
        // Set JTAG FSM back into Run-Test/Idle
        TCKset1(); _NOP();
        TMSset0(); _NOP();
        TCKset0(); _NOP();
        TCKset1(); _NOP();

        _NOP(); _NOP(); _NOP(); _NOP(); _NOP();
        TCLKset0(); _NOP();
    }
}

// -----------------------------------------------------------------------------
short _hil_4w_BlowFuse(unsigned char targetHasTestVpp)
{
    /*if(!targetHasTestVpp) 
    {
        cntrl_sig_16bit;
        SetReg_16Bits_(0x7401); // TDOs get TDI functionality (internal)
        testVpp(0);             // Switchs also TDO functionality to TDI.
    }
    EDT_Instr(IR_PREPARE_BLOW);                     // initialize fuse blowing
    _hil_Delay_1ms(1);
    
    SetVpp(1);
    EDT_Instr(IR_EX_BLOW);                          // execute fuse blowing
    _hil_Delay_1ms(1);
    SetVpp(0);                                       // switch VPP off
    
    testVpp(1);                                      // Restore the normal function of TDO and TDI (within the interface).

    // now perform a BOR via JTAG - we loose control of the device then...
    EDT_Instr(IR_TEST_REG);
    EDT_SetReg_XBits32(0x00000200);    */
    
    return 0;
}
/* EOF */
