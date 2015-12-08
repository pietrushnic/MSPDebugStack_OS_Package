/*
 * hil_generic.c
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

#include "hilFpgaAccess.h"
#include "archFpga.h"
#include "arch.h"
#include "JTAG_defs.h"
#include "hilDelays.h"

unsigned char _hil_generic432_Instr_4(unsigned char instruction)
{
    unsigned short retVal = 0, value = 0, instruction_ = 0;
    unsigned short bitCounter = 0;

    for(bitCounter = 0; bitCounter < 4; ++bitCounter)
    {
        instruction_ <<= 1;
        instruction_ |= (instruction >> bitCounter) & 1;
    }

    hil_fpga_write_cmd_data0(FPGA_CMD_IR4_RD, instruction_);
    hil_fpga_read_data1(1, &retVal);

    for(bitCounter = 0; bitCounter < 4; ++bitCounter)
    {
        value <<= 1;
        value |= (retVal >> bitCounter) & 1;
    }
    return (unsigned char)value;
}

unsigned char _hil_generic432_SetReg_XBits08(unsigned char data)
{
    unsigned short retVal, value = 0,  data_ = 0;
    unsigned short bitCounter;

    for(bitCounter = 0; bitCounter < 8; ++bitCounter)
    {
        data_ <<= 1;
        data_ |= (data >> bitCounter) & 1;
    }

    hil_fpga_write_cmd_data0(FPGA_CMD_DR8_RD, data_);
    hil_fpga_read_data1(1, &retVal);

    for(bitCounter = 0; bitCounter < 8; ++bitCounter)
    {
        value <<= 1;
        value |= (retVal >> bitCounter) & 1;
    }
    return (unsigned char)value;
}

unsigned short _hil_generic432_SetReg_XBits16(unsigned short data)
{
    unsigned long retVal, value = 0,  data_ = 0;
    unsigned short bitCounter;

    for(bitCounter = 0; bitCounter < 16; ++bitCounter)
    {
        data_ <<= 1;
        data_ |= (data >> bitCounter) & 1;
    }

    hil_fpga_write_cmd_data0_data1_count(FPGA_CMD_DRX_RD, 16, (unsigned short*)&data_, 1);
    hil_fpga_read_data1(2, (unsigned short*)&retVal);

    for(bitCounter = 0; bitCounter < 16; ++bitCounter)
    {
        value <<= 1;
        value |= (retVal >> bitCounter) & 1;
    }
    return value;
}

// -----------------------------------------------------------------------------
unsigned long _hil_generic432_SetReg_XBits32(unsigned long data)
{
    unsigned long retVal, value = 0,  data_ = 0;
    unsigned short bitCounter;

    for(bitCounter = 0; bitCounter < 32; ++bitCounter)
    {
        data_ <<= 1;
        data_ |= (data >> bitCounter) & 1;
    }

    hil_fpga_write_cmd_data0_data1_count(FPGA_CMD_DRX_RD, 32, (unsigned short*)&data_, 2);
    hil_fpga_read_data1(2, (unsigned short*)&retVal);

    for(bitCounter = 0; bitCounter < 32; ++bitCounter)
    {
        value <<= 1;
        value |= (retVal >> bitCounter) & 1;
    }
    return value;
}

// -----------------------------------------------------------------------------
unsigned long long _hil_generic432_SetReg_XBits64(unsigned long long data)
{
    unsigned long long retVal, value = 0,  data_ = 0;
    unsigned short bitCounter;

    for(bitCounter = 0; bitCounter < 64; ++bitCounter)
    {
        data_ <<= 1;
        data_ |= (data >> bitCounter) & 1;
    }

    hil_fpga_write_cmd_data0_data1_count(FPGA_CMD_DRX_RD, 64, (unsigned short*)&data_, 4);
    hil_fpga_read_data1(4, (unsigned short*)&retVal);

    for(bitCounter = 0; bitCounter < 64; ++bitCounter)
    {
        value <<= 1;
        value |= (retVal >> bitCounter) & 1;
    }

    return value;
}

unsigned long long _hil_generic432_SetReg_XBits(unsigned long long data, unsigned short count)
{
    unsigned long long retVal = 0, value = 0,  data_ =0;
    unsigned short words = (count + 15) / 16;
    unsigned short bitCounter;

    for(bitCounter = 0; bitCounter < count; ++bitCounter)
    {
        data_ <<= 1;
        data_ |= (data >> bitCounter) & 1;
    }

    hil_fpga_write_cmd_data0_data1_count(FPGA_CMD_DRX_RD, count, (unsigned short*)&data_, words);
    hil_fpga_read_data1(words, (unsigned short*)&retVal);

    for(bitCounter = 0; bitCounter < count; ++bitCounter)
    {
        value <<= 1;
        value |= (retVal >> bitCounter) & 1;
    }
    return value;
}
