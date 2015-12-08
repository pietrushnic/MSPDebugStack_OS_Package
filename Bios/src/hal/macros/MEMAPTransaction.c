/**
* \ingroup MODULMACROS
*
* \file MEMAPTransaction.c
*
* \brief <FILEBRIEF>
*
*/
/*
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

#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "stream.h"
#include "JTAG_defs.h"
#include "error_def.h"
#include <string.h>

/**
  MEMAPTransaction
  Performs a memory transaction (Read/Write) on the specified port
  PortNum<8>: selects the port number
  RnW<8>: selects whether to perform a Read or a Write trasaction
  Datawidth<16>: Indicates the width of the data transaction (8-/16-/32-bits)
  Address<32>: Destination address
  Count<32>: The number of bytes to write
*/
#ifdef MSP_FET
    extern unsigned long cswValues[4];
#endif

/// \brief Low-level write function with timeout
/// \return 0 if timeout occured and non-zero if success
unsigned char writeLowLevel(unsigned char address, unsigned long dataIn)
{
    unsigned char retry = MAX_RETRY;
    unsigned char ack = 0;
    unsigned long long dataIn64 = ((unsigned long long)dataIn) << 3;

    do
    {
        ack = SetReg_XBits(dataIn64 | (address >> 1) | WRITE, 35) & 0x7;
    } while((ack != ACK) && (--retry));

    return retry;
}
/// \brief Low-level read function with timeout
/// \return 0 if timeout occured and non-zero if success
unsigned char readLowLevel(unsigned char address, unsigned long *dataInOut)
{
    unsigned char retry = MAX_RETRY;
    unsigned long long dataIn64 = ((unsigned long long)*dataInOut) << 3;
    unsigned long long dataOut64;

    do
    {
        dataOut64 = SetReg_XBits(dataIn64 | (address >> 1) | READ, 35);
    } while((dataOut64 & 0x7 != ACK) && (--retry));

    *dataInOut = dataOut64 >> 3;

    return retry;
}

/// \brief MACRO function which does low-level access and returns -1 on timeout
#define WRITE_AND_RETURN_ON_TIMEOUT(addr, data) \
    do                                          \
    {                                           \
        if(!writeLowLevel((addr), (data)))      \
        {                                       \
            return -1;                          \
        }                                       \
    } while(0)

/// \brief MACRO function which does low-level access and returns -1 on timeout
#define READ_AND_RETURN_ON_TIMEOUT(addr, data) \
    do                                         \
    {                                          \
        if(!readLowLevel((addr), (data)))      \
        {                                      \
            return -1;                         \
        }                                      \
    } while(0)

/// \brief Stream the correct data size back to the DLL
#define STREAM_DATA(data)                                     \
    do                                                        \
    {                                                         \
        data >>= (address & 0x3) * 8;                         \
        switch(dataWidth)                                     \
        {                                                     \
        case AP_CSW_SIZE_8BIT:                                \
            STREAM_put_byte((unsigned char)data);             \
            break;                                            \
        case AP_CSW_SIZE_16BIT:                               \
            STREAM_put_word((unsigned short)data);            \
            break;                                            \
        case AP_CSW_SIZE_32BIT:                               \
            STREAM_put_long((unsigned long)data);             \
            break;                                            \
        }                                                     \
    } while(0)


HAL_FUNCTION(_hal_MEMAPTransaction)
{
    short retVal = 0;
 #ifdef MSP_FET
    static unsigned long long apsel;
    static unsigned short rnw;
    static unsigned short dataWidth;
    static unsigned long address;
    static unsigned long count;

    unsigned char *pBuf;
    unsigned short sizeOfBuf;

    unsigned long data;

    if(flags & MESSAGE_NEW_MSG)
    {   // Do initial setup
        apsel = 0ull;
        if(STREAM_get_word((unsigned short *)&apsel) == -1)
        {
            return -1;
        }
        if(STREAM_get_word(&rnw) == -1)
        {
            return -1;
        }
        if(STREAM_get_word(&dataWidth) == -1)
        {
            return -1;
        }
        if(STREAM_get_long(&address) == -1)
        {
            return -1;
        }
        if(STREAM_get_long(&count) == -1)
        {
            return -1;
        }
        if(!count)
        { // Nothing to do, just return without failure
            return 0;
        }

        apsel <<= 24;
        count >>= dataWidth; // Calculate the number of transactions needed

        // Write SELECT register
        IHIL_Instr4(IR4_DPACC);
        WRITE_AND_RETURN_ON_TIMEOUT(DP_SELECT, apsel);

        // Write CSW register
        IHIL_Instr4(IR4_APACC);
        data = cswValues[apsel & 0x3] | ((count == 1) ? AP_CSW_ADDRINC_OFF : AP_CSW_ADDRINC_SINGLE) | dataWidth;
        WRITE_AND_RETURN_ON_TIMEOUT(AP_CSW, data);

        // Write TAR register
        WRITE_AND_RETURN_ON_TIMEOUT(AP_TAR, address);
    }

    // Now that the setup is done, do the actual data transfer
    if(rnw == WRITE)
    {
        STREAM_get_buffer((void **)&pBuf,&sizeOfBuf);

        while(count && sizeOfBuf)
        {
            data = 0;
            memcpy(&data, pBuf, (1 << dataWidth)); // Create copy of data
            data <<= (address & 0x3) * 8;  // Move to correct byte lane

            WRITE_AND_RETURN_ON_TIMEOUT(AP_DRW, data);

            pBuf += (1 << dataWidth);
            address += (1 << dataWidth);
            if(sizeOfBuf >= (1 << dataWidth))
            {
                sizeOfBuf -= (1 << dataWidth);
            }
            else
            {
                sizeOfBuf = 0;
            }
            --count;

            // Check for address wrapping
            if(count && !(address & 0x000003FFul))
            { // Write TAR register
                WRITE_AND_RETURN_ON_TIMEOUT(AP_TAR, address);
            }
        }
    }
    else
    { // READ
        // Initiate read
        READ_AND_RETURN_ON_TIMEOUT(AP_DRW,&data);

        while(--count)
        {
            READ_AND_RETURN_ON_TIMEOUT(AP_DRW,&data);
            STREAM_DATA(data);

            // Check for address wrapping
            if(count)
            {
                address += (1 << dataWidth);
                if(!(address & 0x000003FFul))
                { // Write TAR register
                    WRITE_AND_RETURN_ON_TIMEOUT(AP_TAR, address);
                    READ_AND_RETURN_ON_TIMEOUT(AP_DRW,&data);
                }
            }
        }
        IHIL_Instr4(IR4_DPACC);
        READ_AND_RETURN_ON_TIMEOUT(DP_RDBUFF,&data);
        STREAM_DATA(data);
    }

    if(count)
    { // More data expected
        retVal = 1;
    }
    else
    { // Check for errors
        IHIL_Write_Read_Dp(DP_CTRL_STAT, &data, READ);

        // This is a temporary work-around, the lower level function must be fixed!
        IHIL_Write_Read_Dp(DP_RDBUFF, &data, READ);

        if(data & DP_CTRL_STAT_STICKYERR)
        {
            // Clear the error flags
            data |= 0xF0000000;
            IHIL_Write_Read_Dp(DP_CTRL_STAT, &data, WRITE);
            retVal = -1;
        }
        else
        {
            retVal = 0;
        }
    }

#endif
    return retVal;
}
