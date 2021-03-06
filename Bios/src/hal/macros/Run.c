/**
* \ingroup MODULMACROS
*
* \file Run.c
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

/**
  Run: Runs the Core Debug
*/
#ifdef MSP_FET
extern ARMConfigSettings armConfigSettings;
#endif

HAL_FUNCTION(_hal_Run)
{
#ifdef MSP_FET
    unsigned long data;

    // First Disable all breakpoints
    data = KEY;
    IHIL_Write_Read_Mem_Ap(0, FP_CTRL, &data, WRITE);

    // Perform single step with interrupts disabled to step over current hw breakpoint
    data = DBGKEY | C_DEBUGEN | C_STEP | C_MASKINTS;
    IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE);

    // Re-enable all breakpoints
    data = KEY | ENABLE;
    IHIL_Write_Read_Mem_Ap(0, FP_CTRL, &data, WRITE);

    // Do the actual run
    data = DBGKEY | C_DEBUGEN;
    IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE);
#endif
    return 0;
}
