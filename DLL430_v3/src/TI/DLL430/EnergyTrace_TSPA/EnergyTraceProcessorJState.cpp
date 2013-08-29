/*
 * EnergyTraceProcessorJState.cpp
 *
 * Process incoming serial data into EnergyTrace records
 *
 * Copyright (c) 2007 - 2013 Texas Instruments Incorporated - http://www.ti.com/
 *
 * All rights reserved not granted herein.
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free,
 * non-exclusive license under copyrights and patents it now or hereafter
 * owns or controls to make, have made, use, import, offer to sell and sell ("Utilize")
 * this software subject to the terms herein.  With respect to the foregoing patent
 * license, such license is granted  solely to the extent that any such patent is necessary
 * to Utilize the software alone.  The patent license shall not apply to any combinations which
 * include this software, other than combinations with devices manufactured by or for TI (“TI Devices”).
 * No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce this license (including the
 * above copyright notice and the disclaimer and (if applicable) source code license limitations below)
 * in the documentation and/or other materials provided with the distribution
 *
 * Redistribution and use in binary form, without modification, are permitted provided that the following
 * conditions are met:
 *
 *	* No reverse engineering, decompilation, or disassembly of this software is permitted with respect to any
 *     software provided in binary form.
 *	* any redistribution and use are licensed by TI for use only with TI Devices.
 *	* Nothing shall obligate TI to provide you with source code for the software licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution of the source code are permitted
 * provided that the following conditions are met:
 *
 *   * any redistribution and use of the source code, including any resulting derivative works, are licensed by
 *     TI for use only with TI Devices.
 *   * any redistribution and use of any object code compiled from the source code and any resulting derivative
 *     works, are licensed by TI for use only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI’S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TI AND TI’S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <EnergyTraceProcessorJState.h>

using namespace TI::DLL430;
using namespace std;

EnergyTraceProcessorJState::EnergyTraceProcessorJState()
{
	Reset();
}

EnergyTraceProcessorJState::~EnergyTraceProcessorJState()
{
}

void EnergyTraceProcessorJState::Reset(void)
{
	LPMXP5_0 = false;
	LPMXP5_1 = false;

	PGACTCPU = false;
	MCLKACT = false;
	SMCLKACT = false;
	ACLKACT = false;

	LPM4P5 = false;
	MODACT0 = false;
	lastValidLPM = 0;
}

uint64_t EnergyTraceProcessorJState::getLastValidLpm()
{
	return lastValidLPM;
}

bool EnergyTraceProcessorJState::AddData(const uint64_t Jstate)
{
	LPMXP5_1 = (Jstate & LPMX5_1_MASK_J) != 0;
	LPMXP5_0 = (Jstate & LPMX5_0_MASK_J) != 0;

	PGACTCPU = (Jstate & PGACTCPU_MASK_J) != 0;
	MCLKACT = (Jstate & MCLKACT_MASK_J) != 0;
	SMCLKACT = (Jstate & SMCLKACT_MASK_J) != 0;
	ACLKACT = (Jstate & ACLKACT_MASK_J) != 0;

	LPM4P5 = (Jstate & LPM4P5_MASK_J) != 0;
	MODACT0 = (Jstate & MODACT0_MASK_J) != 0;

	if(!LPMXP5_1 && !LPMXP5_0 && !PGACTCPU && !MCLKACT) // Sync broken discard sample
	{
		return false;
	}
	else if(!LPMXP5_1 && !LPMXP5_0 && PGACTCPU && !MCLKACT) // Sync broken discard sample
	{
		return false;
	}
	else if(!LPMXP5_1 && !LPMXP5_0 && PGACTCPU && MCLKACT) // AM
	{
		return true;
	}
	else if(LPMXP5_1 && !LPMXP5_0 && PGACTCPU && !MCLKACT && SMCLKACT && ACLKACT && MODACT0)  //LPM0
	{
		lastValidLPM = Jstate;
		return true;
	}
	else if(LPMXP5_1 && !LPMXP5_0 && PGACTCPU && !MCLKACT && SMCLKACT && ACLKACT && !MODACT0)  //LPM1
	{
		lastValidLPM = Jstate;
		return true;
	}
	else if(LPMXP5_1 && !LPMXP5_0 && PGACTCPU && !MCLKACT && !SMCLKACT && ACLKACT)  //LPM2
	{
		lastValidLPM = Jstate;
		return true;
	}
	else if(LPMXP5_1 && !LPMXP5_0 && !PGACTCPU && !MCLKACT && !SMCLKACT && ACLKACT)  // LPM3
	{
		lastValidLPM = Jstate;
		return true;
	}
	else if(LPMXP5_1 && !LPMXP5_0 && !PGACTCPU && !MCLKACT && !SMCLKACT)  // LPM4
	{
		lastValidLPM = Jstate;
		return true;
	}
	else if(LPMXP5_1 && LPMXP5_0 && !LPM4P5)  // LPM3.5
	{
		lastValidLPM = Jstate;
		return true;
	}
	else if(LPMXP5_1 && LPMXP5_0 && LPM4P5)  // LPM4.5
	{
		lastValidLPM = Jstate;
		return true;
	}
	else if(!LPMXP5_1 && LPMXP5_0)  //Protected code
	{
		return true;
	}
	return false;
}
