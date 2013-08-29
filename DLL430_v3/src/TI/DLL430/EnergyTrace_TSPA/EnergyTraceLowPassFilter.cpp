/*
 * EnergyTraceLowPassFilter.cpp
 *
 * Filter incoming value using low-pass filter
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

#include <EnergyTraceLowPassFilter.h>
#include <math.h>

using namespace TI::DLL430;

//---------------------------------
EnergyTraceLowPassFilter::EnergyTraceLowPassFilter() :
	LPF_WEIGHT(0.9),
	THRESHOLD(2000000),
	mOutput(0),
	mPrevSample(0.0)
{
}

//---------------------------------
EnergyTraceLowPassFilter::~EnergyTraceLowPassFilter()
{
}

//---------------------------------
void EnergyTraceLowPassFilter::setCalibrationValues(double *calibrationValues, uint16_t vcc)
{
	LPF_WEIGHT = calibrationValues[0];
	THRESHOLD = calibrationValues[1];
}

//---------------------------------
void EnergyTraceLowPassFilter::Reset(void)
{
	mOutput = 0;
	mPrevSample = 0.0;
}

//---------------------------------
bool EnergyTraceLowPassFilter::AddData(void *data, size_t size)
{
	double newSample = (double) *((uint32_t *)data);
	double lpfWeight = 0.5;

	// If the value is below the threshold, filter it to reduce jitter for constant current, otherwise pass it through
	if (newSample <= 1*1000) // <1uA
	{
		lpfWeight = 0.95;
	}
	else if (newSample <= 10*1000) // <10uA
	{
		lpfWeight = 0.9;
	}
	else if (newSample < 200*1000) // <200uA
	{
		lpfWeight = 0.8;
	}
	else if (newSample < 1*1000*1000) // <1mA
	{
		lpfWeight = 0.6;
	}

	newSample = (1.0-lpfWeight)*newSample + lpfWeight*mPrevSample;
	mOutput = (uint32_t)newSample;
	mPrevSample = newSample;

	return true;
}

void EnergyTraceLowPassFilter::setPrevSample(double PrevSample)
{
	mPrevSample = PrevSample;;
}



//---------------------------------
void* EnergyTraceLowPassFilter::GetReadBufferPtr(void)
{
	return (void *)&mOutput;
}

//---------------------------------
size_t EnergyTraceLowPassFilter::GetReadBufferSize(void)
{
	return sizeof(mOutput);
}
