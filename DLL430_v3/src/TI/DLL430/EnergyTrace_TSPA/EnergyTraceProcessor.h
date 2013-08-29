/*
 * EnergyTraceProcessor.h
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

#if _MSC_VER > 1000
#pragma once
#endif

#ifndef ENERGY_TRACE_PROCESSOR_H
#define ENERGY_TRACE_PROCESSOR_H

#include <IDataProcessor.h>
#include <DoubleBuffer.h>
#include <stdint.h>
#include <vector>


namespace TI
{
	namespace DLL430
	{
		class EnergyTraceLowPassFilter;
		class EnergyTraceRunningAverageFilter;

		class EnergyTraceProcessor : public IDataProcessor
		{
		public:
			/**
			 * \brief Constructor creates the two buffers
			 * \param size The desired size of the buffers
			 */
			EnergyTraceProcessor();
			~EnergyTraceProcessor();

			void setCalibrationValues(double *calibrationValues, uint16_t vcc);
			static const uint32_t TIME_BASE_ns = 640;

		protected:
			static const uint32_t NUM_CALIBRATION_POINTS = 2;
			static const uint32_t minUpdateRateInMsec = 1000;

			static const uint32_t SKIP_COUNTER = 5;
			static const size_t RUNNING_AVERAGE_SIZE = 50;		///< Buffer size for running avergae filter

			static const uint16_t DQ_LENGTH = 100;				// Length of double-ended queue
			static const uint16_t SOBEL_OFFSET = 2;				// Offset for deltaN's
			static const uint16_t SOBEL_STEP = 25;				// Threshold for Sobel step up&down detection

			static const uint16_t ACCUMULATED_N_DIV_MIN = 1;
			static const uint16_t ACCUMULATED_N_DIV_MAX = 20;
			static const uint16_t ACCUMULATED_N_DIV_STEP_SIZE = 5;

			static const uint32_t IOUT_FILTER_THRESHOLD_NA = 500*1000;	// Threshold to use IOut filter

			uint32_t tickThreshold;								// Adaptive filter threshold
			uint32_t deltaNThreshold;							// Adaptive filter threshold
			double sobelStepThreshold;							// Adaptive filter threshold
			double oneTickinMicroWsec;							// Energy equivalent of 1 tick in uWsec (uJ)


#pragma pack(1)
			///< \brief The EnergyTrace Record sent to the IDE
			typedef struct
			{
				uint64_t header;    ///< The header consists of: [1byte eventID][7byte timestamp in usec]
				uint64_t dstate;    ///< Device state
				uint32_t current;   ///< Current I in nA
				uint16_t voltage;   ///< Voltage V in mV
				uint32_t energy;    ///< Energy  E in nw
			} EnergyRecord;
#pragma pack()



#pragma pack(1)
			///< \brief The EnergyTrace Record sent to the IDE
			typedef struct
			{
				uint64_t header;    ///< The header consists of: [1byte eventID][7byte timestamp in usec]
				uint32_t current;   ///< Current I in nA
				uint16_t voltage;   ///< Voltage V in mV
				uint32_t energy;	///< Energy  E in nw
			} EnergyRecordEt8;
#pragma pack()

#pragma pack(1)
			///< \brief The record received from the firmware - differs from the one sent to the IDE(Analog and Dstate mode)
			typedef struct EnergyTraceRecord
			{
				uint8_t eventID;
				uint32_t TimeStamp;
				uint64_t JState;
				uint32_t currentTicks;
				uint16_t voltage;
			} EnergyTraceRecord_t;
#pragma pack()

#pragma pack(1)
			///< \brief The record received from the firmware - differs from the one sent to the IDE (Analog mode)
			typedef struct EnergyTraceRecordEt8
			{
				uint8_t eventID;
				uint32_t TimeStamp;
				uint32_t currentTicks;
				uint16_t voltage;
			} EnergyTraceRecordEt8_t;
#pragma pack()

			///< \brief Calibration values used to calculate the correct value
			typedef struct Calibration
			{
				double threshold;   ///< Threshold value at which this calibration record must be used
				double refCurrent;  ///< Reference current for calibration point
				double gradient;    ///< Gradient with which to calculate actual value
				double offset;      ///< Offset with which to calculate actual value
			} Calibration_t;

			IDataProcessor *mFilter;   ///< Low Pass Filter used to filter the current samples
			IDataProcessor *mVoutFilter;
			bool mFilterEnable;
			double mTimeTag_us;                  ///< The current timetag
			uint32_t mPrevTimeTag;                 ///< Previous timetag value received from the firmware
			uint32_t mPrevCurrentTick;           ///< Previous current value received from firmware
			Calibration_t mCalibrationValues[NUM_CALIBRATION_POINTS]; ///< Calibration values used to calculate the current
			uint8_t mSkip;                       ///< First record that was recorded

			///< \brief Calculate the correct values to use when calculating the current
			void calculateCalibration(uint16_t vcc);
		};
	}
}

#endif // ENERGY_TRACE_PROCESSOR_H
