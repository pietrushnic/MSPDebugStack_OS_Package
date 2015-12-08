/*
* RF430F51xx.cpp
*
* Definition of RF430F51xx devices.
*
* Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
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

#include <pch.h>
#include "MSP430F5xxx.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;

struct RF430F51xx_Timer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	RF430F51xx_Timer() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::COMP_B,
		Eem::Empty, Eem::RTC, Eem::Empty, Eem::Empty,
		Eem::USCI1, Eem::USCI0, Eem::Empty, Eem::TD3_1,
		Eem::TD3_0, Eem::TA3_0, Eem::Empty, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
		)
	{}
};

typedef ClockInfo<GCC_EXTENDED, 0x041D, RF430F51xx_Timer, EmptyEemClockNames> RF430F51xx_Clock;


template<class OffsetType, class FlashSizeType>
struct RF430F51xx_MainFlashMemory : MemoryInfo<
	MemoryArea::MAIN, FlashType, Mapped, NotProtectable, Bits16Type,
	FlashSizeType, OffsetType, SegmentSize<0x200>,
	BankSize<0x8000>, Banks<1>, NoMask> {};


template<
	const char* description,
	const unsigned int versionId,
	class FlashOffsetType,
	class FlashSizeType,
	class RamSizeType
>
struct RF430F51xx : Device<
	description,
	DefaultBits20Type,
	enhanced,
	MSP430F5xxx_Match<versionId>,
	SmallEemMode,
	MSP430F5xxx_DefaultVoltageTestVpp,
	RF430F51xx_Clock,
	FunctionMappingXv2,
	FuncletMappingXv2,
	MemoryList<std::tuple<
		RF430F51xx_MainFlashMemory<FlashOffsetType, FlashSizeType>,
		MSP430F5xxx_InfoFlashMemoryInfo,
		MSP430F5xxx_BootFlashMemoryInfo,
		MSP430F5xxx_BootCodeMemoryInfo,
		MSP430F5xxx_SystemRamInfo<RamSizeType>,
		MSP430F5xxx_peripherl16lbitMemoryInfo,
		MSP430F5xxx_CPUMemoryInfo,
		MSP430F5xxx_EEMMemoryInfo
	> >, //until C++0x, the space between the brackets is important
	MSP430F5xxx_Features,
	MSP430F5xxx_ExtFeatures
>
{
};


extern const char RF430F5175[] = "RF430F5175";
extern const char RF430F5155[] = "RF430F5155";
extern const char RF430F5144[] = "RF430F5144";

static const DeviceRegistrator< RF430F51xx<RF430F5175, 0x81EB, Offset<0x8000>, Size<0x8000>, Size<0x800> > > regRF430F5175_type;
static const DeviceRegistrator< RF430F51xx<RF430F5155, 0x81EA, Offset<0xC000>, Size<0x4000>, Size<0x400> > > regRF430F5155_type;
static const DeviceRegistrator< RF430F51xx<RF430F5144, 0x81E9, Offset<0xC000>, Size<0x4000>, Size<0x400> > > regRF430F5144_type;
