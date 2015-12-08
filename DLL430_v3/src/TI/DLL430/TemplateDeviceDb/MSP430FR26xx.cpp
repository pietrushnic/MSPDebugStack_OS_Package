/*
 * MSP430FR26xx.cpp
 *
 * Definition MSP430FR26xx.
 *
 * Copyright (C) 2011-2014 Texas Instruments Incorporated - http://www.ti.com/
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

#include "FramMemoryAccessBase.h"
#include "MpuWriteProtection.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;


static const ClockPair FR25_TA2_2 = {"Timer2_A2", 0x8B};
static const ClockPair FR25_TA2_3 = {"Timer3_A2", 0x8C};

struct MSP430FR26xx_EemTimerLarge : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430FR26xx_EemTimerLarge() : EemTimerImpl(
		Eem::PORT, Eem::CAPTIVATE, Eem::Empty, Eem::Empty,
		Eem::ADC10_A, Eem::RTC, Eem::Empty, Eem::eUSCIB0,
		Eem::eUSCIA1, Eem::eUSCIA0, Eem::Empty, FR25_TA2_2,
		FR25_TA2_3, Eem::TA3_0, Eem::TA3_1, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

typedef ClockInfo<GCC_EXTENDED, 0x041F, MSP430FR26xx_EemTimerLarge, EmptyEemClockNames> FR26xx_LargeClockInfo;

typedef Trigger<EMEX_SMALL_5XX, 0x03, 0x01, 0x04, 0x01, 0x01, 0x01, 0x01, 0x02, 0x00, 0x01> FR26xxTrigger;
typedef EemInfo<0x00, 0x01, 0x01, FR26xxTrigger, SmallSequencer> FR26xxEemMode;


typedef MpuWriteProtection<0x160, 0x1, 0x00FF, 0xA500> MpuFR26xxMain;
typedef MpuWriteProtection<0x160, 0x2, 0x00FF, 0xA500> MpuFR26xxInfo;

template<class SizeType, class OffsetType>
struct MSP430FR26xx_MainFramMemory : MemoryInfo<
										MemoryArea::MAIN, RamType, Mapped, NotProtectable, Bits16Type,
										SizeType, OffsetType, SegmentSize<0x01>,
										BankSize<0x0>, Banks<1>, NoMask,
										MemoryCreator<FramMemoryAccessBase<MpuFR26xxMain> >
									> {};


typedef MemoryInfo<
	MemoryArea::LIB, RomType, Mapped, NotProtectable, Bits16Type,
	Size<0x3000>, Offset<0x4000>, SegmentSize<0x1>, BankSize<0>, Banks<1>,
	NoMask, MemoryCreator<BootcodeRomAccess>
> MSP430FR26xx_LibraryCodeMemoryInfo;


template<class SizeType>
struct MSP430FR26xx_SystemRamInfo :	MemoryInfo<
										MemoryArea::RAM, RamType, Mapped, NotProtectable, Bits16Type,
										SizeType, Offset<0x2000>, SegmentSize<0x1>,
										BankSize<0x0>, Banks<1>, NoMask
									> {};


typedef MemoryInfo<
	MemoryArea::INFO, RamType, Mapped, NotProtectable, Bits16Type,
	Size<0x200>, Offset<0x1800>, SegmentSize<0x01>, BankSize<0x0>, Banks<1>,
	NoMask, MemoryCreator<FramMemoryAccessBase<MpuFR26xxInfo> >
> MSP430FR26xx_InfoFramMemoryInfo;


typedef MemoryInfo<
	MemoryArea::BSL, RomType, Mapped, Protectable, Bits16Type,
	Size<0x800>, Offset<0x1000>, SegmentSize<0x01>, BankSize<0>, Banks<1>,
	NoMask, MemoryCreator<BslRomAccessGR>
> MSP430FR26xx_BslMemory;


struct FunctionMappingMSP430FR26xx : public FunctionMappingXv2FRAM
{
	FunctionMappingMSP430FR26xx(){
		ReplacePair(ID_WriteFramQuickXv2, ID_WriteMemWordsXv2);
	}
};


struct FuncletMappingXv2FRAMFR26xx : FuncletMappingImpl
{
	FuncletMappingXv2FRAMFR26xx()
		: FuncletMappingImpl(
			FuncletCode( eraseFuncletCodeXv2FR41xx, sizeof(eraseFuncletCodeXv2FR41xx) ),
			FuncletCode( writeFuncletCodeXv2FRAM, sizeof(writeFuncletCodeXv2FRAM) ) )
	{}
};

typedef Features<MOD_OSC, false, true, true, false, true> MSP430FR26xx_Features;

template<
	const char* description,
	const unsigned int versionId,
	const unsigned int FramOffset,
	const unsigned int FramSize,
	const unsigned int SysRamSize
>
struct MSP430FR26xx_type : Device<
							description,
							DefaultBits20Type,
							regular,
							MSP430F5xxx_Match<versionId>,
							FR26xxEemMode,
							MSP430F5xxx_DefaultVoltageTestVpp,
							FR26xx_LargeClockInfo,
							FunctionMappingMSP430FR26xx,
							FuncletMappingXv2FRAMFR26xx,
							MemoryList<std::tuple<
								MSP430FR26xx_MainFramMemory< Size<FramSize>, Offset<FramOffset> >,
								MSP430FR26xx_LibraryCodeMemoryInfo,
								MSP430FR26xx_SystemRamInfo< Size<SysRamSize> >,
								MSP430F5xxx_BootCodeMemoryInfo,
								MSP430FR26xx_InfoFramMemoryInfo,
								MSP430FR26xx_BslMemory,
								MSP430F5xxx_peripherl16lbitMemoryInfo,
								MSP430F5xxx_CPUMemoryInfo,
								MSP430F5xxx_EEMMemoryInfo
							> >, //until C++0x, the space between the brackets is important
							MSP430FR26xx_Features,
							NoExtendedFeatures,
							PowerSettings<0x00010018, // Test reg mask
										  0x00010000, // Test reg default
									      0x00000018, // Test reg value to enable LPMx.5
										  0x00000018, // Test reg value to disable LPMx.5
										  0x4020,	  // 3V Test reg mask
										  0x0000,	  // 3V Test reg default
										  0x4020,	  // 3V Test reg value to enable LPMx.5
										  0x4020>	  // 3V Test reg value to disable LPMx.5
						> {};


extern const char MSP430FR2633[] = "MSP430FR2633";
extern const char MSP430FR2533[] = "MSP430FR2533";
extern const char MSP430FR2632[] = "MSP430FR2632";
extern const char MSP430FR2532[] = "MSP430FR2532";

extern const char MSP430FR2433[] = "MSP430FR2433";


static const DeviceRegistrator<
	MSP430FR26xx_type<MSP430FR2633, 0x823C, 0xC400, 0x3C00, 0x1000>
> regMSP430FR2633;

static const DeviceRegistrator<
	MSP430FR26xx_type<MSP430FR2533, 0x823D, 0xC400, 0x3C00, 0x800>
> regMSP430FR2533;

static const DeviceRegistrator<
	MSP430FR26xx_type<MSP430FR2632, 0x823E, 0xE000, 0x2000, 0x800>
> regMSP430FR2632;

static const DeviceRegistrator<
	MSP430FR26xx_type<MSP430FR2532, 0x823F, 0xE000, 0x2000, 0x400>
> regMSP430FR2532;


static const DeviceRegistrator<
	MSP430FR26xx_type<MSP430FR2433, 0x8240, 0xC400, 0x3C00, 0x1000>
> regMSP430FR2433;
