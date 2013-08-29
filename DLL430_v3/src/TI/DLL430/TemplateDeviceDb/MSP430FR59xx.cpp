/*
 * MSP430FR59xx.cpp
 *
 * Definition MSP430FR59xx.
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/ 
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

#include "MSP430F5xxx.h"
#include "MpuFr5969.h"
#include "FramMemoryAccessFRx9.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;

typedef IdCode<0xFFFF, 0xFFFF, 0xFF, 0x00, 0x00, 0x00, 0x0> MSP430FR59xxPGIdMask;

template<const unsigned int versionId, const unsigned char revision>
struct MSP430FR59xx_Match : Match<
						IdCode<versionId, 0x00, revision, 0x00, 0x00, 0x00, 0x0>, MSP430FR59xxPGIdMask> {};

struct MSP430FR59xx_EemTimerLarge : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430FR59xx_EemTimerLarge() : EemTimerImpl(
		Eem::TA2_1, Eem::Empty, Eem::AES, Eem::COMP_D, 
		Eem::ADC12_B, Eem::RTC, Eem::Empty, Eem::eUSCIB0,
		Eem::eUSCIA1, Eem::eUSCIA0, Eem::TB7_0, Eem::TA3_0,
		Eem::TA3_1, Eem::TA3_2, Eem::TA2_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

typedef ClockInfo<GCC_EXTENDED, 0xBDFF, MSP430FR59xx_EemTimerLarge, EmptyEemClockNames> FR59xx_SmallClockInfo;	

typedef Trigger<EMEX_SMALL_5XX, 0x03, 0x01, 0x04, 0x01, 0x01, 0x01, 0x01, 0x02, 0x00, 0x00> FR59xxTrigger;

typedef EemInfo<0x00, 0x01, 0x01, FR59xxTrigger, SmallSequencer> FR59xxEemMode;


struct MSP430FR59xx_MainFramMemory : MemoryInfo<
										Name::main, RamType, Mapped, NotProtectable, Bits16Type, 
										Size<0xFC00> , Offset<0x4400>, SegmentSize<0x01>, 
										BankSize<0x0>, Banks<1>, NoMask, MemoryCreator<FramMemoryAccessFRx9<MpuFr5969> >
									> {};


struct MSP430FR59xx_SystemRamInfo : MemoryInfo<
										Name::system, RamType, Mapped, NotProtectable, Bits16Type, 
										Size<0x800>, Offset<0x1C00>, SegmentSize<0x1>,
										BankSize<0x0>, Banks<1>, NoMask> {};


typedef MemoryInfo<
	Name::information, RamType, Mapped, NotProtectable, Bits16Type, 
	Size<0x200> , Offset<0x1800>, SegmentSize<0x01>, BankSize<0x0>, Banks<1>,
	NoMask, MemoryCreator<FramMemoryAccessFRx9<MpuFr5969> >
> MSP430FR59xx_InfoFramMemoryInfo;


typedef MemoryInfo<
	Name::boot, RomType, Mapped, Protectable, Bits16Type, 
	Size<0x800> , Offset<0x1000>, SegmentSize<0x01>, BankSize<0>, Banks<1>, 
	NoMask, MemoryCreator<BslRomAccess>
> MSP430FR59xx_BootFramMemoryInfo;


typedef Features<MOD_OSC, false, true, true, false, true, false> MSP430FR59xx_Features;


struct FunctionMappingMSP430FR59xx : public FunctionMappingBase
{
	FunctionMappingMSP430FR59xx(){
		FunctionMappingImpl::fcntMap_ = boost::assign::map_list_of
			(ID_BlowFuse, ID_BlowFuseFram)
			(ID_SyncJtag_AssertPor_SaveContext, ID_SyncJtag_AssertPor_SaveContextXv2)
			(ID_SyncJtag_Conditional_SaveContext,ID_SyncJtag_Conditional_SaveContextXv2)
			(ID_RestoreContext_ReleaseJtag,ID_RestoreContext_ReleaseJtagXv2)
			(ID_ReadMemBytes,ID_ReadMemWordsXv2)
			(ID_ReadMemWords,ID_ReadMemWordsXv2)
			(ID_ReadMemQuick,ID_ReadMemQuickXv2)
			(ID_WriteMemBytes,ID_WriteMemWordsXv2)
			(ID_WriteMemWords,ID_WriteMemWordsXv2)
			(ID_EemDataExchange,ID_EemDataExchangeXv2)
			(ID_SingleStep,ID_SingleStepJStateXv2) 
			(ID_ReadAllCpuRegs,ID_ReadAllCpuRegsXv2)
			(ID_WriteAllCpuRegs,ID_WriteAllCpuRegsXv2)
			(ID_Psa,ID_PsaXv2)
			(ID_WaitForEem,ID_PollJStateReg)
			(ID_ExecuteFunclet,ID_ExecuteFuncletXv2)
			(ID_WaitForStorage,ID_WaitForStorageX);
		}
};

struct FunctionMappingMSP430FR59xxPG12 : public FunctionMappingMSP430FR59xx
{
	FunctionMappingMSP430FR59xxPG12(){
		ReplacePair(ID_SingleStep,ID_SingleStepXv2);
		ReplacePair(ID_WaitForEem,ID_WaitForEem); //Important, overwrite with default mapping
	}
};


typedef PowerSettings<0x00010018, // Test reg mask
					  0x00010018, // Test reg value to enable LPMx.5 
					  0x00010000, // Test reg value to disable LPMx.5 
					  0xC020,	  // 3V Test reg mask 
					  0xC020,	  // 3V Test reg value to enable LPMx.5 
					  0x4020>	  // 3V Test reg value to disable LPMx.5
					  MSP430FR59xx_PowerSettings;


typedef PowerSettings<0x00010018, // Test reg mask
					  0x00010018, // Test reg value to enable LPMx.5 
					  0x00010000, // Test reg value to disable LPMx.5 
					  0xC020,	  // 3V Test reg mask 
					  0x4020,	  // 3V Test reg value to enable LPMx.5 
					  0x4020>	  // 3V Test reg value to disable LPMx.5 
					  MSP430FR59xxPG12_PowerSettings;


template<
	const char* description,
	const unsigned int versionId,
	const unsigned char revision = 0x20,
	class FunctionMappingType = FunctionMappingMSP430FR59xx,
	class PowerSettingsType = MSP430FR59xx_PowerSettings

>
struct MSP430FR59xx_type : Device<
							description, 
							ObjectId<0>,
							DefaultBits20Type, 
							regular, 
							MSP430FR59xx_Match<versionId, revision>, //MSP430F5xxx_Match<versionId>,
							FR59xxEemMode, 
							MSP430F5xxx_DefaultVoltageTestVpp,
							FR59xx_SmallClockInfo,
							FunctionMappingType, //FunctionMappingMSP430FR59xx,
							FuncletMappingXv2FRAM,
							MemoryList<boost::tuple<
								MSP430FR59xx_MainFramMemory,
								MSP430FR59xx_InfoFramMemoryInfo, 
								MSP430FR59xx_BootFramMemoryInfo,
								MSP430F5xxx_BootCodeMemoryInfo,
								MSP430FR59xx_SystemRamInfo,	//MSP430F5xxx_SystemRamInfo< Size<0x800> >,
								MSP430F5xxx_peripherl16lbitMemoryInfo, 
								MSP430F5xxx_CPUMemoryInfo, 
								MSP430F5xxx_EEMMemoryInfo 
							> >, //until C++0x, the space between the brackets is important
							MSP430FR59xx_Features,
							NoExtendedFeatures,
							PowerSettingsType
						> {};

extern const char MSP430FR5969[] = "MSP430FR5969";
extern const char MSP430FR5949[] = "MSP430FR5949";

static const DeviceRegistrator<
	MSP430FR59xx_type<MSP430FR5969, 0x8169> 
> regMSP430FR5969;

static const DeviceRegistrator<
	MSP430FR59xx_type<MSP430FR5949, 0x8161> 
> regMSP430FR5949;


static const DeviceRegistrator<
	MSP430FR59xx_type<MSP430FR5969, 0x8169, 0x21> 
> regMSP430FR5969PG21;

static const DeviceRegistrator<
	MSP430FR59xx_type<MSP430FR5949, 0x8161, 0x21> 
> regMSP430FR5949PG21;


static const DeviceRegistrator<
	MSP430FR59xx_type<MSP430FR5969, 0x8169, 0x12, FunctionMappingMSP430FR59xxPG12, MSP430FR59xxPG12_PowerSettings>
> regMSP430FR5969PG12;

static const DeviceRegistrator<
	MSP430FR59xx_type<MSP430FR5949, 0x8161, 0x12, FunctionMappingMSP430FR59xxPG12, MSP430FR59xxPG12_PowerSettings>
> regMSP430FR5949PG12;
