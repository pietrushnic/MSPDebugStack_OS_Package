/*
 * MSP430FR69xx.cpp
 *
 * Definition MSP430FR69xx.
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


struct MSP430FR69xx_EemTimerLarge : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430FR69xx_EemTimerLarge(const ClockPair& aes, const ClockPair& lcd) : EemTimerImpl(
		Eem::TA2_1, lcd, aes, Eem::COMP_D, 
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
struct FR69xx_Timer_AesLcd : MSP430FR69xx_EemTimerLarge {
	FR69xx_Timer_AesLcd() : MSP430FR69xx_EemTimerLarge(Eem::AES, Eem::LCD_B) {}
};
struct FR69xx_Timer_Lcd : MSP430FR69xx_EemTimerLarge {
	FR69xx_Timer_Lcd() : MSP430FR69xx_EemTimerLarge(Eem::Empty, Eem::LCD_B) {}
};
struct FR69xx_Timer_Aes : MSP430FR69xx_EemTimerLarge {
	FR69xx_Timer_Aes() : MSP430FR69xx_EemTimerLarge(Eem::AES, Eem::Empty) {}
};
struct FR69xx_Timer_None: MSP430FR69xx_EemTimerLarge {
	FR69xx_Timer_None() : MSP430FR69xx_EemTimerLarge(Eem::Empty, Eem::Empty) {}
};


typedef ClockInfo<GCC_EXTENDED, 0xBDFF, FR69xx_Timer_AesLcd, EmptyEemClockNames> FR69xx_AesLcdClockInfo;	
typedef ClockInfo<GCC_EXTENDED, 0xBDFF, FR69xx_Timer_Aes, EmptyEemClockNames> FR69xx_AesClockInfo;	
typedef ClockInfo<GCC_EXTENDED, 0xBDFF, FR69xx_Timer_Lcd, EmptyEemClockNames> FR69xx_LcdClockInfo;	
typedef ClockInfo<GCC_EXTENDED, 0xBDFF, FR69xx_Timer_None, EmptyEemClockNames> FR69xx_ClockInfo;

typedef Trigger<EMEX_SMALL_5XX, 0x03, 0x01, 0x04, 0x01, 0x01, 0x01, 0x01, 0x02, 0x00, 0x00> FR69xxTrigger;

typedef EemInfo<0x00, 0x01, 0x01, FR69xxTrigger, SmallSequencer> FR69xxEemMode;


template<class SizeType>
struct MSP430FR69xx_MainFramMemory : MemoryInfo<
										Name::main, RamType, Mapped, NotProtectable, Bits16Type, 
										SizeType, Offset<0x4400>, SegmentSize<0x01>, 
										BankSize<0x0>, Banks<1>, NoMask, MemoryCreator<FramMemoryAccessFRx9<MpuFr5969> >
									> {};


struct MSP430FR6xxx_SystemRam2Info : MemoryInfo<
				Name::system, RamType, Mapped, NotProtectable, Bits16Type, 
				Size<0x80>, Offset<0x3C00>, SegmentSize<0x1>, BankSize<0x0>, Banks<1>, 
				NoMask> {};


typedef MemoryInfo<
	Name::information, RamType, Mapped, NotProtectable, Bits16Type, 
	Size<0x200> , Offset<0x1800>, SegmentSize<0x01>, BankSize<0x0>, Banks<1>,
	NoMask, MemoryCreator<FramMemoryAccessFRx9<MpuFr5969> >
> MSP430FR69xx_InfoFramMemoryInfo;


typedef MemoryInfo<
	Name::boot, RomType, Mapped, Protectable, Bits16Type, 
	Size<0x800> , Offset<0x1000>, SegmentSize<0x01>, BankSize<0>, Banks<1>, 
	NoMask, MemoryCreator<BslRomAccess>
> MSP430FR69xx_BootFramMemoryInfo;


typedef Features<MOD_OSC, false, true, true, false, true, false> MSP430FR69xx_Features;


struct FunctionMappingMSP430FR69xx : public FunctionMappingBase
{
	FunctionMappingMSP430FR69xx(){
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
			(ID_SingleStep,ID_SingleStepXv2) 
			(ID_ReadAllCpuRegs,ID_ReadAllCpuRegsXv2)
			(ID_WriteAllCpuRegs,ID_WriteAllCpuRegsXv2)
			(ID_Psa,ID_PsaXv2)
			(ID_ExecuteFunclet,ID_ExecuteFuncletXv2)
			(ID_WaitForStorage,ID_WaitForStorageX);
		}
};



template<
	const char* description,
	const unsigned int versionId,
	class ClockInfo,
	class MainFramSize
>
struct MSP430FR69xx_type : Device<
							description, 
							ObjectId<0>,
							DefaultBits20Type, 
							regular,
							MSP430F5xxx_Match<versionId>,
							FR69xxEemMode, 
							MSP430F5xxx_DefaultVoltageTestVpp,
							ClockInfo,
							FunctionMappingMSP430FR69xx,
							FuncletMappingXv2FRAM,
							MemoryList<boost::tuple<
								MSP430FR69xx_MainFramMemory<MainFramSize>,
								MSP430FR69xx_InfoFramMemoryInfo, 
								MSP430FR69xx_BootFramMemoryInfo,
								MSP430F5xxx_BootCodeMemoryInfo,
								MSP430F5xxx_SystemRamInfo< Size<0x800> >,
								MSP430FR6xxx_SystemRam2Info,
								MSP430F5xxx_peripherl16lbitMemoryInfo, 
								MSP430F5xxx_CPUMemoryInfo, 
								MSP430F5xxx_EEMMemoryInfo 
							> >, //until C++0x, the space between the brackets is important
							MSP430FR69xx_Features,
							NoExtendedFeatures,
							PowerSettings<0x00010018, // Test reg mask
									      0x00010000, // Test reg value to enable LPMx.5 
										  0x00010000, // Test reg value to disable LPMx.5 
										  0xC020,	  // 3V Test reg mask 
										  0x4020,	  // 3V Test reg value to enable LPMx.5 
										  0x4020>	  // 3V Test reg value to disable LPMx.5 
						> {};


extern const char MSP430FR5878[] = "MSP430FR5878";
extern const char MSP430FR5879[] = "MSP430FR5879";

extern const char MSP430FR5977[] = "MSP430FR5977";
extern const char MSP430FR5978[] = "MSP430FR5978";
extern const char MSP430FR5979[] = "MSP430FR5979";

extern const char MSP430FR5888[] = "MSP430FR5888";
extern const char MSP430FR5889[] = "MSP430FR5889";

extern const char MSP430FR5988[] = "MSP430FR5988";
extern const char MSP430FR5989[] = "MSP430FR5989";

extern const char MSP430FR6928[] = "MSP430FR6928";
extern const char MSP430FR6929[] = "MSP430FR6929";

extern const char MSP430FR6977[] = "MSP430FR6977";
extern const char MSP430FR6978[] = "MSP430FR6978";
extern const char MSP430FR6979[] = "MSP430FR6979";

extern const char MSP430FR6887[] = "MSP430FR6887";
extern const char MSP430FR6888[] = "MSP430FR6888";
extern const char MSP430FR6889[] = "MSP430FR6889";

extern const char MSP430FR6987[] = "MSP430FR6987";
extern const char MSP430FR6988[] = "MSP430FR6988";
extern const char MSP430FR6989[] = "MSP430FR6989";


static const DeviceRegistrator<	MSP430FR69xx_type<MSP430FR5878, 0x81C8, FR69xx_ClockInfo, Size<0x17C00> > > regMSP430FR5878;
static const DeviceRegistrator<	MSP430FR69xx_type<MSP430FR5879, 0x81C9, FR69xx_ClockInfo, Size<0x1FC00> > > regMSP430FR5879;

static const DeviceRegistrator<	MSP430FR69xx_type<MSP430FR5977, 0x81AF, FR69xx_AesClockInfo, Size<0x0FC00> > > regMSP430FR5977;
static const DeviceRegistrator<	MSP430FR69xx_type<MSP430FR5978, 0x81B0, FR69xx_AesClockInfo, Size<0x17C00> > > regMSP430FR5978;
static const DeviceRegistrator<	MSP430FR69xx_type<MSP430FR5979, 0x81B1, FR69xx_AesClockInfo, Size<0x1FC00> > > regMSP430FR5979;

static const DeviceRegistrator<	MSP430FR69xx_type<MSP430FR5888, 0x81C2, FR69xx_ClockInfo, Size<0x17C00> > > regMSP430FR5888;
static const DeviceRegistrator<	MSP430FR69xx_type<MSP430FR5889, 0x81C3, FR69xx_ClockInfo, Size<0x1FC00> > > regMSP430FR5889;

static const DeviceRegistrator<	MSP430FR69xx_type<MSP430FR5988, 0x81AA, FR69xx_AesClockInfo, Size<0x17C00> > > regMSP430FR5988;
static const DeviceRegistrator<	MSP430FR69xx_type<MSP430FR5989, 0x81AB, FR69xx_AesClockInfo, Size<0x1FC00> > > regMSP430FR5989;

static const DeviceRegistrator<	MSP430FR69xx_type<MSP430FR6928, 0x81B3, FR69xx_AesLcdClockInfo, Size<0x17C00> > > regMSP430FR6928;
static const DeviceRegistrator<	MSP430FR69xx_type<MSP430FR6929, 0x81B4, FR69xx_AesLcdClockInfo, Size<0x1FC00> > > regMSP430FR6929;

static const DeviceRegistrator<	MSP430FR69xx_type<MSP430FR6977, 0x81AC, FR69xx_AesLcdClockInfo, Size<0x0FC00> > > regMSP430FR6977;
static const DeviceRegistrator<	MSP430FR69xx_type<MSP430FR6978, 0x81AD, FR69xx_AesLcdClockInfo, Size<0x17C00> > > regMSP430FR6978;
static const DeviceRegistrator<	MSP430FR69xx_type<MSP430FR6979, 0x81AE, FR69xx_AesLcdClockInfo, Size<0x1FC00> > > regMSP430FR6979;

static const DeviceRegistrator<	MSP430FR69xx_type<MSP430FR6887, 0x81BE, FR69xx_LcdClockInfo, Size<0x0FC00> > > regMSP430FR6887;
static const DeviceRegistrator<	MSP430FR69xx_type<MSP430FR6888, 0x81BF, FR69xx_LcdClockInfo, Size<0x17C00> > > regMSP430FR6888;
static const DeviceRegistrator<	MSP430FR69xx_type<MSP430FR6889, 0x81C0, FR69xx_LcdClockInfo, Size<0x1FC00> > > regMSP430FR6889;

static const DeviceRegistrator<	MSP430FR69xx_type<MSP430FR6987, 0x81A6, FR69xx_AesLcdClockInfo, Size<0x0FC00> > > regMSP430FR6987;
static const DeviceRegistrator<	MSP430FR69xx_type<MSP430FR6988, 0x81A7, FR69xx_AesLcdClockInfo, Size<0x17C00> > > regMSP430FR6988;
static const DeviceRegistrator<	MSP430FR69xx_type<MSP430FR6989, 0x81A8, FR69xx_AesLcdClockInfo, Size<0x1FC00> > > regMSP430FR6989;
