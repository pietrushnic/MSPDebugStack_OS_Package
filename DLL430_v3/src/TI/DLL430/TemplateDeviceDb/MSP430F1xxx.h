/*
 * MSP430F1xxx.h
 *
 * Default values for MSP430F1xxx family - to be used when devices are created, e.g. within MSP430F11xx.cpp.
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

#pragma once

#include "SpecialMemoryTypes.h"
#include "MSP430Defaults.h"
#include "MSP430F1_2_4xxx_masks.h"
#include "Registration.h"

namespace TI { namespace DLL430 { namespace TemplateDeviceDb {
	typedef IdCode<0xFFFF, 0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x0F> MSP430F1xxxIdMask;

	template<const unsigned int versionId, const unsigned int fuses>
	struct MSP430F1xxx_Match : Match< IdCode<versionId, 0x0, 0, 0, 0, 0, fuses>, MSP430F1xxxIdMask> {};

	namespace Memory {
		typedef MemoryInfo<
			MemoryArea::INFO, FlashType, Mapped, NotProtectable, Bits16Type,
			Size<0x100>, Offset<0x1000>, SegmentSize<0x80>, BankSize<0x80>, Banks<2>,
			NoMask, MemoryCreator<InformationFlashAccess>
		> MSP430F1xxx_InfoFlashMemoryInfo;

		typedef MemoryInfo<
			MemoryArea::BSL, FlashType, Mapped, Protectable, Bits16Type,
			Size<0x400>, Offset<0x0c00>, SegmentSize<0x200>, BankSize<0>, Banks<4>,
			NoMask, MemoryCreator<BootcodeRomAccess>
		> MSP430F1xxx_BootFlashMemoryInfo;

		typedef MemoryInfo<
			MemoryArea::CPU, RegisterType, NotMapped, NotProtectable, BitsDeviceDefaultType,
			Size<0x10>, Offset<0x0>, SegmentSize<0x1>, BankSize<0x0>, Banks<1>,
			NoMask
		> MSP430F1xxx_CPUMemoryInfo;

		typedef MemoryInfo<
			MemoryArea::EEM, RegisterType, NotMapped, NotProtectable, BitsDeviceDefaultType,
			Size<0x80>, Offset<0>, SegmentSize<0x1>, BankSize<0x0>, Banks<1>,
			NoMask
		> MSP430F1xxx_EEMMemoryInfo;

		template<class SizeType>
		struct MSP430F1xxx_SystemRamInfo : MemoryInfo<
						MemoryArea::RAM, RamType, Mapped, NotProtectable, Bits16Type,
						SizeType, Offset<0x200>, SegmentSize<0x1>, BankSize<0x0>, Banks<1>,
						NoMask> {};

		template<class SizeType>
		struct MSP430F1xxx_SystemRam2Info : MemoryInfo<
						MemoryArea::RAM, RamType, Mapped, NotProtectable, Bits16Type,
						SizeType, Offset<0x1100>, SegmentSize<0x1>, BankSize<0x0>, Banks<1>,
						NoMask> {};

		template<class FlashSizeType, class OffsetType>
		struct MSP430F1xxx_MainFlashMemory : MemoryInfo<
						MemoryArea::MAIN, FlashType, Mapped, NotProtectable, Bits16Type,
						FlashSizeType, OffsetType, SegmentSize<0x200>, BankSize<0x10000>, Banks<1>,
						NoMask> {};
	} //namespace Memory

	typedef VoltageInfo<1800, 3600, 2700, 2500, 6000, 7000, true> MSP430F1xxx_DefaultVoltageTestVpp;
	typedef VoltageInfo<1800, 3600, 2700, 2500, 6000, 7000, false> MSP430F1xxx_DefaultVoltageNoTestVpp;

	typedef Features<BC_1xx, true, false, true, false, false> MSP430F1xxx_I2C;
	typedef Features<BC_1xx, false, false, true, false, false> MSP430F1xxx_NoI2C;

} //namespace TemplateDeviceDb
}//namespace DLL430
}//namespace TI
