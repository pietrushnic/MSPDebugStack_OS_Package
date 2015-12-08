/*
 * ArmRandomMemoryAccess.cpp
 *
 * Memory class for accessing RAM.
 *
 * Copyright (C) 2007 - 2011 Texas Instruments Incorporated - http://www.ti.com/
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
#include "ArmRandomMemoryAccess.h"
#include "HalExecCommand.h"
#include "DeviceHandle.h"
#include "MSP432_FlashLib.h"

using namespace TI::DLL430;

ArmRandomMemoryAccess::ArmRandomMemoryAccess(
	MemoryArea::Name name,
	DeviceHandle* devHandle,
	uint32_t start,
	uint32_t end,
	uint32_t seg,
	uint32_t banks,
	bool mapped,
	const bool isProtected,
	MemoryManager* mm,
	uint8_t psa)
	: MemoryAreaBase(name, devHandle, start, end, seg, banks, mapped, isProtected, psa), mm(mm)
{
}

ArmRandomMemoryAccess::~ArmRandomMemoryAccess()
{
}

bool ArmRandomMemoryAccess::doWrite(uint32_t address, const uint8_t* buffer, size_t count)
{
	HalExecElement *el = new HalExecElement(ID_MEMAPTransaction);

	el->appendInputData16(0);                        // APSEL
	el->appendInputData16(0);                        // rnw = WRITE

	if ((address & 0x1) || ((address + count) & 01))
	{
		el->appendInputData16(0);                    // dataWidth = 8
	}
	else if ((address & 0x2) || ((address + count) & 02))
	{
		el->appendInputData16(1);                    // dataWidth = 16
	}
	else
	{
		if (name == MemoryArea::PERIPHERY_16BIT)
		{// Force 16-bit transactions to avoid bus errors when accessing 16-bit peripherals
			el->appendInputData16(1);                // dataWidth = 16
		}
		else
		{
			el->appendInputData16(2);                // dataWidth = 32
		}
	}

	el->appendInputData32(address + this->getStart());// address
	el->appendInputData32(count);                    // size in bytes

	for (size_t i = 0; i < count; ++i)
	{
		el->appendInputData8(static_cast<uint8_t>(buffer[i]));
	}

	this->elements.emplace_back(el);

	return true;
}

bool ArmRandomMemoryAccess::doRead(uint32_t address, uint8_t* buffer, size_t count)
{
	HalExecElement *el = new HalExecElement(ID_MEMAPTransaction);

	el->appendInputData16(0);                        // APSEL
	el->appendInputData16(1);                        // rnw = READ

	if ((address & 0x1) || ((address + count) & 01))
	{
		el->appendInputData16(0);                    // dataWidth = 8
	}
	else if ((address & 0x2) || ((address + count) & 02))
	{
		el->appendInputData16(1);                    // dataWidth = 16
	}
	else
	{
		if (name == MemoryArea::PERIPHERY_16BIT)
		{// Force 16-bit transactions to avoid bus errors when accessing 16-bit peripherals
			el->appendInputData16(1);                // dataWidth = 16
		}
		else
		{
			el->appendInputData16(2);                // dataWidth = 32
		}
	}

	el->appendInputData32(address + this->getStart());// address
	el->appendInputData32(count);                    // size in bytes

	ReadElement r(buffer, count, 0, 0, 0);
	this->readMap[this->elements.size()] = r;
	this->elements.emplace_back(el);

	return true;
}

bool ArmRandomMemoryAccess::doWrite(uint32_t address, uint32_t value)
{
	return doWrite(address, (uint8_t*)&value, sizeof(value));
}

bool ArmRandomMemoryAccess::postSync(const HalExecCommand& cmd)
{
	return MemoryAreaBase::postSync(cmd);
}

bool ArmRandomMemoryAccess::uploadFunclet()
{
	MemoryArea* ram = mm->getMemoryArea(MemoryArea::RAM, 0);

	return ram && ram->write(
		MSP432_FlashLib::MSP432P401_Funclet_start - ram->getStart(),
		MSP432_FlashLib::MSP432P401_Funclet,
		MSP432_FlashLib::MSP432P401_Funclet_length) && ram->sync();
}
