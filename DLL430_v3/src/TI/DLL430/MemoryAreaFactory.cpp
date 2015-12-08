/*
 * MemoryAreaFactory.cpp
 *
 * Abstract Factory for creating classes of interface type MemoryAreaBase.
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
#include "MemoryAreaFactory.h"
#include "EemMemoryAccess.h"
#include "CpuMemoryAccess.h"
#include "ArmCpuMemoryAccess.h"
#include "ReadonlyMemoryAccess.h"
#include "RegisterAccessBase.h"
#include "FlashMemoryAccessBase.h"
#include "DeviceInfo.h"
#include "DeviceHandle.h"

using namespace TI::DLL430;


MemoryAreaFactory::MemoryAreaFactory (DeviceHandle* devHandle, const DeviceInfo* devInfo)
 : devHandle(devHandle)
 , psa(devInfo->getPsaType())
{
}

CpuRegisters* MemoryAreaFactory::createCpuRegisters(const DeviceInfo::memoryInfo& memInfo)
{
	if (memInfo.name == MemoryArea::CPU)
	{
		if (memInfo.bits != 32)
		{
			return new CpuMemoryAccess(memInfo.name,
				this->devHandle,
				memInfo.offset,
				memInfo.size,
				memInfo.seg_size,
				memInfo.banks,
				memInfo.mmapped,
				memInfo.bits);
		}
		else
		{
			return new ArmCpuMemoryAccess(memInfo.name,
				this->devHandle,
				memInfo.offset,
				memInfo.size,
				memInfo.seg_size,
				memInfo.banks,
				memInfo.mmapped,
				memInfo.bits);
		}
	}
	return 0;
}

MemoryAreaBase* MemoryAreaFactory::createMemoryArea(MemoryManager* mm, const DeviceInfo::memoryInfo& memInfo)
{
	MemoryAreaBase* access = 0;

	if (memInfo.memoryCreatorPtr->isImplemented())
	{
		access = (*memInfo.memoryCreatorPtr)(memInfo.name,
			this->devHandle,
			memInfo.offset,
			memInfo.size,
			memInfo.seg_size,
			memInfo.banks,
			memInfo.mmapped,
			memInfo.isProtected,
			mm,
			this->psa);
	}
	else
	{
		if (memInfo.name == MemoryArea::EEM)
		{
			return new EemMemoryAccess(memInfo.name, this->devHandle,
				memInfo.offset,
				memInfo.size,
				memInfo.seg_size,
				memInfo.banks,
				memInfo.mmapped,
				memInfo.bits);
		}

		switch (memInfo.type)
		{
		case DeviceInfo::MEMTYPE_FLASH:
			access = new FlashMemoryAccessBase(memInfo.name,
				this->devHandle,
				memInfo.offset,
				memInfo.size,
				memInfo.seg_size,
				memInfo.banks,
				memInfo.mmapped,
				memInfo.isProtected,
				mm,
				this->psa);
			break;

		case DeviceInfo::MEMTYPE_ROM:
			access = new ReadonlyMemoryAccess(
				memInfo.name,
				this->devHandle,
				memInfo.offset,
				memInfo.size,
				memInfo.seg_size,
				memInfo.banks,
				memInfo.mmapped,
				memInfo.isProtected,
				mm,
				this->psa);
			break;

		case DeviceInfo::MEMTYPE_RAM:
			access = new RandomMemoryAccess(
				memInfo.name,
				this->devHandle,
				memInfo.offset,
				memInfo.size,
				memInfo.seg_size,
				memInfo.banks,
				memInfo.mmapped,
				memInfo.isProtected,
				mm,
				this->psa);
			break;

		case DeviceInfo::MEMTYPE_REGISTER:
			//Handles 8 and 16bit register access
			access = new RegisterAccess(memInfo.name,
				this->devHandle,
				memInfo.offset,
				memInfo.size,
				memInfo.seg_size,
				memInfo.banks,
				memInfo.mmapped,
				memInfo.bits,
				memInfo.mask);
			break;
		}
	}

	return access;
}
