/*
 * MemoryManagerV3.cpp
 *
 * Manages to which MemoryAreaBases to communicate dependent on address.
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

#include "MemoryManagerV3.h"
#include "DeviceInfo.h"
#include "MemoryAreaFactory.h"
#include "HalExecCommand.h"
#include "DeviceHandle.h"
#include "FetHandleV3.h"

using namespace TI::DLL430;

MemoryManagerV3::MemoryManagerV3 (DeviceHandle* parent, const DeviceInfo* devInfo)
 : parent(parent), lastError(MEMORY_NO_ERROR), preserveRam(true)
{
	MemoryAreaFactory fac(parent, devInfo);
	for (const auto& it : devInfo->getMemoryInfo())
	{
		if (it->name == MemoryArea::CPU)
		{
			CpuRegisters* cpu = fac.createCpuRegisters(*it);
			if (cpu)
			{
				cpus.push_back(CpuList::value_type(cpu));
			}
		}
		else
		{
			MemoryAreaBase* area = fac.createMemoryArea(this, *it);
			if (area)
			{
				types.push_back(MemoryList::value_type(area));
			}
		}
	}
}

MemoryAreaBase* MemoryManagerV3::getMemoryArea(MemoryArea::Name name, size_t subIndex)
{
	for (const auto& it : types)
	{
		if (it->getName() == name && subIndex-- == 0)
		{
			return &(*it);
		}
	}
	return nullptr;
}

CpuRegisters* MemoryManagerV3::getCpuRegisters(size_t index)
{
	return (index < cpus.size()) ? this->cpus[index].get() : 0;
}

MemoryAreaBase* MemoryManagerV3::getMemoryArea (size_t index)
{
	return index < count() ? this->types[index].get() : nullptr;
}

size_t MemoryManagerV3::count () const
{
	return this->types.size();
}

bool MemoryManagerV3::doForMemoryAreas(uint32_t address, const uint8_t* buffer, size_t count, AccessFunction function)
{
	if (count == 0) //Nothing to do
	{
		return true;
	}

	for (const auto& it : types)
	{
		if (!it->isMapped() || !it->isAccessible())
			continue;

		const uint32_t bottom = std::max(address, it->getStart());
		const uint32_t top = std::min<uint32_t>(address + static_cast<uint32_t>(count) - 1, it->getEnd());
		if (bottom > top)
			continue;

		const uint8_t* bufferPtr = buffer ? (buffer + bottom - address) : 0;
		if (!((*it).*function)(bottom - it->getStart(), bufferPtr, top - bottom + 1))
		{
			lastError = it->getError();
			return false;
		}
	}
	return true;
}

bool MemoryManagerV3::read(uint32_t address, uint8_t* buffer, size_t count)
{
	if (count == 0) //Nothing to do
	{
		return true;
	}

	for (const auto& it : types)
	{
		if (!it->isMapped())
			continue;

		const uint32_t bottom = std::max(address, it->getStart());
		const uint32_t top = std::min<uint32_t>(address + count - 1, it->getEnd());
		if (bottom > top)
			continue;

		uint8_t* bufferPtr = buffer ? (buffer + bottom - address) : 0;
		if (!it->read(bottom - it->getStart(), bufferPtr, top - bottom + 1))
		{
			lastError = it->getError();
			return false;
		}
	}
	return true;
}

bool MemoryManagerV3::verify(uint32_t address, const uint8_t* buffer, size_t count)
{
	return doForMemoryAreas(address, buffer, count, &MemoryArea::verify);
}

bool MemoryManagerV3::overwrite(uint32_t address, const uint8_t* buffer, size_t count)
{
	return doForMemoryAreas(address, buffer, count, &MemoryArea::overwrite);
}

bool MemoryManagerV3::write(uint32_t address, const uint8_t* buffer, size_t count)
{
	return doForMemoryAreas(address, buffer, count, &MemoryArea::write);
}

bool MemoryManagerV3::write(uint32_t address, uint32_t value)
{
	for (const auto& it : types)
	{
		if (!it->isMapped() || !it->isAccessible() || it->isReadOnly())
			continue;

		if (it->getStart() <= address && address <= it->getEnd())
		{
			bool success = it->write(address - it->getStart(), value);
			if (!success)
				lastError = it->getError();

			return success;
		}
	}
	return true;
}

bool MemoryManagerV3::sync()
{
	for (const auto& it : types)
	{
		if (!it->isMapped() || !it->isAccessible())
			continue;

		if (!it->sync())
			return false;
	}
	return true;

}

bool MemoryManagerV3::isReadOnly() const
{
	return false;
}

bool MemoryManagerV3::lock(MemoryArea::Name name, bool action)
{
	MemoryAreaBase* mab = getMemoryArea(name);
	if (nullptr != mab)
	{
		return action ? mab->lock() : mab->unlock();
	}
	return true;
}

bool MemoryManagerV3::erase()
{
	MemoryArea* main = this->getMemoryArea(MemoryArea::MAIN);
	MemoryArea* info = this->getMemoryArea(MemoryArea::INFO);
	MemoryArea* bsl = this->getMemoryArea(MemoryArea::BSL);

	if (main && !main->erase())
		return false;

	if (info && !info->erase())
		return false;

	if (bsl && !bsl->erase())
		return false;

	return true;
}

bool MemoryManagerV3::erase(uint32_t start, uint32_t end)
{
	for (const auto& it : types)
	{
		if (!it->isMapped() || !it->isAccessible())
			continue;

		if (it->isReadOnly())
			continue;

		const uint32_t bottom = std::max(start, it->getStart());
		const uint32_t top = std::min(end, it->getEnd());
		if (bottom > top)
			continue;

		if (!it->erase(bottom, top))
			return false;
	}
	return true;
}

MemoryError MemoryManagerV3::getLastError()
{
	MemoryError error = lastError;
	lastError = MEMORY_NO_ERROR;
	return error;
}

void MemoryManagerV3::setRamPreserveMode(bool enabled)
{
	preserveRam = enabled;
}

bool MemoryManagerV3::getRamPreserveMode() const
{
	return preserveRam;
}

bool MemoryManagerV3::uploadFunclet(FuncletCode::Type type)
{
	const FuncletCode& funclet = parent->getFunclet(type);
	const uint8_t* code = (uint8_t*)funclet.code();
	const size_t count = funclet.codeSize();

	MemoryArea* ram = this->getMemoryArea(MemoryArea::RAM, 0);

	return ram && ram->write(0, code, count) && ram->sync();
}

bool MemoryManagerV3::checkMinFlashVoltage() const
{
	const unsigned minFlashVcc = parent->getMinFlashVcc();

	if ( FetHandle* fetHandle = parent->getFetHandle() )
	{
		if ( const ConfigManager* configManager = fetHandle->getConfigManager() )
		{
			//Don't check during updates (no valid voltage will be returned)
			return configManager->isUpdateRequired() ||
				   configManager->getDeviceVcc() >= minFlashVcc ||
				   configManager->getExternalVcc() >= minFlashVcc;
		}
	}
	return false;
}
