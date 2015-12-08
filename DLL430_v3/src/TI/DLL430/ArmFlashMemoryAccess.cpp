/*
 * ArmFlashMemoryAccess.cpp
 *
 * Memory class for accessing flash memory.
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
#include "ArmFlashMemoryAccess.h"
#include "HalExecCommand.h"
#include "DeviceHandle.h"
#include "FetHandleV3.h"
#include "ClockCalibration.h"
#include "MSP432_FlashLib.h"

using namespace TI::DLL430;
using std::vector;
using std::bind;
using std::shared_ptr;

ArmFlashMemoryAccess::ArmFlashMemoryAccess (
				MemoryArea::Name name,
				DeviceHandle* devHandle,
				uint32_t start,
				uint32_t end,
				uint32_t seg,
				uint32_t banks,
				bool mapped,
				const bool isProtected,
				MemoryManager* mm,
				uint8_t psa
)
 : ArmRandomMemoryAccess(name, devHandle, start, end, seg, banks, mapped, isProtected, mm, psa),
   mSegmentSize(seg)
{
}

ArmFlashMemoryAccess::~ArmFlashMemoryAccess()
{
}

bool ArmFlashMemoryAccess::eraseInit()
{
	MemoryArea* cpu = mm->getMemoryArea(MemoryArea::CPU);
	DebugManager *dbm = devHandle->getDebugManager();
	if (!cpu || !dbm)
	{
		return false;
	}
	mm->write(MSP432_FlashLib::FLASH_FUNCTION_ADDRESS, MSP432_FlashLib::FLASH_INIT);
	cpu->write(13, MSP432_FlashLib::RAM_LOADER_STACK);
	cpu->write(15, MSP432_FlashLib::RAM_LOADER_MAIN);

	mm->sync();
	dbm->run(0);

	uint32_t status = 0;
	do
	{
		uint8_t tmp[4] = { 0 };
		mm->read(MSP432_FlashLib::RETURN_CODE_ADDRESS, tmp, sizeof(status)) && mm->sync();
		status = tmp[0] | (tmp[1] << 8) | (tmp[2] << 16) | (tmp[3] << 24);
	} while (status == MSP432_FlashLib::FLASH_BUSY);

	// halt CPU
	dbm->stop();

	// clear RETURN_CODE_ADDRESS value
	mm->write(MSP432_FlashLib::RETURN_CODE_ADDRESS, 0x0000);
	mm->sync();

	return (status == MSP432_FlashLib::FLASH_SUCCESS);
}


bool ArmFlashMemoryAccess::erase()
{
	MemoryArea* cpu = mm->getMemoryArea(MemoryArea::CPU);
	DebugManager *dbm = devHandle->getDebugManager();
	if (!cpu || !dbm)
	{
		return false;
	}
	if (!mm->checkMinFlashVoltage())
	{
		return false;
	}
	dbm->stop();

	if (!uploadFunclet())
	{
		return false;
	}
	if (!eraseInit())
	{
		return false;
	}

	mm->write(MSP432_FlashLib::FLASH_FUNCTION_ADDRESS, MSP432_FlashLib::FLASH_MASS_ERASE);
	mm->write(MSP432_FlashLib::ERASE_PARAM_ADDRESS, MSP432_FlashLib::FLASH_MASS_ERASE);
	cpu->write(13, MSP432_FlashLib::RAM_LOADER_STACK);
	cpu->write(15, MSP432_FlashLib::RAM_LOADER_MAIN);

	mm->sync();
	dbm->run(0);

	uint32_t status = 0;
	do
	{
		uint8_t tmp[4] = { 0 };
		mm->read(MSP432_FlashLib::RETURN_CODE_ADDRESS, tmp, sizeof(status)) && mm->sync();
		status = tmp[0] | (tmp[1] << 8) | (tmp[2] << 16) | (tmp[3] << 24);
	} while (status == MSP432_FlashLib::FLASH_BUSY);

	dbm->stop();
	return (status == MSP432_FlashLib::FLASH_SUCCESS);
}

bool ArmFlashMemoryAccess::erase(uint32_t start, uint32_t end)
{
	MemoryArea* cpu = mm->getMemoryArea(MemoryArea::CPU);
	DebugManager *dbm = devHandle->getDebugManager();
	if (!cpu || !dbm)
	{
		return false;
	}

	if ( !mm->checkMinFlashVoltage() )
	{
		return false;
	}

	dbm->stop();

	if (!uploadFunclet())
	{
		return false;
	}
	if (!eraseInit())
	{
		return false;
	}
	uint32_t status = 0;
	do
	{
		mm->write(MSP432_FlashLib::FLASH_FUNCTION_ADDRESS, MSP432_FlashLib::FLASH_SECTOR_ERASE);
		mm->write(MSP432_FlashLib::DST_ADDRESS, start);
		cpu->write(13, MSP432_FlashLib::RAM_LOADER_STACK);
		cpu->write(15, MSP432_FlashLib::RAM_LOADER_MAIN);

		mm->sync();
		dbm->run(0);

		do
		{
			uint8_t tmp[4] = { 0 };
			mm->read(MSP432_FlashLib::RETURN_CODE_ADDRESS, tmp, sizeof(status)) && mm->sync();
			status = tmp[0] | (tmp[1] << 8) | (tmp[2] << 16) | (tmp[3] << 24);
		} while (status == MSP432_FlashLib::FLASH_BUSY);

		dbm->stop();
		start += mSegmentSize;
	} while ((status == MSP432_FlashLib::FLASH_SUCCESS) && (start < end));

	return (status == MSP432_FlashLib::FLASH_SUCCESS);
}

bool ArmFlashMemoryAccess::doOverwrite(uint32_t address, const uint8_t* data, size_t size)
{
	return false;
}

bool ArmFlashMemoryAccess::doWrite(uint32_t address, const uint8_t* buffer, size_t count)
{
	MemoryArea* cpu = mm->getMemoryArea(MemoryArea::CPU);
	DebugManager *dbm = devHandle->getDebugManager();
	if (!cpu || !dbm)
	{
		return false;
	}
	if ( !mm->checkMinFlashVoltage() )
	{
		return false;
	}

	dbm->stop();

	if (!uploadFunclet())
	{
		return false;
	}
	if (!eraseInit())
	{
		return false;
	}
	size_t bytesToWrite = count;
	size_t bytesIndex = 0;
	uint32_t status = MSP432_FlashLib::FLASH_SUCCESS;

	while ((status == MSP432_FlashLib::FLASH_SUCCESS) && (bytesIndex < count))
	{
		// Do not copy more than SRC_LENGTH_MAX bytes to RAM loader buffer
		if (count - bytesIndex > MSP432_FlashLib::SRC_LENGTH_MAX)
		{
			bytesToWrite = MSP432_FlashLib::SRC_LENGTH_MAX;
		}
		else
		{
			bytesToWrite = count - bytesIndex;
		}

		// Setup
		mm->write(MSP432_FlashLib::FLASH_FUNCTION_ADDRESS, MSP432_FlashLib::FLASH_PROGRAM);
		mm->write(MSP432_FlashLib::SRC_ADDRESS, MSP432_FlashLib::RAM_LOADER_BUFFER);
		mm->write(MSP432_FlashLib::DST_ADDRESS, address + bytesIndex);
		mm->write(MSP432_FlashLib::SRC_LENGTH_ADDRESS, bytesToWrite);

		cpu->write(13, MSP432_FlashLib::RAM_LOADER_STACK);
		cpu->write(15, MSP432_FlashLib::RAM_LOADER_MAIN);

		mm->write(MSP432_FlashLib::RAM_LOADER_BUFFER, buffer + bytesIndex, bytesToWrite);

		mm->sync();
		dbm->run(0);

		do
		{
			uint8_t tmp[4] = { 0 };
			mm->read(MSP432_FlashLib::RETURN_CODE_ADDRESS, tmp, sizeof(status)) && mm->sync();
			status = tmp[0] | (tmp[1] << 8) | (tmp[2] << 16) | (tmp[3] << 24);
		} while (status == MSP432_FlashLib::FLASH_BUSY);

		dbm->stop();
		bytesIndex += bytesToWrite;
	}

	return (status == MSP432_FlashLib::FLASH_SUCCESS);
}
