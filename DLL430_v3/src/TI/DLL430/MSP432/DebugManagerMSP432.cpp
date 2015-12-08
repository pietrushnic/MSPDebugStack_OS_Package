/*
 * DebugManagerMSP432.cpp
 *
 * Functionality for debugging target device.
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
#include "DebugManagerMSP432.h"
#include "DeviceHandleMSP432.h"
#include "HalExecCommand.h"
#include "PollingManager.h"
#include "CpuRegisters.h"

using namespace TI::DLL430;

/** \brief manage debug actions on the target device */
DebugManagerMSP432::DebugManagerMSP432(DeviceHandle* deviceHandle, const DeviceInfo* devInfo)
	: mDeviceHandle(deviceHandle),
	mDevInfo(devInfo),
	mPollingManager(nullptr),
	mCallback(nullptr)
{
}

DebugManagerMSP432::~DebugManagerMSP432()
{
	if (mDeviceHandle->getFetHandle() && mPollingManager)
	{
		mPollingManager->stopBreakpointPolling(*this->mDeviceHandle);
		mPollingManager->setBreakpointCallback(PollingManager::Callback());

		mPollingManager->stopStateStoragePolling(*this->mDeviceHandle);
		mPollingManager->setStateStorageCallback(PollingManager::Callback());

		mPollingManager->stopLpmPolling(*this->mDeviceHandle);
		mPollingManager->setLpmCallback(PollingManager::Callback());
	}
}

bool DebugManagerMSP432::reconnectJTAG()
{
	return false;
}

bool DebugManagerMSP432::run(uint16_t controlType, DebugEventTarget* target, bool releaseJTAG)
{
	bool retVal = false;

	mCallback = target;

	MemoryManager *mm = mDeviceHandle->getMemoryManager();

	CpuRegisters* cpu = mm->getCpuRegisters();
	cpu->flushCache();

	HalExecCommand cmd;
	cmd.elements.emplace_back(new HalExecElement(this->mDeviceHandle->checkHalId(ID_Run)));

	if (this->mDeviceHandle->send(cmd))
	{
		if (releaseJTAG)
		{
			pausePolling();
		}
		else
		{
			this->resumePolling();
		}

		if (controlType)
		{
			retVal = mPollingManager && mPollingManager->startBreakpointPolling(*this->mDeviceHandle);
		}
		else
		{
			retVal = true;
		}
	}

	return retVal;
}

bool DebugManagerMSP432::stop(bool jtagWasReleased)
{
	MemoryManager *mm = mDeviceHandle->getMemoryManager();
	CpuRegisters *cpu = mm->getCpuRegisters();

	// Pause the polling loop while we try to wake up the device and sync again
	this->pausePolling();

	HalExecCommand cmd;
	cmd.elements.emplace_back(new HalExecElement(this->mDeviceHandle->checkHalId(ID_Halt)));

	if (mDeviceHandle->send(cmd))
	{
		return cpu->fillCache(0, 17);
	}

	return false;
}

bool DebugManagerMSP432::singleStep(uint32_t* cycles)
{
	MemoryManager *mm = mDeviceHandle->getMemoryManager();
	CpuRegisters *cpu = mm->getCpuRegisters();

	cpu->flushCache();

	HalExecCommand cmd;
	cmd.elements.emplace_back(new HalExecElement(this->mDeviceHandle->checkHalId(ID_SingleStepMSP432)));

	if (mDeviceHandle->send(cmd))
	{
		return cpu->fillCache(0, 17);
	}

	return false;
}

uint8_t DebugManagerMSP432::getClockControl() const
{
	return 0;
}

uint16_t DebugManagerMSP432::getClockControlSetting() const
{
	return 0;
}

void DebugManagerMSP432::setClockControlSetting(uint16_t clkcntrl)
{
}

uint16_t DebugManagerMSP432::getGeneralClockDefaultSetting() const
{
	return 0;
}

uint16_t DebugManagerMSP432::getClockModuleDefaultSetting() const
{
	return 0;
}

uint16_t DebugManagerMSP432::getClockModuleSetting() const
{
	return 0;
}

void DebugManagerMSP432::setClockModuleSetting(uint16_t modules)
{
}

char** DebugManagerMSP432::getModuleStrings(uint32_t* n) const
{
	*n = 0;
	return nullptr;
}

char** DebugManagerMSP432::getClockStrings(uint32_t* n) const
{
	*n = 0;
	return nullptr;
}

bool DebugManagerMSP432::initEemRegister()
{
	return false;
}

bool DebugManagerMSP432::activatePolling(uint16_t mask)
{
	return false;
}

bool DebugManagerMSP432::activateJStatePolling(DebugEventTarget* cb)
{
	return false;
}

bool DebugManagerMSP432::queryIsInLpm5State()
{
	return false;
}

void DebugManagerMSP432::setOpcode(uint16_t value)
{
}

bool DebugManagerMSP432::saveContext()
{
	return false;
}

void DebugManagerMSP432::setLpmDebugging(bool enable)
{
}

bool DebugManagerMSP432::getLpmDebugging()
{
	return false;
}

void DebugManagerMSP432::pausePolling()
{
	if (mPollingManager)
	{
		mPollingManager->pausePolling();
	}
}

void DebugManagerMSP432::resumePolling()
{
	if (mPollingManager)
	{
		mPollingManager->resumePolling();
	}
}

bool DebugManagerMSP432::syncDeviceAfterLPMx5()
{
	return false;
}

uint64_t DebugManagerMSP432::getCycleCounterValue()
{
	return 0;
}

void DebugManagerMSP432::resetCycleCounterValue()
{
}

bool DebugManagerMSP432::startStoragePolling()
{
	return false;
}

bool DebugManagerMSP432::stopStoragePolling()
{
	return false;
}

void DebugManagerMSP432::setPollingManager(PollingManager* pollingManager)
{
	this->mPollingManager = pollingManager;
	pollingManager->setBreakpointCallback(std::bind(&DebugManagerMSP432::runEvent, this, std::placeholders::_1));
}

void DebugManagerMSP432::enableLegacyCycleCounter(bool enable)
{
}

bool DebugManagerMSP432::legacyCycleCounterEnabled() const
{
	return false;
}

void DebugManagerMSP432::runEvent(MessageDataPtr messageData)
{
	uint32_t DHCSR;
	(*messageData) >> DHCSR;

	if (DHCSR & 0x00020000)
	{ // Breakpoint hit

		MemoryManager *mm = mDeviceHandle->getMemoryManager();
		CpuRegisters *cpu = mm->getCpuRegisters();

		cpu->fillCache(0, 17);

		if (mCallback)
		{
			mCallback->event(DebugEventTarget::BreakpointHit);
		}
	}
}
