/*
 * DeviceHandleMSP432.cpp
 *
 * Communication with target device.
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
#include <MSP430.h>

#include "VersionInfo.h"
#include "DeviceHandleMSP432.h"
#include "DebugManagerMSP432.h"
#include "MemoryManagerV3.h"
#include "DeviceDbManagerExt.h"
#include "HalExecCommand.h"
#include "FetControl.h"
#include "WatchdogControl.h"
#include "FetHandleV3.h"
#include "ClockCalibration.h"
#include "EM/EmulationManager/EmulationManager432.h"
#include "EM/SoftwareBreakpoints/ISoftwareBreakpoints.h"
#include "EM/SoftwareBreakpoints/SoftwareBreakpointManager.h"
#include "EM/EemRegisters/EemRegisterAccess432.h"
#include "EM/Exceptions/Exceptions.h"
#include "ArmRandomMemoryAccess.h"
#include "CpuRegisters.h"

#include "../../Bios/include/error_def.h"
#include "../../Bios/include/ConfigureParameters.h"

#include <iostream>

using namespace TI::DLL430;

DeviceHandleMSP432::DeviceHandleMSP432 (FetHandleV3* parent, uint32_t deviceCode)
 : parent(parent)
 , memoryManager(nullptr)
 , debugManager(nullptr)
 , clockCalibration(nullptr)
 , minFlashVcc(2700)
 , clockSystem(BC_1xx)
 , jtagId(0)
 , deviceIdPtr(0)
 , eemVersion(0)
 , mode(JTAG_MODE_4WIRE)
 , deviceCode(deviceCode)
 , mDeviceHasBeenIdentified(false)
{
}

DeviceHandleMSP432::~DeviceHandleMSP432 ()
{
	setEemRegisterAccess432(nullptr);
	SoftwareBreakpointManager::setMemoryAccessFunctions(nullptr, nullptr, nullptr);

	delete memoryManager;
	delete debugManager;
	delete clockCalibration;
}


EmulationManagerPtr DeviceHandleMSP432::getEmulationManager()
{
	if (!emulationManager)
		throw EM_NoEmulationManagerException();

	return this->emulationManager;
}

MemoryManagerV3* DeviceHandleMSP432::getMemoryManager ()
{
	return this->memoryManager;
}

DebugManager* DeviceHandleMSP432::getDebugManager ()
{
	return this->debugManager;
}

bool DeviceHandleMSP432::runBootCode(uint32_t command)
{
	return false;
}

long DeviceHandleMSP432::magicPatternSend(uint16_t ifMode)
{
	return -1;
}

int32_t DeviceHandleMSP432::identifyDevice (uint32_t activationKey, bool afterMagicPattern)
{
	HalExecCommand cmd;
	HalExecElement* el = new HalExecElement(ID_ScanAP);

	cmd.elements.emplace_back(el);
	if (!this->send(cmd))
	{
		return -1;
	}

	std::vector<AccessPort> accessPortIds;
	AccessPort temp;

	// Scan for valid access ports
	do
	{
		temp.idr = cmd.elements[0]->getOutputAt32(accessPortIds.size() * 13);
		if (temp.idr)
		{
			temp.base = cmd.elements[0]->getOutputAt32(accessPortIds.size() * 13 + 4);
			temp.cfg = cmd.elements[0]->getOutputAt32(accessPortIds.size() * 13 + 8);
			temp.portNum = cmd.elements[0]->getOutputAt8(accessPortIds.size() * 13 + 12);
			if (temp.validate())
			{
				// Discover all the components in the AP
				parseAndAddComponent(accessPortIds.size(), temp.components, temp.base & 0xFFFFFFFC, &temp.pid);

				accessPortIds.push_back(temp);
			}
		}
	} while (temp.idr);

	// No APs/Components found
	if (!accessPortIds.size() || accessPortIds[0].components.size() < 2)
	{
		return -1;
	}

	// Read the CPUID
	el = new HalExecElement(ID_MEMAPTransaction);

	el->appendInputData16(0);                        // APSEL
	el->appendInputData16(1);                        // rnw = READ
	el->appendInputData16(2);                       // dataWidth = 32
	el->appendInputData32(accessPortIds[0].components[1].getBase()+ CPUID_OFFSET);// address
	el->appendInputData32(4);                       // size in bytes

	HalExecCommand readCpuIDCmd;
	readCpuIDCmd.elements.emplace_back(el);

	if (!this->send(readCpuIDCmd))
	{
		return -1;
	}

	MSP430_device_idCode idCode;
	idCode.verId = ((accessPortIds[0].pid >> 12) & 0xF000) | (accessPortIds[0].pid & 0xFFF); // Get the PartNumber
	idCode.verSubId = readCpuIDCmd.elements[0]->getOutputAt32(0) >> 4 & 0xFFF;

	TemplateDeviceDbManagerExt db;
	long devId = db.queryDb(idCode);
	if (devId > 0)
	{
		mDeviceHasBeenIdentified = true;

		setDeviceId(devId);

		if (!afterMagicPattern)
		{
			this->debugManager->stop();
		}
	}

	return devId;
}

bool DeviceHandleMSP432::reset(bool hardReset)
{
	HalExecElement *el = new HalExecElement(ID_ResetMSP432);
	el->appendInputData16(hardReset ? 1 : 0); // RsetType: 0 = Core, 1 = System

	HalExecCommand resetCmd;
	resetCmd.elements.emplace_back(el);

	if (!this->send(resetCmd))
	{
		return false;
	}

	CpuRegisters *cpu = this->memoryManager->getCpuRegisters();
	cpu->fillCache(0, 17);

	return true;
}

void DeviceHandleMSP432::setDeviceId (long id)
{
	if (mDeviceHasBeenIdentified)
	{
		// Enable debug
		HalExecCommand enableDebugCmd;
		enableDebugCmd.elements.emplace_back(new HalExecElement(ID_EnableDebug));
		this->send(enableDebugCmd);

		TemplateDeviceDbManagerExt dbm;
		std::shared_ptr<DeviceInfo> info = dbm.queryDb(id);
		this->configure(info.get());
	}
	else
	{
		this->identifyDevice(0, true);
	}
}


void DeviceHandleMSP432::configure (const DeviceInfo* info)
{
	setEemRegisterAccess432(0);
	this->map = info->getMap();

	delete this->memoryManager;
	this->memoryManager = new MemoryManagerV3(this, info);

	setEemRegisterAccess432( dynamic_cast<ArmRandomMemoryAccess*>(memoryManager->getMemoryArea(MemoryArea::EEM)) );

	this->description = info->getDescription();

	delete this->debugManager;
	this->debugManager = new DebugManagerMSP432(this, info);

	this->emulationManager = EmulationManager432::create();
}

bool DeviceHandleMSP432::sendDeviceConfiguration(uint32_t parameter, uint32_t value)
{
	HalExecElement* el = new HalExecElement(ID_Configure);
	el->appendInputData32(parameter);
	el->appendInputData32(value);

	HalExecCommand configCmd;
	configCmd.elements.emplace_back(el);

	return this->send(configCmd);
}

uint32_t DeviceHandleMSP432::getDeviceCode() const
{
	return this->deviceCode;
}

uint32_t DeviceHandleMSP432::getJtagId()
{
	return this->jtagId;
}

uint32_t DeviceHandleMSP432::getDeviceIdPtr()
{
	return this->deviceIdPtr;
}

uint32_t DeviceHandleMSP432::getEemVersion()
{
	return 0;
}

uint32_t DeviceHandleMSP432::readJtagId()
{
	HalExecCommand cmd;
	HalExecElement* el = new HalExecElement(ID_GetJtagIdCode);

	cmd.elements.emplace_back(el);
	if (!this->send(cmd))
	{
		return 0;
	}

	this->jtagId = cmd.elements[0]->getOutputAt32(0);

	return this->jtagId;
};

bool DeviceHandleMSP432::isJtagFuseBlown()
{
	return false;
}

bool DeviceHandleMSP432::secure ()
{
	return false;
}

FetControl* DeviceHandleMSP432::getControl ()
{
	return parent->getControl();
}

bool DeviceHandleMSP432::send (HalExecCommand &command)
{
	return this->parent->getControl()->send(command);
}

void DeviceHandleMSP432::setWatchdogControl (std::shared_ptr<WatchdogControl> ctrl)
{
	this->wdt = ctrl;
}

std::shared_ptr<WatchdogControl> DeviceHandleMSP432::getWatchdogControl() const
{
	return this->wdt;
}

hal_id DeviceHandleMSP432::checkHalId(hal_id base_id) const
{
	DeviceInfo::function_map_type::const_iterator it = map.find(base_id);
	return (it != map.end()) ? it->second : base_id;
}

const FuncletCode& DeviceHandleMSP432::getFunclet(FuncletCode::Type funclet)
{
	return funcletTable[funclet];
}

bool DeviceHandleMSP432::supportsQuickMemRead() const
{
	return false;
}

uint16_t DeviceHandleMSP432::getMinFlashVcc() const
{
	return minFlashVcc;
}

bool DeviceHandleMSP432::hasFram() const
{
	return false;
}

bool DeviceHandleMSP432::hasLPMx5() const
{
	return false;
}

ClockSystem DeviceHandleMSP432::getClockSystem() const
{
	return clockSystem;
}

void DeviceHandleMSP432::disableHaltOnWakeup()
{
	// Disable Debug
	HalExecCommand cmd;
	cmd.elements.emplace_back(new HalExecElement(ID_DisableDebug));
	this->send(cmd);
}

bool DeviceHandleMSP432::eemAccessibleInLpm() const
{
	return true;
}

bool DeviceHandleMSP432::deviceSupportsEnergyTrace() const
{
	return false;
}

bool DeviceHandleMSP432::parseAndAddComponent(uint8_t portNum, std::vector<ComponentPeripheral> &vec, uint32_t baseAddress, uint64_t *ROMpid)
{
	HalExecElement* el = new HalExecElement(ID_MEMAPTransaction);

	el->appendInputData16(portNum);                  // APSEL
	el->appendInputData16(1);                        // rnw = READ
	el->appendInputData16(2);                       // dataWidth = 32
	el->appendInputData32(baseAddress + PID_OFFSET);// address
	el->appendInputData32(12 * 4);                  // size in bytes

	HalExecCommand readPidCidCmd;
	readPidCidCmd.elements.emplace_back(el);

	if (!this->send(readPidCidCmd))
	{
		return false;
	}

	uint32_t PID[] = {readPidCidCmd.elements[0]->getOutputAt32(0x10),
					  readPidCidCmd.elements[0]->getOutputAt32(0x14),
					  readPidCidCmd.elements[0]->getOutputAt32(0x18),
					  readPidCidCmd.elements[0]->getOutputAt32(0x1C),
					  readPidCidCmd.elements[0]->getOutputAt32(0x00),
					  readPidCidCmd.elements[0]->getOutputAt32(0x04),
					  readPidCidCmd.elements[0]->getOutputAt32(0x08),
					  readPidCidCmd.elements[0]->getOutputAt32(0x0C)};

	uint32_t CID[] = {readPidCidCmd.elements[0]->getOutputAt32(0x20),
					  readPidCidCmd.elements[0]->getOutputAt32(0x24),
					  readPidCidCmd.elements[0]->getOutputAt32(0x28),
					  readPidCidCmd.elements[0]->getOutputAt32(0x2C)};

	ComponentPeripheral component(baseAddress, PID, CID);

	if (component.getComponentType() != COMPONENT_UNKNOWN)
	{
		vec.push_back(component);
	}
	if (component.getComponentType() == COMPONENT_ROM_TABLE)
	{
		*ROMpid = (((uint64_t)PID[7] & 0xff) << 56) |
			(((uint64_t)PID[6] & 0xff) << 48) |
			(((uint64_t)PID[5] & 0xff) << 40) |
			(((uint64_t)PID[4] & 0xff) << 32) |
			(((uint64_t)PID[3] & 0xff) << 24) |
			(((uint64_t)PID[2] & 0xff) << 16) |
			(((uint64_t)PID[1] & 0xff) << 8) |
			(((uint64_t)PID[0] & 0xff));

		uint32_t tableOffset = 0;
		uint32_t ROMEntry;
		do
		{
			HalExecElement* el = new HalExecElement(ID_MEMAPTransaction);

			el->appendInputData16(portNum);                   // APSEL
			el->appendInputData16(1);                         // rnw = READ
			el->appendInputData16(2);                        // dataWidth = 32
			el->appendInputData32(baseAddress + tableOffset);// address
			el->appendInputData32(4);                        // size in bytes

			HalExecCommand readRomEntry;
			readRomEntry.elements.emplace_back(el);

			if (!this->send(readRomEntry))
			{
				return false;
			}
			ROMEntry = readRomEntry.elements[0]->getOutputAt32(0);
			if (ROMEntry & 0x1)
			{
				parseAndAddComponent(portNum, vec, baseAddress + (ROMEntry & 0xFFFFF000), ROMpid);
			}
			tableOffset += 4;
		} while (ROMEntry);
	}

	switch (component.getComponentType())
	{
	case COMPONENT_SCS:
		// Configure the base address for the SCS
		sendDeviceConfiguration(CONFIG_PARAM_SCS_BASE_ADDRESS, component.getBase());
		break;

	case COMPONENT_FPB:
		// Configure the base address for the FPB
		sendDeviceConfiguration(CONFIG_PARAM_FPB_BASE_ADDRESS, component.getBase());
		break;

	default:
		// Do nothing
		break;
	}

	return true;
}
