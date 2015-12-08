/*
 * DeviceHandleV3.cpp
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
#include <cmath>

#include "VersionInfo.h"
#include "DeviceHandleV3.h"
#include "DeviceDbManagerExt.h"
#include "HalExecCommand.h"
#include "FetControl.h"
#include "WatchdogControl.h"
#include "FetHandleV3.h"
#include "MemoryManagerV3.h"
#include "CpuRegisters.h"
#include "DebugManagerV3.h"
#include "ClockCalibration.h"
#include "EM/EmulationManager/EmulationManager430.h"
#include "EM/SoftwareBreakpoints/ISoftwareBreakpoints.h"
#include "EM/SoftwareBreakpoints/SoftwareBreakpointManager.h"
#include "EM/EemRegisters/EemRegisterAccess430.h"
#include "EM/Exceptions/Exceptions.h"
#include "EemMemoryAccess.h"
#include "JtagId.h"
#include "JtagShifts.h"
#include "FileReader.h"

#include "../../Bios/include/error_def.h"
#include "../../Bios/include/ConfigureParameters.h"

using namespace TI::DLL430;

DeviceHandleV3::DeviceHandleV3 (FetHandleV3* parent, uint32_t deviceCode)
 : parent(parent)
 , memoryManager(nullptr)
 , debugManager(nullptr)
 , clockCalibration(nullptr)
 , minFlashVcc(2700)
 , hasTestVpp(false)
 , quickMemRead(true)
 , deviceHasFram(false)
 , deviceHasLPMx5(false)
 , clockSystem(BC_1xx)
 , jtagId(0)
 , deviceIdPtr(0)
 , eemVersion(0)
 , mode(JTAG_MODE_4WIRE)
 , deviceCode(deviceCode)
 , powerTestRegDefault(0)
 , powerTestReg3VDefault(0)
{
	assert(DeviceInfo::nrUsedClockModules == etwCodes.size());
	for (int i=0;i<DeviceInfo::nrUsedClockModules;i++)
	{
		etwCodes[i]=0;
	}
	//// set watchdog and realtime clock as default
	etwCodes[0] = 1;
	etwCodes[10] = 40;
}

DeviceHandleV3::~DeviceHandleV3 ()
{
	setEemRegisterAccess430(0);
	SoftwareBreakpointManager::setMemoryAccessFunctions(nullptr, nullptr, nullptr);

	delete memoryManager;
	delete debugManager;
	delete clockCalibration;
}


EmulationManagerPtr DeviceHandleV3::getEmulationManager()
{
	if (!emulationManager)
		throw EM_NoEmulationManagerException();

	return this->emulationManager;
}

MemoryManagerV3* DeviceHandleV3::getMemoryManager ()
{
	return this->memoryManager;
}

DebugManagerV3* DeviceHandleV3::getDebugManager ()
{
	return this->debugManager;
}


long DeviceHandleV3::magicPatternSend(uint16_t ifMode)
{
	uint16_t mode[6] = {0};

	if (ifMode == ConfigManager::JTAG_MODE_AUTOMATIC)
	{
		mode[0]	= ConfigManager::JTAG_MODE_SPYBIWIRE;
		mode[1]	= ConfigManager::JTAG_MODE_SPYBIWIRE;
		mode[2]	= ConfigManager::JTAG_MODE_4AFTER2;
		mode[3]	= ConfigManager::JTAG_MODE_4AFTER2;
		mode[4]	= ConfigManager::JTAG_MODE_SPYBIWIRE_MSP_FET;
		mode[5]	= ConfigManager::JTAG_MODE_SPYBIWIRE_MSP_FET;
	}
	else
	{
		mode[0]	= ifMode;
		mode[1]	= ifMode;
		mode[2]	= ifMode;
		if (ifMode != ConfigManager::JTAG_MODE_SPYBIWIRE)
		{
			mode[3]	= ConfigManager::JTAG_MODE_SPYBIWIRE_MSP_FET;
			mode[4]	= ConfigManager::JTAG_MODE_SPYBIWIRE_MSP_FET;
			mode[5]	= ConfigManager::JTAG_MODE_SPYBIWIRE_MSP_FET;
		}
		else
		{
			mode[3]	= ifMode;
			mode[4]	= ifMode;
			mode[5]	= ifMode;
		}
	}

	for (uint16_t i = 0; i < 6; i++)
	{
		ConfigManager* cm = parent->getConfigManager();

		HalExecElement* el = new HalExecElement(ID_MagicPattern);
		el->appendInputData16(mode[i]);

		if (mode[i] == ConfigManager::JTAG_MODE_SPYBIWIRE_MSP_FET)
		{
			cm->setJtagMode((ConfigManager::jtagMode)JTAG_MODE_4AFTER2);
		}
		else
		{
			cm->setJtagMode((ConfigManager::jtagMode)mode[i]);
		}

		HalExecCommand cmd;
		cmd.elements.emplace_back(el);

		if (!this->send(cmd))
		{
			uint16_t errorCode =  el->getOutputAt16(0);

			if (errorCode != HALERR_MAGIC_PATTERN )
			{
				return errorCode;
			}
		}
		const uint8_t chainLen = el->getOutputAt8(0);
		const uint8_t iJtagID = el->getOutputAt8(1);

		if ((chainLen > 0) && jtagIdIsValid(iJtagID))
		{
			return 0;
		}
	}
	return -1;
}


int32_t DeviceHandleV3::identifyDevice (uint32_t activationKey, bool afterMagicPattern)
{
	const bool assertBSLValid = (activationKey == 0x20404020);

	sendDeviceConfiguration(CONFIG_PARAM_CLK_CONTROL_TYPE, GccNone);
	sendDeviceConfiguration(CONFIG_PARAM_SFLLDEH, 0);
	sendDeviceConfiguration(CONFIG_PARAM_DEFAULT_CLK_CONTROL, 0x040f);
	sendDeviceConfiguration(CONFIG_PARAM_ENHANCED_PSA, DeviceInfo::PSATYPE_REGULAR);
	sendDeviceConfiguration(CONFIG_PARAM_PSA_TCKL_HIGH, 0);
	sendDeviceConfiguration(CONFIG_PARAM_POWER_TESTREG_MASK, 0x0000);
	sendDeviceConfiguration(CONFIG_PARAM_POWER_TESTREG3V_MASK, 0x0000);
	sendDeviceConfiguration(CONFIG_PARAM_POWER_TESTREG_DEFAULT, 0);
	sendDeviceConfiguration(CONFIG_PARAM_POWER_TESTREG3V_DEFAULT, 0);
	sendDeviceConfiguration(CONFIG_ALT_ROM_ADDR_FOR_CPU_READ, 0);
	sendDeviceConfiguration(CONFIG_ASSERT_BSL_VALID_BIT, (assertBSLValid ? 1 : 0));

	if (this->getWatchdogControl())
	{
		sendDeviceConfiguration(CONFIG_WDT_ADDRESS_5XX, this->getWatchdogControl()->getAddress());
	}


	if (this->isJtagFuseBlown())
	{
		return -5555;
	}

	long devId = -1;
	uint32_t pc = 0;
	uint32_t sr = 0;

	if (jtagIdIsValid(this->getJtagId()))
	{
		devId = this->getDeviceIdentity(activationKey, &pc, &sr, afterMagicPattern);
	}
	if (devId < 0)
	{
		return static_cast<int32_t>(devId);
	}
	this->setDeviceId(devId);
	MemoryManager* mm = this->getMemoryManager();
	if (mm)
	{
		CpuRegisters* cpu = mm->getCpuRegisters();
		if (cpu)
		{
			cpu->write(0, pc);
			cpu->write(2, sr);
			cpu->fillCache(0, 16);
		}
	}
	return static_cast<int32_t>(devId);
}

const std::string & DeviceHandleV3::getDescription()
{
	return description;
}

// reset moved from ConfigManager to Device
bool DeviceHandleV3::reset (bool hardReset)
{
	std::shared_ptr<WatchdogControl> wdt = this->getWatchdogControl();

	HalExecCommand cmd;

	HalExecElement* el = new HalExecElement(this->checkHalId(ID_SyncJtag_AssertPor_SaveContext));

	wdt->addHoldParamsTo(el);

	el->appendInputData8(this->getJtagId());

	for (int i=0;i<16;i++)
	{
		el->appendInputData8(etwCodes[i]);
	}

	cmd.elements.emplace_back(el);
	if (!this->parent->getControl()->send(cmd))
		return false;

	uint16_t wdtCtrl = el->getOutputAt16(0);
	if (!WatchdogControl::checkRead(wdtCtrl))
		return false;

	wdt->set(wdtCtrl);

	MemoryManagerV3* mm = this->getMemoryManager();
	if (mm==nullptr)
		return false;

	if ( CpuRegisters* cpu = mm->getCpuRegisters() )
	{
		cpu->write(0, el->getOutputAt32(2));
		cpu->write(2, el->getOutputAt16(6));
		cpu->fillCache(0, 16);
	}
	return true;
}

void DeviceHandleV3::setDeviceId (long id)
{
	TemplateDeviceDbManagerExt dbm;
	std::shared_ptr<DeviceInfo> info = dbm.queryDb(static_cast<size_t>(id));
	this->configure(info.get());
}


void DeviceHandleV3::configure (const DeviceInfo* info)
{
	using namespace std::placeholders;

	SoftwareBreakpointManager::setMemoryAccessFunctions(nullptr, nullptr, nullptr);
	setEemRegisterAccess430(nullptr);

	this->map = info->getMap();

	this->funcletTable = info->getFuncletMap();

	this->minFlashVcc = info->minFlashVcc();
	this->hasTestVpp = info->hasTestVpp();
	this->quickMemRead = info->quickMemRead();
	this->deviceHasFram = info->hasFram();
	this->deviceHasLPMx5 = (info->powerTestRegMask() != 0) || (info->powerTestReg3VMask() != 0);
	this->clockSystem = info->clockSystem();

	this->powerTestRegDefault = info->powerTestRegDefault();
	this->powerTestReg3VDefault = info->powerTestReg3VDefault();

	this->description = info->getDescription();

	delete this->memoryManager;
	this->memoryManager = new MemoryManagerV3(this, info);

	delete this->debugManager;
	this->debugManager  = new DebugManagerV3(this, info);

	this->emulationManager = EmulationManager430::create(info->getEmulationLevel());

	SoftwareBreakpointManager::setMemoryAccessFunctions(
													std::bind(&MemoryManager::read, memoryManager, _1, _2, _3),
													std::bind(&MemoryManager::overwrite, memoryManager, _1, _2, _3),
													std::bind(&MemoryManager::sync, memoryManager));

	setEemRegisterAccess430( dynamic_cast<EemMemoryAccess*>(memoryManager->getMemoryArea(MemoryArea::EEM)) );

	const ConfigManager* configManager = parent->getConfigManager();
	this->clockCalibration = ClockCalibration::create(this, memoryManager, configManager, *info);

	// If frequency calibration is disabled, remap funclets to FLL versions (those won't change any settings)
	if (configManager && !configManager->freqCalibrationEnabled())
	{
		const FuncletCode& erase = funcletTable[FuncletCode::ERASE];
		if ( erase == FuncletCode(eraseFuncletCodeDCO, sizeof(eraseFuncletCodeDCO), 4) )
		{
			funcletTable[FuncletCode::ERASE] = FuncletCode( eraseFuncletCodeFLL, sizeof(eraseFuncletCodeFLL), 4 );
			funcletTable[FuncletCode::WRITE] = FuncletCode( writeFuncletCodeFLL, sizeof(writeFuncletCodeFLL), 128 );
		}
		else if ( erase == FuncletCode( eraseFuncletCodeXDCO, sizeof(eraseFuncletCodeXDCO), 4 ) )
		{
			funcletTable[FuncletCode::ERASE] = FuncletCode( eraseFuncletCodeXFLL, sizeof(eraseFuncletCodeXFLL), 4 );
			funcletTable[FuncletCode::WRITE] = FuncletCode( writeFuncletCodeXFLL, sizeof(writeFuncletCodeXFLL), 256 );
		}
	}

	const DeviceInfo::ClockMapping& clockMapping = info->getClockMapping();
	assert(DeviceInfo::nrUsedClockModules <= clockMapping.size());

	for (uint32_t i=0;i<DeviceInfo::nrUsedClockModules;++i)
	{
		etwCodes[DeviceInfo::nrUsedClockModules-(i+1)] = clockMapping[i].second;
	}

	sendDeviceConfiguration(CONFIG_PARAM_CLK_CONTROL_TYPE, info->getClockControl());
	sendDeviceConfiguration(CONFIG_PARAM_SFLLDEH, info->getSFll());
	sendDeviceConfiguration(CONFIG_PARAM_DEFAULT_CLK_CONTROL, info->getClockModDefault());
	sendDeviceConfiguration(CONFIG_PARAM_ENHANCED_PSA, info->getPsaType());

	if (this->getJtagId() == 0x89)
	{
		sendDeviceConfiguration(CONFIG_PARAM_PSA_TCKL_HIGH, info->psach());
	}
	else
	{
		sendDeviceConfiguration(CONFIG_PARAM_PSA_TCKL_HIGH, 2);
	}

	// Set power settings
	sendDeviceConfiguration(CONFIG_PARAM_POWER_TESTREG_MASK, info->powerTestRegMask());
	sendDeviceConfiguration(CONFIG_PARAM_POWER_TESTREG_DEFAULT, info->powerTestRegDefault());
	sendDeviceConfiguration(CONFIG_PARAM_TESTREG_ENABLE_LPMX5, info->testRegEnableLpmx5());
	sendDeviceConfiguration(CONFIG_PARAM_TESTREG_DISABLE_LPMX5, info->testRegDisableLpmx5());

	sendDeviceConfiguration(CONFIG_PARAM_POWER_TESTREG3V_MASK, info->powerTestReg3VMask());
	sendDeviceConfiguration(CONFIG_PARAM_POWER_TESTREG3V_DEFAULT, info->powerTestReg3VDefault());
	sendDeviceConfiguration(CONFIG_PARAM_TESTREG3V_ENABLE_LPMX5, info->testReg3VEnableLpmx5());
	sendDeviceConfiguration(CONFIG_PARAM_TESTREG3V_DISABLE_LPMX5, info->testReg3VDisableLpmx5());

	sendDeviceConfiguration(CONFIG_ALT_ROM_ADDR_FOR_CPU_READ, info->b1377() ? 1 : 0);
}

bool DeviceHandleV3::sendDeviceConfiguration(uint32_t parameter, uint32_t value)
{
	HalExecElement* el = new HalExecElement(ID_Configure);
	el->appendInputData32(parameter);
	el->appendInputData32(value);

	HalExecCommand configCmd;
	configCmd.elements.emplace_back(el);

	return this->send(configCmd);
}

long DeviceHandleV3::getDeviceIdentity(uint32_t activationKey, uint32_t* pc, uint32_t* sr, bool afterMagicPattern)
{
	const bool isXv2 = (jtagIdIsXv2(jtagId) != 0);

	HalExecCommand syncCmd;
	syncCmd.setTimeout(5000);

	hal_id syncMacro = isXv2 ? ID_SyncJtag_AssertPor_SaveContextXv2 : ID_SyncJtag_AssertPor_SaveContext;
	if (isXv2 && afterMagicPattern && (jtagId != 0x99))
	{
		syncMacro = ID_SyncJtag_Conditional_SaveContextXv2;
	}

	HalExecElement* elSync = new HalExecElement(syncMacro);

	std::shared_ptr<WatchdogControl> wdt = this->getWatchdogControl();
	wdt->addHoldParamsTo(elSync);

	elSync->appendInputData8(this->getJtagId());

	for (int i=0;i<16;i++)
	{
		elSync->appendInputData8(etwCodes[i]);
	}
	syncCmd.elements.emplace_back(elSync);

	HalExecElement* elDeviceIdPtr = new HalExecElement(ID_GetDeviceIdPtr);
	syncCmd.elements.emplace_back(elDeviceIdPtr);

	if (!this->send(syncCmd))
	{
		return -1;
	}

	uint16_t wdtCtrl = elSync->getOutputAt16(0);
	// check read watchdog password
	if (!WatchdogControl::checkRead(wdtCtrl))
	{
		return -1;
	}
	wdt->set(wdtCtrl);

	if (syncMacro != ID_SyncJtag_Conditional_SaveContextXv2)
	{
		*pc = elSync->getOutputAt32(2);
		*sr = elSync->getOutputAt16(6);
	}
	else
	{
		HalExecElement* elReset = new HalExecElement(ID_ReadMemWordsXv2);
		elReset->appendInputData32(0xFFFE);
		elReset->appendInputData32(1);

		HalExecCommand readReset;
		readReset.elements.emplace_back(elReset);
		if (!this->send(readReset))
		{
			return -1;
		}

		*pc = elReset->getOutputAt16(0);
		*sr = 0;
	}

	this->deviceIdPtr = elDeviceIdPtr->getOutputAt32(0);

	MSP430_device_idCode idCode;
	if (!isXv2)
	{
		const uint32_t idDataAddr = elDeviceIdPtr->getOutputAt32(4);
		if (idDataAddr == 0)
		{
			return -1;
		}

		HalExecElement* elId = new HalExecElement(ID_ReadMemWords);
		elId->appendInputData32(idDataAddr);
		elId->appendInputData32(8);

		HalExecCommand devIdCmd;
		devIdCmd.elements.emplace_back(elId);

		HalExecElement* elFuses = new HalExecElement(ID_GetFuses);

		devIdCmd.elements.emplace_back(elFuses);

		if (!this->send(devIdCmd))
		{
			return -1;
		}
		idCode.verId = elId->getOutputAt16(0);
		idCode.verSubId = 0x0000;
		idCode.revisison = elId->getOutputAt8(2);
		idCode.fab = elId->getOutputAt8(3);
		idCode.self = elId->getOutputAt16(4);
		idCode.config = (char)(elId->getOutputAt8(13) & 0x7F);
		idCode.fuses = elFuses->getOutputAt8(0);
		idCode.activationKey = 0;
	}
	else // must be xv2 CPU device 99, 95, 91
	{
		if (this->deviceIdPtr == 0)
		{
			return -1;
		}

		HalExecElement* elId = new HalExecElement(ID_ReadMemQuickXv2);
		elId->appendInputData32(this->deviceIdPtr);
		elId->appendInputData32(4);
		elId->appendInputData32(*pc);

		HalExecCommand devDesc;
		devDesc.elements.emplace_back(elId);
		if (!this->send(devDesc))
		{
			return -1;
		}
		uint8_t info_len = elId->getOutputAt8(0);

		// get device ID was read out with comand before
		idCode.verId = (elId->getOutputAt8(5) << 8) + elId->getOutputAt8(4);
#ifndef NDEBUG
		printf("version ID (0x91): %4x\n", idCode.verId);
#endif
		idCode.verSubId = 0;
		idCode.revisison = elId->getOutputAt8(6);    // HW Revision
		idCode.config = elId->getOutputAt8(7); // SW Revision

		idCode.verSubId = 0x0000; // init with zero = no sub id

		//Sub ID should be everything but not -1
		int16_t SubId= getSubID(info_len, deviceIdPtr, *pc);
		if (SubId != -1)
		{
			idCode.verSubId = (uint16_t)SubId;
		}
		else
		{
			// in case we have an error during sub id detection
			return -1;
		}
		HalExecCommand eemv;
		HalExecElement* elEem = new HalExecElement(ID_EemDataExchangeXv2);
		elEem->appendInputData8(1);
		elEem->appendInputData8(0x87);
		eemv.elements.emplace_back(elEem);
		if (!this->send(eemv))
		{
			return -1;
		}
		eemVersion = elEem->getOutputAt32(0);

		idCode.fab = 0x55;
		idCode.self = 0x5555;

		idCode.fuses = 0x55;
		idCode.activationKey = activationKey;
	}
   	TemplateDeviceDbManagerExt db;
	return (long)db.queryDb(idCode);
};

int16_t DeviceHandleV3::getSubID(uint32_t info_len, uint32_t deviceIdPtr, uint32_t pc)
{
	const int UNDEFINED_00 = 0x00;
	if ((info_len > 1)&&(info_len < 11))
	{
		int tlv_size=4*((int)pow(2.0, (double)info_len))-2;
		HalExecCommand cmd;
		HalExecElement* el = new HalExecElement(ID_ReadMemQuickXv2);
		el->appendInputData32(deviceIdPtr);
		el->appendInputData32(tlv_size/2);
		el->appendInputData32(pc);
		el->setOutputSize(tlv_size);

		cmd.elements.emplace_back(el);
		if (!this->send(cmd))
		{
			return -1;
		}

		const std::vector<uint8_t>& tlv = el->getOutput();
		int pos = 8;
		//Must have at least 2 byte data left
		while (pos + 3 < tlv_size)
		{
			const uint8_t tag = tlv[pos++];
			const uint8_t len = tlv[pos++];
			const uint8_t *value = &tlv[pos];
			pos += len;

			const int SUBVERSION_TAG = 0x14;
			const int UNDEFINED_FF = 0xFF;

			if (tag == SUBVERSION_TAG)
			{
				return (value[1]<<8) + value[0];
			}
			if ((tag == UNDEFINED_FF) || (tag == UNDEFINED_00) || (tag == SUBVERSION_TAG))
			{
				return UNDEFINED_00;
			}
		}
	}
	return UNDEFINED_00;
}

uint32_t DeviceHandleV3::getDeviceCode() const
{
	return this->deviceCode;
}

uint32_t DeviceHandleV3::getJtagId()
{
	return this->jtagId;
}

uint32_t DeviceHandleV3::getDeviceIdPtr()
{
	return this->deviceIdPtr;
}

uint32_t DeviceHandleV3::getEemVersion()
{
	return this->eemVersion;
}

uint32_t DeviceHandleV3::readJtagId()
{

	HalExecElement* el = new HalExecElement(ID_GetJtagId);

	HalExecCommand cmd;
	cmd.elements.emplace_back(el);
	if (!this->send(cmd))
	{
		return 0;
	}

	uint16_t Id = el->getOutputAt16(0);

#ifndef NDEBUG
	printf("JtagID: %2x\n", Id);
#endif
	if (jtagIdIsValid(Id))
	{
		uint16_t wdtAddress = jtagIdIsXv2(Id) ? WDT_ADDR_XV2 : WDT_ADDR_CPU;
		if (Id == 0x98)
		{
			wdtAddress = WDT_ADDR_FR41XX;
		}

		this->setWatchdogControl( std::make_shared<WatchdogControl>(wdtAddress) );
		this->jtagId = Id;
	}
	return Id;
};

bool DeviceHandleV3::isJtagFuseBlown()
{
	HalExecCommand cmd;
	HalExecElement* el = new HalExecElement(ID_IsJtagFuseBlown);
	cmd.elements.emplace_back(el);

	if (!send(cmd))
	{
		return false;
	}
	if (el->getOutputAt16(0) == 0x5555)			// If the fuse is blown, the device will be in JTAG bypass mode, and data
	{											// input to the device will be echoed with a one-bit delay.
		return true; // Fuse is blown.
	}
	else
	{
		return false; // Fuse blow error.
	}
}


bool DeviceHandleV3::secure ()
{
	HalExecCommand cmd;
	HalExecElement* el = nullptr;
	if (this->checkHalId(ID_BlowFuse) == ID_DummyMacro)
	{
		throw SECURE_NOT_SUPPORTED_ERR;
	}
	else if (this->checkHalId(ID_BlowFuse) == ID_BlowFuse)
	{
		el = new HalExecElement(this->checkHalId(ID_BlowFuse));
		el->appendInputData8(this->hasTestVpp ? 1: 0);
		cmd.setTimeout(10000);	// overwrite 3 sec default with 10 sec
		cmd.elements.emplace_back(el);
	}
	else if (this->checkHalId(ID_BlowFuse) == ID_BlowFuseFram)
	{
		const uint32_t jtagSignatureAddress = (description.find("RF430FRL15") == 0) ? 0xFFD0 : 0xFF80;
		uint8_t lockKey[4] = { 0x55, 0x55, 0x55, 0x55 };

		if (!memoryManager->write(jtagSignatureAddress, lockKey, 4) || !memoryManager->sync())
		{
			return false;
		}

		//Trigger BOR via test register
		send(JtagShifts(HIL_CMD_JTAG_IR, 0x2A, 8)(HIL_CMD_JTAG_DR, 0x200, 32));
	}
	else if (this->checkHalId(ID_BlowFuse) == ID_BlowFuseXv2)
	{
		if (!(memoryManager->uploadFunclet(FuncletCode::WRITE)))
		{
			return false;
		}
		MemoryManager* mm = this->getMemoryManager();
		MemoryArea* ram = mm->getMemoryArea(MemoryArea::RAM, 0);
		const FuncletCode& funclet = this->getFunclet(FuncletCode::WRITE);

		const uint32_t type = 0;
		const uint32_t lenght =2; 	// 2 words
		const uint16_t flags = 0xA508;
		const uint16_t programStartAddress = ram->getStart() + funclet.programStartOffset();

		el = new HalExecElement(this->checkHalId(ID_BlowFuse));
		cmd.setTimeout(15000);	// overwrite 3 sec default with 10 sec

		el->appendInputData16(static_cast<uint16_t>(ram->getStart() & 0xFFFF));
		el->appendInputData16(static_cast<uint16_t>(ram->getSize() & 0xFFFF));
		el->appendInputData16(programStartAddress);
		el->appendInputData32(static_cast<uint32_t>(0x17FC));
		el->appendInputData32(lenght);
		el->appendInputData16(type);
		el->appendInputData16(flags);
		el->appendInputData16(0);
		el->appendInputData16(0);
		cmd.elements.emplace_back(el);
	}
	if (el && !send(cmd))
	{
		unsigned short error = el->getOutputAt16(0);
		if (error == API_CALL_NOT_SUPPORTED)
			throw INTERFACE_SUPPORT_ERR;

		return false;
	}

	ConfigManager* cm = parent->getConfigManager();
	cm->setDeviceCode(0);
	cm->setPassword("");
	cm->start();
	return this->isJtagFuseBlown();
}


FetControl* DeviceHandleV3::getControl ()
{
	return parent->getControl();
}

bool DeviceHandleV3::send (HalExecCommand &command)
{
	return this->parent->getControl()->send(command);
}

void DeviceHandleV3::setWatchdogControl (std::shared_ptr<WatchdogControl> ctrl)
{
	this->wdt = ctrl;
}

std::shared_ptr<WatchdogControl> DeviceHandleV3::getWatchdogControl() const
{
	return this->wdt;
}

hal_id DeviceHandleV3::checkHalId(hal_id base_id) const
{
	DeviceInfo::function_map_type::const_iterator it = map.find(base_id);
	return (it != map.end()) ? it->second : base_id;
}

const FuncletCode& DeviceHandleV3::getFunclet(FuncletCode::Type funclet)
{
	return funcletTable[funclet];
}

bool DeviceHandleV3::supportsQuickMemRead() const
{
	return quickMemRead;
}

uint16_t DeviceHandleV3::getMinFlashVcc() const
{
	return minFlashVcc;
}

bool DeviceHandleV3::hasFram() const
{
	return deviceHasFram;
}

bool DeviceHandleV3::hasLPMx5() const
{
	return deviceHasLPMx5;
}

void DeviceHandleV3::disableHaltOnWakeup()
{
	sendDeviceConfiguration(CONFIG_PARAM_TESTREG_ENABLE_LPMX5, this->powerTestRegDefault);
	sendDeviceConfiguration(CONFIG_PARAM_TESTREG3V_ENABLE_LPMX5, this->powerTestReg3VDefault);

	sendDeviceConfiguration(CONFIG_PARAM_TESTREG_DISABLE_LPMX5, this->powerTestRegDefault);
	sendDeviceConfiguration(CONFIG_PARAM_TESTREG3V_DISABLE_LPMX5, this->powerTestReg3VDefault);
}

bool DeviceHandleV3::eemAccessibleInLpm() const
{
	const uint32_t eemPollMacro = checkHalId(ID_WaitForEem);
	return (eemPollMacro != ID_PollJStateReg) && (eemPollMacro != ID_PollJStateReg20);
}

bool DeviceHandleV3::deviceSupportsEnergyTrace() const
{
	const uint32_t eemPollMacro = checkHalId(ID_WaitForEem);
	return (eemPollMacro == ID_PollJStateReg);
}
