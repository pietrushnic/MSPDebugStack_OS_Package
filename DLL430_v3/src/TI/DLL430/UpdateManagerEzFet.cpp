/*
 * UpdateManagerEzFet.cpp
 *
 * Functionality for updating eZ-FET debugger
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

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#endif

#include <boost/thread/thread.hpp>

#include <iostream>
#include <iomanip>
#include <math.h>

#include <BSL430_DLL/Connections/MSPBSL_Connection5xxUSB.h>
#include <BSL430_DLL/MSPBSL_Factory.h>
#include <BSL430_DLL/Utility_Classes/MSPBSL_CRCEngine.h>

#include "VersionInfo.h"
#include "FetHandleV3.h"
#include "FetHandleManager.h"
#include "ConfigManagerV3.h"
#include "UpdateManagerEzFet.h"
#include "DeviceDbManagerExt.h"
#include "DeviceInfo.h"
#include "HalExecCommand.h"
#include "FetControl.h"
#include "WatchdogControl.h"
#include "Record.h"
#include "DeviceHandleV3.h"

#include "../../Bios/include/eZ_FetDcdc.h"
#include "../../Bios/include/eZ_FetHal.h"
#include "../../Bios/include/eZ_FetHil.h"
#include "../../Bios/include/eZ_FetCore.h"
#include "../../Bios/include/eZ_FetComChannel.h"
#include "../../Bios/include/eZ_FetDcdcController.h"

#include "../../Bios/include/ConfigureParameters.h"

using namespace TI::DLL430;
using namespace std;

static string UpdateLog;

UpdateManagerEzFet::UpdateManagerEzFet(FetHandleV3* fetHandle, ConfigManagerV3* configManagerV3, FetHandleManager* fhManager)
 : fetHandle(fetHandle)
 , configManagerV3(configManagerV3)
 , fetHandleManager(fhManager)
{
}

UpdateManagerEzFet::~UpdateManagerEzFet() {}

void restartFet(auto_ptr<MSPBSL_Connection5xxUSB> eZ_FET)
{
	uint8_t Buffer2[2]; 
	Buffer2[0] = 0xA5;
	Buffer2[1] = 0x04;
	eZ_FET->RX_DataBlockFast(Buffer2,0x0120,2);
	eZ_FET->RX_DataBlockFast(Buffer2,0x0120,2);
}

bool UpdateManagerEzFet::updateCore(FileFuncImpl &firmware)
{
	try
	{
		UpdateLog.append("----TRACE---------------eZ_FET start BSL update------------------------------;\n");
		string verString = "";
	
		// erase reset vector of core
		upCoreErase();		
		boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(4));

		fetHandle->shutdown();
		boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(1));

		UpdateLog.append("----TRACE----fetHandle->shutdown()\n");
		auto_ptr<MSPBSL_Connection5xxUSB> eZ_FET(dynamic_cast<MSPBSL_Connection5xxUSB*>(MSPBSL_Factory::getMSPBSL_Connection("DEVICE:MSP430F5528 VID:0x2047 PID:0x0203")));

		UpdateLog.append("----TRACE----auto_ptr<MSPBSL_Connection5xxUSB> eZ_FET\n");

		if(eZ_FET->loadRAM_BSL() != 0)
		{
			/*Reset FET*/
			eZ_FET->closeBslconnection();
			verString = "BUG";
			UpdateLog.append("----TRACE----eZ_FET->loadRAM_BSL() != 0 \n");
			return false;
		}
	
		eZ_FET->TX_BSL_Version(verString);
		UpdateLog.append("----TRACE----eZ_FET->TX_BSL_Version(verString);\n");

		eZ_FET->massErase();	
		UpdateLog.append("----TRACE----eZ_FET->massErase();\n");

		if((firmware.getNumberOfSegments())==0)
		{
			return false;
		}
	
		for (size_t i = 0; i < firmware.getNumberOfSegments(); i++)
		{
			const DownloadSegment *seg = firmware.getFirmwareSeg(i);
			
			if (seg == NULL)
			{
				UpdateLog.append("----TRACE----eZ_FET end BSL update faild\n");
				return false;
			}
			
			vector<uint8_t> Buffer(seg->size);			
		
			MSPBSL_CRCEngine crcEngine("5xx_CRC");
			crcEngine.initEngine(0xFFFF);

			const uint32_t padding = seg->size % 2;
			const uint32_t data2send = seg->size + padding;
		
			for(uint32_t n=0; n < seg->size; n++)
			{
				Buffer[n]= (seg->data[n] & 0xff);
				crcEngine.addByte(seg->data[n] & 0xff);
			}
			for(uint32_t p=0 ; p < padding; p++)
			{
				Buffer[p] = (0xff);
				crcEngine.addByte(0xff);
			}	

			eZ_FET->RX_DataBlockFast(&Buffer[0], (uint32_t)seg->startAddress&0xfffffffe,(uint16_t)seg->size);

			uint16_t currentCoreCRC[1] = {0}; 
			eZ_FET->CRC_Check(currentCoreCRC, (uint32_t)seg->startAddress&0xfffffffe, seg->size);

			uint32_t expectedCoreCRC = crcEngine.getHighByte()<<8;
			expectedCoreCRC |= crcEngine.getLowByte();

			if(expectedCoreCRC != currentCoreCRC[0])
			{
				if(i != 0)// just debug exeption
				{				
					eZ_FET->closeBslconnection();
					UpdateLog.append("----TRACE----eZ_FET end BSL update faild\n");
					return false;
				}
			}
		}
		UpdateLog.append("----TRACE---------------eZ_FET end BSL update------------------------------;\n");
		eZ_FET->closeBslconnection();
	}
	catch(...)
	{
		UpdateLog.append("----TRACE----eZ_FET end BSL update faild\n");
		return false;	
	}

	return true;
}


bool UpdateManagerEzFet::isUpdateRequired() const
{
 	bool isUpdateRequired = false;
	if (checkHalVersion() != 0)
	{
		isUpdateRequired = true;
	}
	if (checkCoreVersion() != 0)
	{
		isUpdateRequired = true;
	}
	if (checkDcdcLayerVersion() != 0)
	{
		isUpdateRequired = true;
	}
	if (checkDcdcSubMcuVersion() != 0)
	{
		isUpdateRequired = true;
	}
	if (checkHilVersion() != 0)
	{
		isUpdateRequired = true;
	}
	if (checkUartVersion() != 0)
	{
		isUpdateRequired = true;
	}
	return isUpdateRequired;
}

uint16_t UpdateManagerEzFet::checkHilVersion() const
{
	FetControl * control=this->fetHandle->getControl();
	//get current hil version from FET
	const uint32_t currentHilVersion = control->getHilVersion();
	uint16_t expectedHilVersion = 0;
	//get hil CRC from FET
	const uint16_t currentHilCrc= control->getFetHilCrc();
	uint16_t expectedHilCrc  = 0;

	Record fetHil(eZ_FetHilImage, eZ_FetHilImage_address, eZ_FetHilImage_length_of_sections, eZ_FetHilImage_sections);

	FileFuncImpl firmware;
	firmware.readFirmware(eZ_FetHilImage, eZ_FetHilImage_address, eZ_FetHilImage_length_of_sections, eZ_FetHilImage_sections);

	//if hil versions or CRC's do not match, update hil
	if(fetHil.getWordAtAdr(0x18F6, &expectedHilVersion) && fetHil.getWordAtAdr(0x18FA, &expectedHilCrc))
	{
		if((expectedHilVersion != currentHilVersion) || (expectedHilCrc != currentHilCrc))
		{
			return 1;
		}
	}
	return 0;
}

uint16_t UpdateManagerEzFet::checkUartVersion() const
{
	FetControl * control=this->fetHandle->getControl();
	const uint32_t currentUartVersion = control->getFetComChannelVersion();
	uint16_t expectedUartVersion = 0;

	const uint16_t currentFetComChannelCrc= control->getFetComChannelCrc();
	uint16_t expectedFetComChannelCRC  = 0;

	Record fetComChannelVersion (eZ_FetComChannelImage, eZ_FetComChannelImage_address, eZ_FetComChannelImage_length_of_sections, eZ_FetComChannelImage_sections);

	FileFuncImpl firmware;
	firmware.readFirmware(eZ_FetComChannelImage, eZ_FetComChannelImage_address, eZ_FetComChannelImage_length_of_sections, eZ_FetComChannelImage_sections);

	
	if(fetComChannelVersion.getWordAtAdr(0x1984, &expectedUartVersion) && fetComChannelVersion.getWordAtAdr(0x19FA, &expectedFetComChannelCRC))
	{
		if((expectedUartVersion != currentUartVersion) || (expectedFetComChannelCRC != currentFetComChannelCrc))
		{
			return 1;
		}
	}
	return 0;
}

uint16_t UpdateManagerEzFet::checkDcdcLayerVersion()const 
{
	FetControl * control=this->fetHandle->getControl();
	//get current dcdc layer version from FET
	const uint32_t currentDcdcLayerVersion = control->getDcdcLayerVersion();
	uint16_t expectedDcdcLayerVersion  = 0;
	//get dcdc layer CRC from FET
	const uint16_t currentDcdcCrc= control->getFetDcdcCrc();
	uint16_t expectedDcdcCrc  = 0;

	Record fetDcdcLayer(eZ_FetDcdcImage, eZ_FetDcdcImage_address, eZ_FetDcdcImage_length_of_sections, eZ_FetDcdcImage_sections);

	FileFuncImpl firmware;
	firmware.readFirmware(eZ_FetDcdcImage, eZ_FetDcdcImage_address, eZ_FetDcdcImage_length_of_sections, eZ_FetDcdcImage_sections);
	
	//if dcdc layer versions or CRC's do not match, update dcdc layer
	if(fetDcdcLayer.getWordAtAdr(0x1804, &expectedDcdcLayerVersion) && fetDcdcLayer.getWordAtAdr(0x187A,&expectedDcdcCrc))
	{
		if((expectedDcdcLayerVersion != currentDcdcLayerVersion) || (expectedDcdcCrc != currentDcdcCrc))
		{
			return 1;
		}
	}
	return 0;
}

uint16_t UpdateManagerEzFet::checkDcdcSubMcuVersion() const
{
	FetControl * control=this->fetHandle->getControl();
	const uint32_t currentDcdcSubMcuVersion = control->getDcdcSubMcuVersion();
	uint16_t expectedDcdcSubMcuVersion = 0;
	
	Record fetBugConverterVersion(eZ_FetDcdcControllerImage,
				eZ_FetDcdcControllerImage_address, eZ_FetDcdcControllerImage_length_of_sections,
				eZ_FetDcdcControllerImage_sections);

	if(control->getFetToolId() == ConfigManager::eZ_FET_NO_DCDC)
	{
		return 0;
	}

	if(fetBugConverterVersion.getWordAtAdr(0x1000, &expectedDcdcSubMcuVersion))
	{
		//if dcdc sub-mcu versions do not match, update sub-mcu
		if(currentDcdcSubMcuVersion != expectedDcdcSubMcuVersion)
		{
			return 1;
		}
	}
	return 0;
}

uint16_t UpdateManagerEzFet::checkHalVersion() const
{
	FetControl * control=this->fetHandle->getControl();
	//get hal CRC from FET
	const uint16_t currentHalCrc= control->getFetHalCrc();
	uint16_t expectedHalCrc  = 0;

	FileFuncImpl firmware;
	firmware.readFirmware(eZ_FetHalImage, eZ_FetHalImage_address, eZ_FetHalImage_length_of_sections, eZ_FetHalImage_sections);

	Record fetHal(eZ_FetHalImage, eZ_FetHalImage_address, eZ_FetHalImage_length_of_sections, eZ_FetHalImage_sections);

	//if hal versions or CRC's do not match, update Hal
	if(fetHal.getWordAtAdr(0x197A, &expectedHalCrc))
	{
		if(expectedHalCrc != currentHalCrc)
		{
			return 1;
		}
		return 0;
	}
	return 1;
}


uint16_t UpdateManagerEzFet::checkCoreVersion() const
{
	FetControl * control=this->fetHandle->getControl();
	
	//get current core version from FET
	const uint16_t actualFetCoreVersion = control->getFetCoreVersion();
	uint16_t expectedFetCoreVersion = 0;
	//get core CRC from FET
	const uint16_t currentCoreCrc= control->getFetCoreCrc();
	uint16_t expectedCoreCrc  = 0;

	FileFuncImpl firmware;
	firmware.readFirmware(eZ_FetCoreImage, eZ_FetCoreImage_address, eZ_FetCoreImage_length_of_sections, eZ_FetCoreImage_sections);
	
	Record fetCore(eZ_FetCoreImage, eZ_FetCoreImage_address, eZ_FetCoreImage_length_of_sections, eZ_FetCoreImage_sections);
	//get core version from image (core version is stored in address 0x4404)
	if(fetCore.getWordAtAdr(0x4404, &expectedFetCoreVersion) && fetCore.getWordAtAdr(0x4402, &expectedCoreCrc))
	{
		//if core versions or CRC's do not match, update core
		if((expectedFetCoreVersion != actualFetCoreVersion) || (currentCoreCrc != expectedCoreCrc))
		{
			return 1;
		}
	}
	return 0;
}

VersionInfo UpdateManagerEzFet::getHalVersion() const
{
	FetControl * control=this->fetHandle->getControl();
	std::vector<uint8_t> * sw_info = this->fetHandle->getSwVersion();
	const uint16_t currentHalCrc= control->getFetHalCrc();
	uint16_t expectedHalCrc  = 0;

	Record fetHal(eZ_FetHalImage, eZ_FetHalImage_address, eZ_FetHalImage_length_of_sections, eZ_FetHalImage_sections);

	if(fetHal.getWordAtAdr(0x197A, &expectedHalCrc))
	{
		if(expectedHalCrc != currentHalCrc)
		{
			return VersionInfo(1, 0, 0, 0);
		}
	}
	if(sw_info==NULL)
	{
		return VersionInfo(0, 0, 0, 0);
	}
	if(sw_info->size()<4)
	{
		return VersionInfo(0, 0, 0, 0);
	}
	unsigned char major=sw_info->at(1);
	return VersionInfo((((major&0xC0)>>6)+1),(major&0x3f),sw_info->at(0), 
		(sw_info->at(3)<<8)+sw_info->at(2));
}


bool UpdateManagerEzFet::firmWareUpdate(const char* fname, UpdateNotifyCallback callback, bool* coreUpdate)
{
	FetControl* control = this->fetHandle->getControl();
	bool returnValue = true;

	if(control == NULL)
	{
		return false;
	}

	const uint32_t halVersion = getHalVersion().get();
		
	UpdateLog.clear();

	UpdateLog.append("\n\n\n ------------------------Start Firmware update--------------------------- \n");

	if(checkCoreVersion() != 0)
	{	
		*coreUpdate = true;
		FileFuncImpl firmware;
		firmware.readFirmware(eZ_FetCoreImage, eZ_FetCoreImage_address, eZ_FetCoreImage_length_of_sections, eZ_FetCoreImage_sections);
		UpdateLog.append("----TRACE----call updateCore(firmware)\n");
		return updateCore(firmware);
	}
	
	// just for core update test do not call during normal debug
	if(fname)
	{
		if (string(fname).find("CORE_RST_VECTOR_ERASE") != string::npos)
		{
			upCoreErase();
			if(callback)
			{
				callback(BL_INIT,0,0);
				callback(BL_DATA_BLOCK_PROGRAMMED,100,0);
				callback(BL_UPDATE_DONE,0,0);
				callback(BL_EXIT,0,0);
			}
			UpdateLog.append("----TRACE----CORE_RST_VECTOR_ERASE done\n");
			return true;
		}
	}

	if (fname)
	{
		FileFuncImpl firmware;
		if (!firmware.readFirmware(fname))
		{
			UpdateLog.append("----TRACE--- firmware.readFirmware(fname)faild \n");	
			return false;
		}
		if(callback)
		{
			callback(BL_INIT, 0, 0);
			callback(BL_PROGRAM_FIRMWARE, 0, 0);
			callback(BL_DATA_BLOCK_PROGRAMMED, 0, 0);
		}
		returnValue = updateFirmware(firmware);
		if(!returnValue)
		{
			UpdateLog.append("----TRACE--- returnValue = updateFirmware(firmware) faild  \n");	
		}

		if(callback)
		{
			callback(BL_DATA_BLOCK_PROGRAMMED, 100, 0);
			callback(BL_UPDATE_DONE,0,0);
			callback(BL_EXIT,0,0);
		}
	}
	else
	{
		uint32_t requiredUpdates = 0;
		bool subMcuUpdate = false;

		if(checkDcdcLayerVersion() != 0)
		{
			requiredUpdates++;			
		}	
		if (checkDcdcSubMcuVersion() != 0)
		{
			requiredUpdates++;
			subMcuUpdate = true;
		}
		if(checkHilVersion() != 0)
		{
			requiredUpdates++;
		}
		if(halVersion == 10000000)// to do find out wher it comes from 
		{
			requiredUpdates++;
		}	
		if(checkUartVersion() != 0)
		{
			requiredUpdates++;
		}

		uint32_t percent = 0;	
		if(requiredUpdates)
		{
			percent = 100/requiredUpdates;
		}
		if(callback)
		{
			callback(BL_INIT, 0, 0);
			callback(BL_PROGRAM_FIRMWARE, 0, 0);
		}	
	
		if(checkHilVersion() != 0)
		{
			FileFuncImpl firmware;
			firmware.readFirmware(eZ_FetHilImage, eZ_FetHilImage_address, eZ_FetHilImage_length_of_sections, eZ_FetHilImage_sections);
			
			returnValue = updateFirmware(firmware);
			if(!returnValue)
			{
				UpdateLog.append("----TRACE----HilLayer update faild\n");
			}
			control->resetCommunication();
			control->setObjectDbEntry(0);

			if(callback)
			{
				callback(BL_DATA_BLOCK_PROGRAMMED, 100-(--requiredUpdates)*percent, 0);
			}
		}
		if(halVersion == 10000000)
		{
			FileFuncImpl firmware;
			firmware.readFirmware(eZ_FetHalImage, eZ_FetHalImage_address, eZ_FetHalImage_length_of_sections, eZ_FetHalImage_sections);

			returnValue = updateFirmware(firmware);
			if(!returnValue)
			{
				UpdateLog.append("----TRACE----HalLayer update faild\n");
			}
		
			control->resetCommunication(); // just workaround... 2 times
			control->setObjectDbEntry(0);
			control->resetCommunication();
			control->setObjectDbEntry(0);

			if(callback)
			{
				callback(BL_DATA_BLOCK_PROGRAMMED, 100-(--requiredUpdates)*percent, 0);
			}
		}

		//--------------------------------------SubMcuUpate-----------------------------------------------------
		if(subMcuUpdate && returnValue && (control->getFetToolId() != ConfigManager::eZ_FET_NO_DCDC))
		{
			returnValue = true;

			DeviceHandleManager* dhm = this->fetHandle->getDeviceHandleManager();		
			DeviceChainInfoList* dcil = dhm->getDeviceChainInfo();

			// the first device is the only one
			DeviceChainInfoList::iterator itsdcil = dcil->begin();

			// if we are alreaddy connected to a device.
			if(itsdcil->isInUse()) 
			{	
				itsdcil->setInUse(false);
			}
			//-----------------------------------------
			configManagerV3->setJtagMode(ConfigManager::JTAG_MODE_SPYBIWIRE_SUBMCU);
			if(!configManagerV3->start())
			{
				UpdateLog.append("----TRACE---- configManagerV3->start() \n");
			}

			// save the device handle to work with
			DeviceHandle* singleDevice = dhm->createDeviceHandle(itsdcil,0);

			if (singleDevice && singleDevice->getJtagId() != 0x89)
			{
				returnValue = false;
				UpdateLog.append("----TRACE---- singleDevice->getJtagId() != 0x89 \n");
			}

			// sanity check
			if (singleDevice == NULL)
			{
				configManagerV3->stop();
				returnValue = false;
				UpdateLog.append("----TRACE---- singleDevice==NULL \n");
			}

			if (returnValue)
			{
				const long setId = singleDevice->identifyDevice(0);
				if(setId == -5555)
				{
					returnValue = false;
					UpdateLog.append("----TRACE---- Fuse Blown\n");		
				}
				else if (setId < 0)
				{
					returnValue = false;
					UpdateLog.append("----TRACE----No device detected\n");		
				}
			}
			
			if (returnValue)
			{
				// Now programm the Sub MCU
				returnValue = returnValue && this->upInit(2); 
				if(!returnValue)
				{
					UpdateLog.append("----TRACE----upInit(2) failed \n");
				}

				returnValue = returnValue && this->programmSubMcu(singleDevice);
				if(!returnValue)
				{
					UpdateLog.append("----TRACE----programm the Sub MCU update failed \n");
				}

				returnValue = returnValue && this->upInit(0);
				if(!returnValue)
				{
					UpdateLog.append("----TRACE----upInit(0) failed \n");
				}

				if(!configManagerV3->stop())
				{
					UpdateLog.append("----TRACE----Stop JTAG done failed \n");
				}

				// destroy device handle
				dhm->destroyDeviceHandle(singleDevice);
				singleDevice = NULL;
				itsdcil->setInUse(false);

				control->resetCommunication();	
				control->setObjectDbEntry(0);

				if(callback)
				{
					callback(BL_DATA_BLOCK_PROGRAMMED, 100-(--requiredUpdates)*percent, 0);
				}
			}
		}
		
		if(checkDcdcLayerVersion() != 0)
		{
			FileFuncImpl firmware;
			firmware.readFirmware(eZ_FetDcdcImage, eZ_FetDcdcImage_address, eZ_FetDcdcImage_length_of_sections, eZ_FetDcdcImage_sections);
			
			returnValue = updateFirmware(firmware);
			if(!returnValue)
			{
				UpdateLog.append("----TRACE----DcdcLayer update failed \n");
			}
			control->resetCommunication();
			control->setObjectDbEntry(0);

			if(callback)
			{
				callback(BL_DATA_BLOCK_PROGRAMMED, 100-(--requiredUpdates)*percent, 0);
			}			
		}		
		if(checkUartVersion() != 0)
		{
			FileFuncImpl firmware;
			firmware.readFirmware(eZ_FetComChannelImage, eZ_FetComChannelImage_address, eZ_FetComChannelImage_length_of_sections, eZ_FetComChannelImage_sections);
			
			returnValue = updateFirmware(firmware);
			if(!returnValue)
			{
				UpdateLog.append("----TRACE----Uart Layer update failed \n");
			}

			control->resetCommunication();
			control->setObjectDbEntry(0);
			
			if(callback)
			{
				callback(BL_DATA_BLOCK_PROGRAMMED,(100-(--requiredUpdates)*percent),0);
			}
		}
	}
	if(callback)
	{
		callback(BL_DATA_BLOCK_PROGRAMMED,100,0);
		callback(BL_UPDATE_DONE,0,0);
		callback(BL_EXIT,0,0);
	}

	if(!returnValue)
	{
#if defined(_WIN32) || defined(_WIN64)
		char binaryPath[256] = {0};
		uint32_t pathLength = 0;

		pathLength = GetModuleFileName(0, binaryPath, sizeof(binaryPath));
		while (pathLength > 0 && binaryPath[pathLength-1] != '\\')
		{
			--pathLength;
		}
		string logfile = string(binaryPath, pathLength) + "Update.log";

		UpdateLog.append("\n---------------------Firmware upate end--------------------------\n");

		ofstream(logfile.c_str(), ios::app | ios::out) << UpdateLog;
#endif
	}
	return returnValue;
}

bool UpdateManagerEzFet::programmSubMcu(DeviceHandle * singleDevice)
{
	FileFuncImpl firmware;
	firmware.readFirmware(eZ_FetDcdcControllerImage,
				eZ_FetDcdcControllerImage_address, eZ_FetDcdcControllerImage_length_of_sections,
				eZ_FetDcdcControllerImage_sections);

	if((firmware.getNumberOfSegments())==0)
	{
		return false;
	}
	if(!singleDevice)
	{
		UpdateLog.append("----TRACE---- SUB mcu !singleDevice\n");
		return false;
	}
	MemoryManager* mm = singleDevice->getMemoryManager();

	if(!mm)
	{
		UpdateLog.append("----TRACE---- SUB mcu !mm\n");
		return false;
	}
	MemoryArea* main = mm->getMemoryArea("main");

	singleDevice->reset();

	//erase sub MCU. 
	bool eraseSubMcuMain = main->erase();
	if(!eraseSubMcuMain)
	{	
		UpdateLog.append("----TRACE---- SUB mcu !eraseSubMcuMain\n");
		return false;
	}

	MemoryArea* info = mm->getMemoryArea("information");
	singleDevice->reset();

	//erase sub MCU. 
	bool eraseSubMcuinfo = info->erase();
	if(!eraseSubMcuinfo)
	{	
		UpdateLog.append("----TRACE---- SUB mcu !eraseSubMcuinfo\n");
		return false;
	}
	
	singleDevice->reset();

	bool writeSubMcu = firmware.writeSegs(singleDevice);
	if(!writeSubMcu)
	{
		UpdateLog.append("----TRACE---- SUB mcu !writeSubMcu = firmware.writeSegs(singleDevice\n");
	}
	firmware.close();
	return writeSubMcu;
}

bool UpdateManagerEzFet::updateFirmware(const FileFuncImpl &firmware)
{
	if((firmware.getNumberOfSegments())==0)
	{
		return false;
	}
	// start HAL update routine

	if(!this->upInit(1))
	{
		return false;
	}	
	if(!this->upErase(firmware))
	{
		return false;
	}
	if(!this->upWrite(firmware))
	{
		return false;
	}
	if(!this->upInit(0))
	{
		return false;
	}
	// give the firmware time to execute initialisation
	boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(1));
	return true;
}

bool UpdateManagerEzFet::upInit(unsigned char level)
{
	HalExecCommand updateCmd;
	updateCmd.setTimeout(10000);
	HalExecElement* el = new HalExecElement(ID_Zero, UpInit);
	el->setAddrFlag(false);
	el->appendInputData8(level);
	
	updateCmd.elements.push_back(el);
	this->fetHandle->send(updateCmd);
	return true;
}

bool UpdateManagerEzFet::InitDcdc(unsigned char level)
{
	HalExecCommand updateCmd;
	updateCmd.setTimeout(40000);
	HalExecElement* el = new HalExecElement(ID_Zero, dcdcInitInterface);
	el->setAddrFlag(false);
	el->appendInputData8(level);
	
	updateCmd.elements.push_back(el);
	this->fetHandle->send(updateCmd);
	return true;
}


bool UpdateManagerEzFet::upErase(const FileFuncImpl& firmware)
{
	HalExecCommand updateCmd;
	updateCmd.setTimeout(40000);
	for (size_t i = 0; i < firmware.getNumberOfSegments(); ++i)
	{
		updateCmd.elements.clear();
		HalExecElement* el = new HalExecElement(ID_Zero, UpErase);
		el->setAddrFlag(false);

		const DownloadSegment *seg = firmware.getFirmwareSeg(i);
		if (seg == NULL)
			return false;

		el->appendInputData32(seg->startAddress&0xfffffffe);
		el->appendInputData32(seg->size);

		updateCmd.elements.push_back(el);
		if(this->fetHandle->send(updateCmd)==false)
		{
			return false;
		}
	}
	return true;
}

bool UpdateManagerEzFet::upWrite(const FileFuncImpl& firmware)
{
	HalExecCommand updateCmd;
	updateCmd.setTimeout(45000);
	for (size_t i = firmware.getNumberOfSegments(); i > 0; i--)
	{
		const DownloadSegment *seg = firmware.getFirmwareSeg(i-1);
	
		if (seg == NULL)
		{
			return false;
		}

		updateCmd.elements.clear();
		// create Core telegram -> update write 
		HalExecElement* el = new HalExecElement(ID_Zero, UpWrite); 
		// no HAL id needed for update
		el->setAddrFlag(false);

		const uint32_t padding = seg->size % 2;
		const uint32_t data2send = seg->size + padding;

		// add address data
		el->appendInputData32(seg->startAddress&0xfffffffe);
		el->appendInputData32(data2send);

		// add update data
		for(uint32_t n=0; n < seg->size; n++)
		{
			el->appendInputData8(seg->data[n] & 0xff);
		}
		for(uint32_t p=0 ; p < padding; p++)
		{
			el->appendInputData8(0xff);
		}
		updateCmd.elements.push_back(el);
		if(this->fetHandle->send(updateCmd)==false)
		{
			return false;
		}
	}
	return true;
}


bool UpdateManagerEzFet::upRead(const FileFuncImpl& firmware)
{
	HalExecCommand updateCmd;
	updateCmd.setTimeout(45000);
	for (size_t i = 0; i < firmware.getNumberOfSegments(); ++i)
	{
		const DownloadSegment *seg = firmware.getFirmwareSeg(i);
		if ( seg == NULL )
			return false;

		const uint32_t padding = seg->size % 2;
		const uint32_t data2receive = seg->size + padding;

		updateCmd.elements.clear();

		HalExecElement* el = new HalExecElement(ID_Zero, UpRead);
		// no HAL id needed for update
		el->setAddrFlag(false);	
		el->appendInputData32(seg->startAddress&0xfffffffe);
		el->appendInputData32(data2receive);

		updateCmd.elements.push_back(el);
		if (!this->fetHandle->send(updateCmd))
			return false;

		for (size_t j = 0; j < seg->size; ++j)
		{
			if ( el->getOutputAt8(j) != seg->data[j] )
				return false;
		}
	}
	return true;
}

bool UpdateManagerEzFet::upCoreErase()
{
		FetControl * control=this->fetHandle->getControl();
		// create 0x55 command erase signature
		// command forces safecore to search for new core on reset
		std::vector<uint8_t> data_55;
		data_55.push_back(0x03);
		data_55.push_back(0x55);
		uint8_t id=control->createResponseId();
		data_55.push_back(id);
		data_55.push_back(0x00);

		control->sendData(data_55);
		control->clearResponse();

		
		return true;
}

bool UpdateManagerEzFet::upCoreWrite()
{
		return true;
}

//reads back flashed core data and compares it to data in core image
//returns: true, if data on FET and core image are equal 
bool UpdateManagerEzFet::upCoreRead()
{	
	return true;
}
/*EOF*/
