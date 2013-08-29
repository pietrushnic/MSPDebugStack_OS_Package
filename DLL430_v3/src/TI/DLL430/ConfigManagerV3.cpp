/*
 * ConfigManagerV3.cpp
 *
 * Functionality for configuring target device.
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

#include "VersionInfo.h"
#include "FetHandleV3.h"
#include "ConfigManagerV3.h"
#include "DeviceDbManagerExt.h"
#include "DeviceInfo.h"
#include "HalExecCommand.h"
#include "FetControl.h"
#include "WatchdogControl.h"
#include "Record.h"
#include "PinSequence.h"
#include "DeviceHandleV3.h"
#include "UpdateManagerEzFet.h"
#include "UpdateManagerMSP_FET430.h"
#include "UpdateManagerDummy.h"
#include "FetHandleManager.h"
#include "ConfigureParameters.h"
#include "EnergyTraceManager.h"

#include <boost/thread/thread.hpp>

#include <iostream>
#include <iomanip>
#include <math.h>

using namespace TI::DLL430;
using namespace std;

ConfigManagerV3::ConfigManagerV3 (FetHandleV3* parent, FetHandleManager* fhManager)
 : parent(parent)
 , vcc(0)
 , mode(ConfigManager::JTAG_MODE_4WIRE)
 , deviceCode(0)
 , freqCalibration(true)
 , mEnergyTraceManager(0)
 , mhighres(0)
 , ulpDebug(false)
{
	updateCmd.setTimeout(20000);
	updateManagerFet = 0;

	FetControl* control = this->parent->getControl();
	if(control->getFetToolId()== EZ_FET || control->getFetToolId() == eZ_FET_NO_DCDC )
	{
		this->updateManagerFet = new UpdateManagerEzFet(parent, this, fhManager);
	}
	else if(control->getFetToolId()== MSP_FET430)
	{
		this->updateManagerFet = new UpdateManagerMSP_FET430(parent, this);
	}
	else
	{
		this->updateManagerFet = new UpdateManagerDummy();
	}
}

ConfigManagerV3::~ConfigManagerV3()
{
	FetControl* control=this->parent->getControl();
	
	// send reset command to FET
	if((control != NULL)&&(control->hasCommunication()))
	{
		this->stop();
	}
	delete updateManagerFet;
}

bool ConfigManagerV3::isUpdateRequired() const 
{
	return updateManagerFet->isUpdateRequired();
}

bool ConfigManagerV3::isEnergyTraceSupported()  
{
	FetControl* control = this->parent->getControl();
	if(control->getFetToolId()== EZ_FET)
	{
		return true;
	}	
	return false;
}

bool ConfigManagerV3::init ()
{
	string tag;
	string value;

	// If any kind of update is required do not configure any JTAG or SBW Speed-> perhaps you will use a null pointer
	if(updateManagerFet->isUpdateRequired())
	{
		return true;
	}
	// read configuration for JTAG speed EDT trace.... 
	JTAG_4WIRE_SPEED jtagSpeed = JTAG_4WIRE_SPEED_8_MHZ;
	JTAG_2WIRE_SPEED sbwSpeed  = JTAG_2WIRE_SPEED_600_KHZ;

	string iniFile = "MSP430DLL.INI";

	if (const char* iniPathEnv = getenv("MSP430_DLL_INI_PATH"))
	{
		iniFile = string(iniPathEnv) + "/" + iniFile;
	}
	
	ifstream DllV3Ini(iniFile.c_str());

	while (DllV3Ini && !DllV3Ini.eof())
	{
		DllV3Ini >> tag >> value; 
		if (tag == "SBW_SPEED")
		{
			if (value == "JTAG_2WIRE_SPEED_600_KHZ")
			{
				sbwSpeed = JTAG_2WIRE_SPEED_600_KHZ;
			}
			if (value == "JTAG_2WIRE_SPEED_400_KHZ")
			{
				sbwSpeed = JTAG_2WIRE_SPEED_400_KHZ;
			}
			if (value == "JTAG_2WIRE_SPEED_200_KHZ")
			{
				sbwSpeed = JTAG_2WIRE_SPEED_200_KHZ;
			}
			if (value == "JTAG_2WIRE_SPEED_100_KHZ")
			{
				sbwSpeed = JTAG_2WIRE_SPEED_100_KHZ;
			}
		}
		if (tag == "JTAG_SPEED")
		{
			if (value == "JTAG_4WIRE_SPEED_8_MHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_8_MHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_4_MHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_4_MHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_2_MHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_2_MHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_1_MHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_1_MHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_500_KHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_500_KHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_250_KHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_250_KHZ;
			}
		}
		if (tag == "ULP_DEBUG")
		{
			ulpDebug = (value == "ON");
		}

		if (tag == "DCO_CALIBRATION")
		{
			freqCalibration = (value != "OFF");
		}
	}
	this->setJtagSpeed(jtagSpeed,sbwSpeed);
	return true;

}

VersionInfo ConfigManagerV3::getHalVersion() const
{
	return this->updateManagerFet->getHalVersion();
}


void ConfigManagerV3::setJtagMode (enum ConfigManager::jtagMode mode)
{
	this->mode = mode;
}

uint16_t ConfigManagerV3::start()
{
	return this->start(password, deviceCode);
}

bool ConfigManagerV3::totalErase()
{
	if(this->start() != 0x1)
	{
		return false;
	}
	HalExecCommand cmd;
	HalExecElement* el = new HalExecElement(ID_SendJtagMailboxXv2);
	el->appendInputData16(0x11);
	el->appendInputData16(0xA55A);
	el->appendInputData16(0x1B1B);
	el->appendInputData16(0x1);
	cmd.elements.push_back(el);

	if (!this->parent->send(cmd))
	{
		return false;
	}

	// assert hard RST/NMI and feed in magic pattern to stop device execution
	// thr RST/NMI will remove the register protection
	if(!this->reset(false, 10, 0x99))
	{
		return false;
	}
	// restart jtag connection and if needed feed in JTAG passowrd
	if(this->start() != 0x1)
	{
		return false;
	}
	return true;
}

uint16_t AsciToHex(const char* password)
{
    uint16_t usTemp =0;
	
    for( int i = 3 ; i >= 0; i--)
    {
		char TempLetterToDig = password[i];
        TempLetterToDig = toupper(TempLetterToDig);
        if(TempLetterToDig >= 'A' && TempLetterToDig <= 'F')
        {
            usTemp |= (int)(TempLetterToDig - 7 - '0') << (12-(i*4));
        }
        else
        {
            usTemp |= (int)(TempLetterToDig - '0') << (12-(i*4));
        }      
    }
    return usTemp;
}

ConfigManager::jtagMode ConfigManagerV3::getInterfaceMode () const
{
	HalExecCommand cmd;
	HalExecElement* el = new HalExecElement(ID_GetInterfaceMode);

	el->setOutputSize(4);
	cmd.elements.push_back(el);

	if (!this->parent->send(cmd))
	{
		return JTAG_MODE_UNDEF;
	}
	uint16_t jtagID = el->getOutputAt16(0);
    uint16_t ifMode = el->getOutputAt16(2);
	if (jtagID == 0xFFFF)
	{
		return JTAG_MODE_UNDEF;

	}
	switch (ifMode)
	{
		case 0: // JTAG Mode
			return JTAG_MODE_4WIRE;
		case 1: // SBW2 Mode
			return JTAG_MODE_SPYBIWIRE;
		case 2: // SBW4 Mode
			return JTAG_MODE_4AFTER2;
		default:
			return JTAG_MODE_UNDEF;
	}
	return JTAG_MODE_UNDEF;
}

uint16_t ConfigManagerV3::start(const string& pwd, uint32_t deviceCode)
{
	uint8_t numOfDevices = 0;
	//#words in hex, ie. #characters / 4
	const uint16_t pwLength = (uint16_t)pwd.length() / 4;
	
	// if we have an L092 Rom device
	if (deviceCode == 0xDEADBABE)
	{
		if (pwLength > 4)
		{
			return 0;
		}

		HalExecElement* el = new HalExecElement(ID_UnlockC092);
		el->appendInputData16(pwLength);

		const char* pwdinternal = pwd.c_str();
		for (uint16_t i = 0; i < pwLength; i++)
		{
			uint16_t hexWord = AsciToHex(pwdinternal);		
			el->appendInputData16(hexWord);
			pwdinternal += 4;      
		} 

		el->setOutputSize(1);

		HalExecCommand cmd;
		cmd.elements.push_back(el);

		if (!this->parent->send(cmd))
		{
			return 0;
		}
		numOfDevices = 1;
		return numOfDevices;
	}
	// if we have an L092 device
	if(deviceCode == 0xA55AA55A|| deviceCode == 0x5AA55AA5) 
	{
		HalExecElement* el = new HalExecElement(ID_StartJtagActivationCode);		

		el->appendInputData8(0);
		el->appendInputData8(0);
		el->appendInputData32(deviceCode);
		
		el->setOutputSize(1);

		HalExecCommand cmd;
		cmd.elements.push_back(el);

		if (!this->parent->send(cmd))
		{
			return 0;
		}
		numOfDevices = 1;
		return numOfDevices;
	}
	// if we have a device locked with a custom password
	if ( !pwd.empty() )
	{
		if (pwLength > 60)
		{
			return 0;
		}

		HalExecElement* el = new HalExecElement(ID_UnlockDeviceXv2);
		switch (this->mode) 
		{
			case ConfigManager::JTAG_MODE_4WIRE:
				el->appendInputData16(0);
				break;
			case ConfigManager::JTAG_MODE_SPYBIWIRE:
				el->appendInputData16(1);
				break;
			case ConfigManager::JTAG_MODE_4AFTER2:	
				el->appendInputData16(2);
				break;
			default:
				delete el;
				return 0;
		}

		el->appendInputData16(pwLength);	

		const char* pwdinternal = pwd.c_str();
		for (uint16_t i = 0; i < pwLength; i++)
		{
			const uint16_t hexWord = AsciToHex(pwdinternal);		
			el->appendInputData16(hexWord);
			pwdinternal += 4;      
		}
		
		el->setOutputSize(2);

		HalExecCommand cmd;
		cmd.elements.push_back(el);

		if (!this->parent->send(cmd))
		{
			return 0;
		}
		#ifndef NDEBUG
			printf("Unlock device\n");
		#endif

		numOfDevices = 1;
		return numOfDevices;
	}
	// if we have a "normal" msp430 device without special handling
 	if((deviceCode != 0xA55AA55A && deviceCode != 0x5AA55AA5) || deviceCode == 0x80058005)
	{
		HalExecCommand startJtag;
		HalExecElement* e2 = new HalExecElement(ID_StartJtag);
		switch (this->mode) {
		case ConfigManager::JTAG_MODE_4WIRE:
			e2->appendInputData8(0);
			break;
		case ConfigManager::JTAG_MODE_SPYBIWIRE:
			e2->appendInputData8(1);
			break;
		case ConfigManager::JTAG_MODE_4AFTER2:	
			e2->appendInputData8(2);
			break;
		case ConfigManager::JTAG_MODE_SPYBIWIRE_SUBMCU:	
			e2->appendInputData8(5);
			break;
		default:
			delete e2;
			return 0;
		}
		e2->setOutputSize(1);
		startJtag.elements.push_back(e2);
		if (!this->parent->send(startJtag))
		{
			return 0;
		}
		numOfDevices = startJtag.elements.at(0).getOutputAt8(0);
	#ifndef NDEBUG
		printf("num of devices %i\n",numOfDevices);
	#endif
		return numOfDevices;
	}
	return 0;
}

bool ConfigManagerV3::configureJtagSpeed(uint32_t speed)
{
	enum INTERFACE_SPEED iSpeed = (enum INTERFACE_SPEED)speed;	
	bool retValue = false;
	switch (iSpeed) 
	{
		case FAST:
			retValue = this->setJtagSpeed(JTAG_4WIRE_SPEED_8_MHZ,JTAG_2WIRE_SPEED_600_KHZ);
			break;
		case MEDIUM:
			retValue = this->setJtagSpeed(JTAG_4WIRE_SPEED_8_MHZ,JTAG_2WIRE_SPEED_400_KHZ);
			break;
		case SLOW:
			retValue = this->setJtagSpeed(JTAG_4WIRE_SPEED_8_MHZ,JTAG_2WIRE_SPEED_200_KHZ);
			break;
		default:
			retValue = this->setJtagSpeed(JTAG_4WIRE_SPEED_8_MHZ,JTAG_2WIRE_SPEED_600_KHZ);
			break;
	}
	return retValue;
}

bool ConfigManagerV3::setJtagSpeed(JTAG_4WIRE_SPEED speedJtag, JTAG_2WIRE_SPEED speedSbw)
{
	FetControl * control=this->parent->getControl();
	if(control->getFetToolId() == EZ_FET)
	{
		HalExecElement* el = new HalExecElement(ID_Configure);
		el->appendInputData32(CONFIG_PARAM_JTAG_SPEED);
		el->appendInputData32(speedJtag);
		el->appendInputData32(speedSbw);
		el->setOutputSize(0);
		HalExecCommand configCmd;
		configCmd.elements.push_back(el);
		return this->parent->send(configCmd);
	}
	else
	{
		return true;
	}
}

bool ConfigManagerV3::stop ()
{
	HalExecCommand stopJtag;
	stopJtag.elements.push_back(new HalExecElement(ID_StopJtag));
	return this->parent->send(stopJtag);
}

long ConfigManagerV3::MSP430I_MagicPattern()
{
	const ConfigManager::jtagMode oldJtagMode = mode;

	HalExecElement* el = new HalExecElement(ID_Reset430I);
	HalExecCommand cmd;
	cmd.elements.push_back(el);

	bool success = this->parent->send(cmd);
	if (!success)
	{
		setJtagMode(JTAG_MODE_SPYBIWIRE);
		start();

		el = new HalExecElement(ID_Reset430I);
		cmd.elements.clear();
		cmd.elements.push_back(el);
		success = this->parent->send(cmd);

		//If SBW failed as well, set original Jtag mode
		if (!success)
		{
			setJtagMode(oldJtagMode);
			start();
		}
	}

	if (success)
	{
		//GET Chain Length
		uint8_t chainLen= cmd.elements.at(0).getOutputAt8(0);
		//Get JTAG ID
		uint8_t iJtagID= cmd.elements.at(0).getOutputAt8(1);

		if ((chainLen > 0) && (iJtagID == 0x89))
		{
			// Return the protocol
			return cmd.elements.at(0).getOutputAt8(2);
		}
	}

	return -1;
}


bool ConfigManagerV3::reset(bool vcc, uint16_t pin, uint16_t JtagId)
{
	if (JtagId != 0x89 || deviceCode == 0x20404020)
	{
		if (vcc)
		{
			uint16_t voltage = this->getDeviceVcc();
			if (!this->setDeviceVcc(0))
			{
				return false;
			}
			// keep voltage 0 for minmum 5 seconds
			boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(5));

			if (!this->setDeviceVcc(voltage))
			{
				return false;
			}
			this->start();
		}

		if (vcc || pin != 0)
		{
			const hal_id resetMacro = (deviceCode == 0x20404020) ? ID_Reset430I : ID_ResetXv2;

			HalExecElement* el = new HalExecElement(resetMacro);

			HalExecCommand cmd;
			cmd.setTimeout(10000);
			cmd.elements.push_back(el);
			if (!this->parent->send(cmd))
			{
				return false;
			}
		}
	}
	else
	{
		/* for all other CPU architectures we just toggle the RST pint to create a BOR */
		if (pin != 0) 
		{
			list<PinState> pinStates;
			pinStates.push_back( PinState(JTAG_PIN_SELTST, false)(JTAG_PIN_RST, false).setDelay(pin) );
			pinStates.push_back( PinState(JTAG_PIN_SELTST, false)(JTAG_PIN_RST, true) );
			if (!sendPinSequence(pinStates, parent))
			{
				return false;
			}
		}
		if (vcc) 
		{
			uint16_t voltage = this->getDeviceVcc();
			if (!this->setDeviceVcc(0))
			{
				return false;
			}
			// keep voltate 0 for minmum 5 seconds 
			boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(5));
			if (!this->setDeviceVcc(voltage))
			{
				return false;
			}
		}
	}
	return true;
}

#define ENERGYTRACE_FINE_MODE 0x8000;
void ConfigManagerV3::setCurrentDrive(uint32_t value) 
{
	if(value)
	{
		mhighres = ENERGYTRACE_FINE_MODE;
	}
	else
	{
		mhighres = 0;
	}
}

void ConfigManagerV3::setEnergyTraceManager(EnergyTraceManager * etm)
{
	mEnergyTraceManager	= etm;		
}

bool ConfigManagerV3::setVccEzFet(uint16_t vcc)
{
	HalExecCommand cmd;
	#ifndef NDEBUG
		printf("VCC  in[mV]: %i\n",vcc);
	#endif
		HalExecElement* el;
		if(vcc)
		{
			uint16_t actualVcc = 0;
			//calculate VCC to configure sub mcu. 
			for(uint16_t i = 0 ; i < 4; i++)
			{
				actualVcc = actualVcc + this->getDeviceVcc();
			}
			actualVcc = actualVcc / 4;

			actualVcc = ((actualVcc + 50) / 100) * 100;

			if(actualVcc > 3600)
			{
				actualVcc = 3600;
			}

			if(actualVcc < 1800)
			{
				actualVcc = 1800;
			}

			//  send VCC set comand to SubMco
			el = new HalExecElement(ID_Zero, dcdcSetVcc);
			el->appendInputData16(actualVcc + mhighres); // mhighres Added for high resolution mode --> default 0;
			el->setInputMinSize(2);
			cmd.elements.push_back(el);
			if (!this->parent->send(cmd))
			{
				return false;
			}	
			// now run calibration to support Energy Trace
			if(mEnergyTraceManager)
			{
				if(!this->mEnergyTraceManager->doCalibration(actualVcc))
				{
					return false;
				}
			}
			// then send power on command to hil module to switch MOSFET
			el = new HalExecElement(ID_SetVcc);
			el->appendInputData16(actualVcc);
			el->setInputMinSize(2);
			cmd.elements.push_back(el);
			if (!this->parent->send(cmd))
			{
				return false;
			}
			this->vcc = actualVcc;

			if(vcc)
			{
				boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::milliseconds(500));
			}
			return true;	
		}
		else // just shutdown voltage
		{
			// then send power on command to hil module to switch MOSFET
			el = new HalExecElement(ID_SetVcc);
			el->appendInputData16(0);
			el->setInputMinSize(2);
			cmd.elements.push_back(el);
			if (!this->parent->send(cmd))
			{
				return false;
			}

			// Send power down comand to Sub mcu Firmware
			el = new HalExecElement(ID_Zero, dcdcPowerDown);
			el->setInputMinSize(2);
			cmd.elements.push_back(el);
			if (!this->parent->send(cmd))
			{
				return false;
			}	

			return true;
		}
		return false;
}


bool ConfigManagerV3::setVccMspFetUif(uint16_t vcc)
{
	HalExecCommand cmd;
	#ifndef NDEBUG
		printf("VCC  in[mV]: %i\n",vcc);
	#endif
		HalExecElement* el = new HalExecElement(ID_SetVcc);
		el->appendInputData16(vcc);
		el->setInputMinSize(2);
		cmd.elements.push_back(el);
		if (!this->parent->send(cmd))
		{
			return false;
		}
		this->vcc = vcc;
	#ifndef NDEBUG
		printf("VCC out[mV]: %i\n",this->getDeviceVcc ());
	#endif
		if(vcc)
		{
			boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::milliseconds(1000));
		}
	#ifndef NDEBUG
		printf("VCC out[mV]: %i\n",this->getDeviceVcc ());
	#endif
		return true;
}

bool ConfigManagerV3::setDeviceVcc (uint16_t vcc)
{
	FetControl * control=this->parent->getControl();
	if(control->getFetToolId()== EZ_FET)
	{
		return setVccEzFet(vcc);
	}
	else if(control->getFetToolId()== MSP_FET430)
	{
		return setVccMspFetUif(vcc);
	}
	else
	{
		return setVccMspFetUif(vcc);
	}
	return false;
}

uint16_t ConfigManagerV3::getDeviceVcc () const
{
	HalExecCommand cmd;
	HalExecElement* el = new HalExecElement(ID_GetVcc);
	el->setOutputSize(4);
	cmd.elements.push_back(el);
	if (!this->parent->send(cmd))
	{
		return 0;
	}
	return el->getOutputAt16(0);
}

uint16_t ConfigManagerV3::getExternalVcc () const
{
	HalExecCommand cmd;
	HalExecElement* el = new HalExecElement(ID_GetVcc);
	el->setOutputSize(4);
	cmd.elements.push_back(el);
	if (!this->parent->send(cmd))
		return 0;

	return el->getOutputAt16(2);
}


bool ConfigManagerV3::firmWareUpdate(const char* fname, UpdateNotifyCallback callback, bool* coreUpdate)
{	
	return this->updateManagerFet->firmWareUpdate(fname, callback, coreUpdate);
}

void ConfigManagerV3::setPassword(const string& pwd)
{
	this->password = pwd;
}

bool ConfigManagerV3::setDeviceCode(uint32_t deviceCode)
{
	 this->deviceCode = deviceCode;
	 return true;
}

bool ConfigManagerV3::freqCalibrationEnabled() const
{
	return freqCalibration;
}

bool ConfigManagerV3::ulpDebugEnabled() const
{
	return ulpDebug;
}

void ConfigManagerV3::setUlpDebug(bool ulp) 
{
	ulpDebug = ulp;
}

