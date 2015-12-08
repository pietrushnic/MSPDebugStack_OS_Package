/*
 * DeviceHandleV3.h
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

#pragma once

#include "DeviceHandle.h"
#include "MemoryManagerV3.h"
#include "DebugManagerV3.h"
#include "DeviceInfo.h"
#include "WatchdogControl.h"
#include "FuncletCode.h"

namespace TI
{
	namespace DLL430
	{
		class HalCommand;

		class DeviceHandleV3  : public DeviceHandle
		{
		public:
			DeviceHandleV3 (FetHandleV3*, uint32_t deviceCode);
			~DeviceHandleV3 ();

			DeviceHandleV3(const DeviceHandleV3&) = delete;
			DeviceHandleV3& operator=(const DeviceHandleV3&) = delete;

			EmulationManagerPtr getEmulationManager();
			MemoryManagerV3* getMemoryManager ();
			DebugManagerV3* getDebugManager ();
			FetHandleV3* getFetHandle () { return parent; }
			ClockCalibration* getClockCalibration() { return clockCalibration; }

			/// \brief Run the boot code writing the the specified command in the mailbox first
			/// \param command[in] Command to be put in the mailbox
			/// \return True if bootcode execution succeeded
			long magicPatternSend(uint16_t ifMode);
			int32_t identifyDevice(uint32_t activationKey, bool afterMagicPattern);
			const std::string & getDescription();
			bool secure ();
			bool reset(bool hardReset);

			FetControl* getControl ();

			bool send (HalExecCommand &command);

			void setWatchdogControl (std::shared_ptr<WatchdogControl>);
			std::shared_ptr<WatchdogControl> getWatchdogControl() const;
			uint32_t readJtagId();
			uint32_t getJtagId();
			uint32_t getDeviceIdPtr();
			uint32_t getEemVersion();
			bool isJtagFuseBlown();
			uint32_t getDeviceCode() const;

			void setDeviceId (long id);

			hal_id checkHalId(hal_id base_id) const;

			const FuncletCode& getFunclet(FuncletCode::Type funclet);

			bool supportsQuickMemRead() const;
			uint16_t getMinFlashVcc() const;
			bool hasFram() const;
			bool hasLPMx5() const;

			void disableHaltOnWakeup();

			bool eemAccessibleInLpm() const;

			bool deviceSupportsEnergyTrace() const;

		protected:

		private:
			FetHandleV3* parent;
			EmulationManagerPtr emulationManager;
			MemoryManagerV3* memoryManager;
			DebugManagerV3* debugManager;
			ClockCalibration* clockCalibration;

			uint16_t minFlashVcc;
			bool hasTestVpp;
			bool quickMemRead;
			bool deviceHasFram;
			bool deviceHasLPMx5;
			ClockSystem clockSystem;

			uint32_t jtagId;
			uint32_t deviceIdPtr;
			uint32_t eemVersion;
			enum DeviceHandle::jtagMode mode;
			uint32_t deviceCode;

			uint32_t powerTestRegDefault;
			uint16_t powerTestReg3VDefault;

			std::shared_ptr<WatchdogControl> wdt;
			DeviceInfo::function_map_type map;
			DeviceInfo::funclet_map_type funcletTable;

			std::string description;

			typedef std::array<uint8_t, DeviceInfo::nrUsedClockModules> EtwCodes;
			EtwCodes etwCodes;

			void configure (const DeviceInfo* info);
			long getDeviceIdentity(uint32_t activationKey, uint32_t* pc, uint32_t* sr, bool afterMagicPattern);

			bool sendDeviceConfiguration(uint32_t parameter, uint32_t value);

			int16_t getSubID(uint32_t info_len, uint32_t deviceIdPtr, uint32_t pc);
		};
	}
}
