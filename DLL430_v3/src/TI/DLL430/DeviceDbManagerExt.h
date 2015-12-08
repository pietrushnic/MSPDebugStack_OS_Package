/*
 * DebugManager.h
 *
 * Implementation for accessing TemplateDeviceDb.
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

#include "DeviceDbManager.h"
#include "DeviceInfo.h"


namespace TI
{
	namespace DLL430
	{
		//identifier to find devices in DB
		struct MSP430_device_idCode {
			uint16_t verId;
			uint16_t verSubId;
			uint8_t  revisison;
			uint8_t  fab;
			uint16_t self;
			char     config;
			uint8_t  fuses;
			uint32_t activationKey;
			uint8_t revisisonMax;
		};

		class TemplateDeviceDbManagerExt : public DeviceDbManager
		{
		public:
			/** \brief  Creates DeviceInfo and returns pointer to it.
			 *			The parameter wouldn't be necessary, but defined by interface.
			 *
			 * \param	position of the device in the internal device table.
			 *
			 * \return pointer to DeviceInfo as SmartPointer
			 */
			virtual DeviceInfoPtr queryDb (size_t) const;

			/** \brief  Prepares templateDeviceDb to point to correct device
			 *			See description of queryDb (size_t id)  above
			 *
			 * \param idCode to find device with as MSP430_device_idCode
			 * \return dummy value of '1' which is of no meaning.
			 */
			virtual size_t queryDb ( const struct MSP430_device_idCode& idCode ) const;

			/** \brief Get highest valid database entry
			 *
			 * \return max allowed id
			 */
			virtual size_t getMaxId () const;

			/** \brief check if the device has TestVPP
			 */
			virtual void dumpDatabase() const;
		};
	} //namespace DLL430
} //namespace TI
