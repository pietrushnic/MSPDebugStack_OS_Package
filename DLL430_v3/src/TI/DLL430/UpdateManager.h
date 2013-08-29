/*
 * UpdateManager.h
 *
 * Provides routines for update handling for various debuggers
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

#if _MSC_VER > 1000
#pragma once
#endif
#ifndef DLL430_UPDATEMANAGER_H
#define DLL430_UPDATEMANAGER_H

#include <stdint.h>
#include <boost/function.hpp>

#include "VersionInfo.h"

namespace TI
{
	namespace DLL430
	{
		enum UPDATE_STATUS_MESSAGES {
			BL_INIT = 0, /**< Initializing Update Bootloader */
			BL_ERASE_INT_VECTORS = 1, /**< Erasing mapped interrupt vectors */
			BL_ERASE_FIRMWARE = 2, /**< Erasing firmware memory section */
			BL_PROGRAM_FIRMWARE = 3, /**< Program new firmware */
			BL_DATA_BLOCK_PROGRAMMED = 4, /**< One data block of the new firmware was successfully programmed */
			BL_EXIT = 5, /**< Exit Update Bootlader and reboot firmware */
			BL_UPDATE_DONE = 6, /**< Update was successfully finished */
			BL_UPDATE_ERROR = 7, /**< An error occured during firmware update */
			BL_WAIT_FOR_TIMEOUT = 8 /**< An error occured during firmware update */
		};

		class DeviceHandle;
		class FetHandleManager;

		typedef boost::function3<void, uint32_t, uint32_t, uint32_t> UpdateNotifyCallback;

		/** \brief manage the target device and the connection between FET and target device */
		class UpdateManager
		{
		public:
			/** \brief get the version of the FET Hal
			 *
			 * \return the version information
			 */
			virtual VersionInfo getHalVersion () const = 0;

			/** \brief chick if firmware update is required 
			 *
			 * \return true if firmware update is required 
			 */
			virtual bool isUpdateRequired () const = 0;

			/** \brief perform firmwareupdate
			 *
			 * \param fname defines the TI-txt file to be used for update or NULL for internal image
			 * \param callback defines the callback for update messages or NULL for no messages
			 * \param clientHandle reference given by the caller instance, returned in callback
			 * \return true on success
			 */	
			virtual bool firmWareUpdate(const char* fname, UpdateNotifyCallback callback = 0, bool* coreUpdate = 0) = 0;
		};
	};
};

#endif /* DLL430_UPDATEMANAGER_H */
