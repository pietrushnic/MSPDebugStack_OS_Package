/*
 * HidUpdateManager.h
 *
 * Recovery for broken eZ-FETs and MSP-FET Debuggers 
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
#ifndef DLL430_HIDUPDATEMANAGER_H
#define DLL430_HIDUPDATEMANAGER_H

#include <stdint.h>
#include <string>

#include "UpdateManager.h"

#define MSPBSL_STANDARD_USB_VID 0X2047
#define MSPBSL_EZ_FET_USB_PID 0x0203

class MSPBSL_Connection5xxUSB;
class MSPBSL_Connection5xx;

namespace TI
{	
	namespace DLL430
	{		
		class FileFuncImpl;
		class HidUpdateManager 
		{
		public:
			static uint32_t countHidDevices(uint16_t productId);

			HidUpdateManager ();
			~HidUpdateManager ();
						
			bool hid_firmWareUpdate(const char* fname, UpdateNotifyCallback callback);
		
		private:
			MSPBSL_Connection5xxUSB* BslFet;
			bool hid_updateCore(const FileFuncImpl &firmware)const;
			uint16_t hid_readToolId();
			std::string hid_enumerateBSL();
			uint16_t hid_getBSLToolId();
		};

	};
};

#endif /* DLL430_HIDUPDATEMANAGER_H */
