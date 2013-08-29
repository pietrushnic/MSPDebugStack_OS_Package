/*
 * UsbIoChannel.h
 *
 * Base class for USB IOChannel devices, e.g. CDC or HID.
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
#ifndef DLL430_USBIOCHANNEL_H
#define DLL430_USBIOCHANNEL_H

#include "IoChannel.h"
#include <boost/ptr_container/ptr_map.hpp>
#include <list>

struct halObjectDb_object;

namespace TI
{
	namespace DLL430
	{
		class PortInfoListBuilder;

		class UsbIoChannel : public IoChannel
		{
		public:
			enum settings {
				Setting_FlowControlMode = 0,
				Setting_Baudrate        = 1,
				Setting_DmaTimeout      = 2,
				Setting_DTR             = 3,
				Setting_RTS             = 4,
				Setting_DSR             = 5,
				Setting_DataFormat      = 6,
				Setting_FwUpdateMode    = 10,
				Setting_FlushBuffer     = 11,
			};

			enum flowControl {
				FlowControl_None     = 0x00,
				FlowControl_XonXoff  = 0x01,
				FlowControl_Hardware = 0x24,
			};
			
			UsbIoChannel(const PortInfo&);
			virtual ~UsbIoChannel();

			std::vector<uint8_t> * getHwVersion();
			std::vector<uint8_t> * getSwVersion();

			bool setObjectDbEntry (size_t entry);
			size_t getFunctionAddress (unsigned long id);

		protected:
			struct FetFunction {
				halObjectDb_object* object; //can be 0
				uint16_t functionId;
				uint16_t callAddress;
				bool sticky;
			};
			typedef boost::ptr_map<unsigned long, UsbIoChannel::FetFunction> object_map_type;
			typedef std::list<UsbIoChannel::FetFunction*> loaded_list_type;
			
			uint16_t createCrc(const uint8_t * buf);

			PortInfo portInfo;
			
			object_map_type  available;
			loaded_list_type loaded;
			loaded_list_type requested;

			size_t totalSize;
			size_t availableSize;
			size_t stickySize;
			size_t requestedSize;
			
		private:
			UsbIoChannel(const UsbIoChannel&);
			void operator=(const UsbIoChannel&);
		};

	};
};

#endif /* DLL430_USBIOCHANNEL_H */
