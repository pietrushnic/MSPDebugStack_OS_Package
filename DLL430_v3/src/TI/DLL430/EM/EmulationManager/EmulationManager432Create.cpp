/*
 * EmulationManager430Create.cpp
 *
 * Creators for different EEM modules for MSP430
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

#include "EmulationManager432.h"

#include "../TriggerManager/TriggerManager432.h"
#include "../TriggerCondition/TriggerConditionManager432.h"
#include "../BreakpointManager/BreakpointManager432.h"

using namespace TI::DLL430;

namespace {

	class Em : public EmulationManager432
	{
	public:
		static std::shared_ptr<Em> create()
		{
			std::shared_ptr<Em> em = std::make_shared<Em>();

			em->mTriggerManager = std::make_shared<TriggerManager432>(6, 0);
			em->mBreakpointManager = std::make_shared<BreakpointManager432>();
			em->mTriggerConditionManager = std::make_shared<TriggerConditionManager432>(em->mTriggerManager);
			return em;
		}

		virtual void reset() { *this = *create(); }
	};
}


EmulationManagerPtr EmulationManager432::create()
{
	return Em::create();
}
