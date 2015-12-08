/*
 * TriggerConditionManager432.cpp
 *
 * Implementation of trigger condition manager for 432
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

#include "TriggerConditionManager432.h"
#include "AddressCondition432.h"
#include "../TriggerManager/TriggerManager432.h"
#include "../Exceptions/Exceptions.h"

using namespace TI::DLL430;


TriggerConditionManager432::TriggerConditionManager432(TriggerManager432Ptr triggerManager)
	: triggerManager_(triggerManager)
{
}


SoftwareTriggerConditionPtr TriggerConditionManager432::createSoftwareTriggerCondition(uint32_t address)
{
	throw EM_SoftwareBreakpointsNotEnabledException();
}


RegisterConditionPtr TriggerConditionManager432::createRegisterCondition(uint8_t reg, uint32_t value, uint32_t mask,
																		 ComparisonOperation op)
{
	throw EM_TriggerResourceException();
}


InstructionAddressConditionPtr TriggerConditionManager432::createInstructionAddressCondition(uint32_t address, uint32_t,
																							 AccessType,
																							 ComparisonOperation)
{
	if (triggerManager_->numAvailableCodeTriggers() < 1)
		throw EM_TriggerResourceException();

	return std::make_shared<AddressCondition432>(triggerManager_, address);
}


DataAddressConditionPtr TriggerConditionManager432::createDataAddressCondition(uint32_t address, uint32_t,
																			   AccessType, ComparisonOperation)
{
	if (triggerManager_->numAvailableCodeTriggers() < 1)
		throw EM_TriggerResourceException();

	return std::make_shared<AddressCondition432>(triggerManager_, address);
}


DataValueConditionPtr TriggerConditionManager432::createDataValueCondition(uint32_t value, uint32_t mask,
																		   AccessType accessType,  ComparisonOperation op)
{
	throw EM_TriggerResourceException();
}


InstructionRangeConditionPtr TriggerConditionManager432::createInstructionRangeCondition(uint32_t minAddress, uint32_t maxAddress,
																						 uint32_t minMask, uint32_t maxMask,
																						 AccessType accessType, bool outside)
{
	throw EM_TriggerResourceException();
}


AddressRangeConditionPtr TriggerConditionManager432::createAddressRangeCondition(uint32_t minAddress, uint32_t maxAddress,
																				 uint32_t minMask, uint32_t maxMask,
																				 AccessType accessType, bool outside)
{
	throw EM_TriggerResourceException();
}


DataRangeConditionPtr TriggerConditionManager432::createDataRangeCondition(uint32_t minValue, uint32_t maxValue,
																		   uint32_t minMask, uint32_t maxMask,
																		   AccessType accessType, bool outside)
{
	throw EM_TriggerResourceException();
}
