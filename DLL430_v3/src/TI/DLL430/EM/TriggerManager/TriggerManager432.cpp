/*
 * TriggerManager432.cpp
 *
 * Handles trigger resources on 432
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

#include "../Trigger/Trigger432.h"

#include "TriggerManager432.h"

#include "../Exceptions/Exceptions.h"

using namespace TI::DLL430;

TriggerManager432::TriggerManager432(int numCodeTriggers, int numLiteralTriggers)
{
	uint32_t id = 0;

	for (int codeTrigger = 0; codeTrigger < numCodeTriggers; ++codeTrigger)
	{
		mCodeTriggers.push_back( Trigger432(Trigger432::CODE_TRIGGER, id++) );
	}

	for (int literalTrigger = 0; literalTrigger < numLiteralTriggers; ++literalTrigger)
	{
		mLiteralTriggers.push_back( Trigger432(Trigger432::LITERAL_TRIGGER, id++) );
	}
}


Trigger432* TriggerManager432::getCodeTrigger()
{
	for (Trigger432& trigger : mCodeTriggers)
	{
		if (!trigger.isInUse())
		{
			trigger.isInUse(true);
			return &trigger;
		}
	}
	return nullptr;
}


Trigger432* TriggerManager432::getLiteralTrigger()
{
	for (Trigger432& trigger : mLiteralTriggers)
	{
		if (!trigger.isInUse())
		{
			trigger.isInUse(true);
			return &trigger;
		}
	}
	return nullptr;
}


void TriggerManager432::releaseTrigger(Trigger432* trigger)
{
	trigger->reset();
}

int TriggerManager432::numAvailableCodeTriggers() const
{
	int count = 0;

	for (const Trigger432& trigger : mCodeTriggers)
	{
		if (!trigger.isInUse())
		{
			++count;
		}
	}
	return count;
}


int TriggerManager432::numAvailableLiteralTriggers() const
{
	int count = 0;

	for (const Trigger432& trigger : mLiteralTriggers)
	{
		if (!trigger.isInUse())
		{
			++count;
		}
	}
	return count;
}

void TriggerManager432::writeAllTriggers() const
{
	for (const Trigger432& trigger : mCodeTriggers)
	{
		trigger.write();
	}
	for (const Trigger432& trigger : mLiteralTriggers)
	{
		trigger.write();
	}
}
