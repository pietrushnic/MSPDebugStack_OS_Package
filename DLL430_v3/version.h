/*
 * Version.h
 *
 * Holds Version number.
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/ 
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

#ifndef DLL430_VERSION_H
#define DLL430_VERSION_H

#define VERSION_MAJOR 3
#define VERSION_MINOR 3
#define VERSION_PATCH 1
#define VERSION_BUILD 3
/* 
Version History
===============
Version:
    3.3.6.101
Date:
    07.09.2012
General:
    Updated DCDC Firmware
Embedded Changes implemented:
    = None
DLL Changes implemented:
    * UpdateManagerEzFet.cpp: Included new DCDC Firmware
TODO:
    * [Gerneral] Assess and tweak accuracy using Stand-alone mode
==================================================
Version:
    3.3.6.100
Date:
    07.09.2012
General:
    Updated calibration method with new resistor values
    Added Basic low-pass filtering of current samples
Embedded Changes implemented:
    * FetDCDC.c: 
        Changed calibration loop to just accumulate, not average samples
        Also do VCC sampling during calibration to compensate for possible offset
DLL Changes implemented:
    + EnergyTraceLowPassFilter: Added class to do filtering of data
    * EnergyTraceManager:Updated calibration function accordingly
TODO:
    * [Gerneral] Assess and tweak accuracy using Stand-alone mode
==================================================
Version:
    3.3.5.106
Date:
    28.08.2012
General:
    Fixed regression introduced after merge against ezFET
Embedded Changes implemented:
    None
DLL Changes implemented:
    * Reset HalCommand ID to 0 if it has been killed in order to be abel to identify if it is active or not
    * Changed calibration method to use combination of resistors as well so as to obtain more calibration points.
    * DisableEnergyTrace always returned false - corrected
==================================================
Version: 
    3.3.5.104
Date: 
    27.06.2012
General:
    Implemented calibration procedure where EnergyTrace is first calibrated against know reference currents
Embedded Changes implemented:
    Fixed regression where a subsequent debug session will not will
DLL Changes implemented:
    Implemented proper gating of EnergyTrace for different scenarios.
TODO:
    * [DLL] Multi-threaded reliability: add consumer thread to send data to IDE
==================================================
Version: 
    3.2.4.000
Date: 
    25.04.2012
Embedded Changes implemented: 
    * PollJStateReg.c: Resized non-interruptable part to minimum
DLL Changes implemented: 
    * EnergyTraceProcessor.cpp: Changed timestamp byte ordering to be correct (byte_0, not byte_7)
TODO:
    * [Embedded] Speed optimizations for IR/DR shifts
    * [General] Implement calibratrion state-machine
    * [DLL] Multi-threaded reliability: add consumer thread to send data to IDE
==================================================
Version: 
    3.2.3.004
Date: 
    22.03.2012
Embedded Changes implemented: 
    * PollJStateReg.c: added buffer to contain JSTATE, voltage and current measurements
                       send info to DLL if buffer is full.
    * hil_2w.c: updated sbw_shift function to be able to handle 64-bit shifts
    * hil.c: 
        + added EDT_Common method to get the latest time-tag
        + added EDT_Common method to measure target voltage
        + added setup for TIMERA0_2 for time-stamping
DLL Changes implemented: 
    + IProcessor.h: Added generic processor interface to process serial data
    + DoubleBuffer.h: Added generic template class to handle double buffering
    + EnergyTraceProcessor.h: 
        * Added processor class to process the records received from the firmware
        * Takes raw time-tag and ticks value and determines current, voltage and actual time values
    + Added correct FW binaries to DLL
TODO:
    * [Embedded] Speed optimizations for IR/DR shifts
    * [General] Implement calibratrion state-machine
    * [DLL] Multi-threaded reliability: add consumer thread to send data to IDE
==================================================
*/

#endif
