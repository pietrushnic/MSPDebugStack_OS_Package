/*
 * bios.h
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
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

//! \ingroup MODULBIOS
//! \file bios.h
//! \brief

#ifndef _V3_BIOS_H_
#define _V3_BIOS_H_

#include "hw_compiler_specific.h"
#include "../dcdc/FetDcdc.h"
#include "../Uart/FetUart.h"

//*********** Defines ***********
#define BIOS_LED_MODE       0
#define BIOS_LED_POWER      1

#define BIOS_RX_SIZE        258
#define BIOS_RX_QUEUS       4
#define BIOS_TX_SIZE        258
#define BIOS_TX_QUEUS       1
#define BIOS_TX_FLAGS       1

#define BIOS_RX_RDY         0x01
#define BIOS_RX_CRC_ERROR   0x02
#define BIOS_RX_SIZE_ERROR  0x04
#define BIOS_TX_TO_SEND     0x01
#define BIOS_TX_WAIT_ON_ACK 0x02
#define BIOS_TX_NO_SEND     0x04
#define BIOS_TX_BUSY        0x08

#define BIOS_TX_TIMEOUT     50

#define BIOS_TIMER_BREAK    0x01
#define BIOS_TIMER_COUNT    2
#define BIOS_TIMER_RX       0
#define BIOS_TIMER_TX       1

//*********** Structure definitions ***********

//! \brief Stop RX
//! \details set CTS to TUSB and break RX timeout timer
#define biosSetCts() {CTS_LINE = FALSE;  bios_global_timer_[0].state |= BIOS_TIMER_BREAK;}

//! \brief release (restart) RX
//! \details reset CTS to TUSB and RX timeout timer running
#define biosResetCts() { CTS_LINE = TRUE; bios_global_timer_[0].state &= ~BIOS_TIMER_BREAK;}

//! \struct Contains received messages
typedef struct _BiosRxRecord_ 
{
    unsigned char  active;
    unsigned char  last_cmd_typ;
    unsigned char  last_msg_id;
    unsigned short last_msg_call;
    unsigned char  state[BIOS_RX_QUEUS];
    unsigned char  *data[BIOS_RX_QUEUS];
    unsigned short *datas[BIOS_RX_QUEUS];
    unsigned short count[BIOS_RX_QUEUS];
    unsigned short size[BIOS_RX_QUEUS];
    unsigned short crc[2];
} BiosRxRecord;

extern volatile BiosRxRecord bios_rx_record_;

//! \struct Contains messages to be transmitted
typedef struct _BiosTxRecord_ 
{
    unsigned char  active;
    unsigned char  cannel_to_send;
    unsigned short send_counter[BIOS_TX_QUEUS];
    unsigned short ack_timeout[BIOS_TX_QUEUS];
    unsigned char  state[BIOS_TX_QUEUS];
    unsigned short count[BIOS_TX_QUEUS];
    unsigned short datas[BIOS_TX_QUEUS][BIOS_TX_SIZE/sizeof(unsigned short)];
    unsigned char  *data[BIOS_TX_QUEUS];
    unsigned short  *ext_data;
    unsigned short ext_size;
    unsigned short ext_counter;
} BiosTxRecord;

extern volatile BiosTxRecord bios_tx_record_;

//! \struct Global timer used for timeouts
struct _BiosGlobalTimer_
{
    unsigned short count;
    unsigned char state;
};
typedef struct _BiosGlobalTimer_ BiosGlobalTimer;

//*********** Function protoypes ***********

//! \brief System start up
void BIOS_InitSystem(void);

void BIOS_SuspendSystem(void);
void BIOS_ResumeSystem(void);

//! \brief setup receiver buffers
void BIOS_InitCom(void);

//! \brief install to HAL macros
//! \details copy the HAL irq to RAM irq table. Start the HAL startup code by using the reset vector.
//! \return -1 -> no HAL infos found, HAL not installed
short BIOS_HalInterfaceInit(void);

short BIOS_DcdcInterfaceInit(void);

void BIOS_UartInterfaceClear(void);
void BIOS_DcdcInterfaceClear(void);

short BIOS_UartInterfaceInit(void);
//! \brief main loop
//! \details returns never. 
//! \li send error messages
//! \li execute "commands in loop", eg. voltage regulation, wait(poll) for hit breakpoints
//! \li execute commands send from dll
void BIOS_MainLoop(void);

//! \brief reset the HAL interface
//! \details set irqs to dummy, but didn't disable interrupts.
//! Set all HAL call addresses to NULL
short BIOS_HalInterfaceClear(void);

//! \brief switch LED on
//! \details only the LED control struc are manipulated. The
//! hardware are in timerB1Isr driven
//! \param[in] LED no 0 = BIOS_LED_MODE (red)\n
//!                   1 = BIOS_LED_POWER (green)
//! \return always 0
short BIOS_LedOn(unsigned char no);

//! \brief switch LED off
//! \details only the LED control struc are manipulated. The
//! hardware are in timerB1Isr driven
//! \param[in] no 0 = BIOS_LED_MODE (red)\n
//!               1 = BIOS_LED_POWER (green)
//! \return always 0
short BIOS_LedOff(unsigned char no);

//! \brief LED blink continuous
//! \param[in] no 0 = BIOS_LED_MODE (red)\n
//!               1 = BIOS_LED_POWER (green)
//! \param[in] time (*20ms)
//! \return always 0
short BIOS_LedBlink(unsigned char no,unsigned short time);

//! \brief LED flashes for a time
//! \param[in] no 0 = BIOS_LED_MODE (red)\n
//!               1 = BIOS_LED_POWER (green)
//! \param[in] time (*20ms)
//! \return always 0
//! \details if the LED is on, the LED goes off for "time"
//! if the LED is off, the LED goes on for "time"
short BIOS_LedFlash(unsigned char no,unsigned short time);

//! \brief LEDs blink alternated
//! \param[in] time (*20ms)
//! \return always 0
//! \details both LED are alternated on.
short BIOS_LedAlternate(unsigned short time);

//! \brief fatal error
//! \details LED are alternated, function never return
void BIOS_GlobalError(void);

//! \brief clear RX hard and software buffer
void BIOS_UsbRxClear(void);

//! \brief trigger a error message
//! \details trigger a error message and clear the RX hard and software buffers
void BIOS_UsbRxError(unsigned short code);

//! \brief clears the TX software buffer
void BIOS_UsbTxError(void);

//! \brief finsh record to send
void BIOS_PrepareTx(unsigned int size);

//! \brief trigger sending of TX buffer
//! \details to send the first char we set UTXIFG, followed char loaded by 
//! true TX buffer empty interrupt
void BIOS_StartTx(void);

short IccMonitor_Process(unsigned short flags);
extern volatile BiosGlobalTimer bios_global_timer_[BIOS_TIMER_COUNT];
extern char bios_wb_control_;
extern const unsigned short core_version_;
extern unsigned short tool_id_;
extern __ro_placement volatile const unsigned short safe_core_version_;
extern unsigned long bios_device_flags_;
extern unsigned short bios_info_hw_0_;
extern unsigned short bios_info_hw_1_;
extern DCDC_INFOS_t dcdcInfos_;
extern UART_INFOS_t uartInfos_;
#endif
