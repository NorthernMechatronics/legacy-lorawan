/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, Northern Mechatronics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include <FreeRTOS.h>
#include <LmHandler.h>
#include <queue.h>

#define APPLICATION_CLOCK_SOURCE AM_HAL_CTIMER_LFRC_32HZ
#define APPLICATION_TIMER_PERIOD 32
#define APPLICATION_TIMER_SEGMENT AM_HAL_CTIMER_TIMERA
#define APPLICATION_TIMER_NUMBER 0
#define APPLICATION_TIMER_INT AM_HAL_CTIMER_INT_TIMERA0

#define APPLICATION_TRANSMIT_PERIOD 20

extern uint32_t gui32ApplicationTimerPeriod;

typedef enum
{
    JOIN = 0,
    RESET,
    SEND,
    SYNC_APP,
    SYNC_MAC,
    WAKE
} application_command_e;

extern TaskHandle_t application_task_handle;
extern QueueHandle_t ApplicationTaskQueue;
extern void application_task(void *pvParameters);

#define LM_APPLICATION_PORT 1
#define LM_MULTICAST_PORT   200
#define LM_FUOTA_PORT       201
#define LM_CLOCKSYNC_PORT   202
#define LM_COMPLIANCE_PORT  224

#define LM_BUFFER_SIZE 242

extern uint8_t psLmDataBuffer[LM_BUFFER_SIZE];
extern LmHandlerAppData_t LmAppData;
extern LmHandlerMsgTypes_t LmMsgType;

#endif /* _APPLICATION_H_ */
