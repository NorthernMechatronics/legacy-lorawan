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
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <am_bsp.h>
#include <am_mcu_apollo.h>
#include <am_util.h>

#include <LmHandler.h>
#include <LmHandlerMsgDisplay.h>
#include <LmhpCompliance.h>
#include <NvmCtxMgmt.h>
#include <board.h>
#include <timer.h>

#include "application.h"
#include "application_cli.h"
#include "console_task.h"
#include "task_message.h"


uint32_t gui32ApplicationTimerPeriod;
static uint32_t gui32Counter;

TaskHandle_t application_task_handle;
QueueHandle_t ApplicationTaskQueue;

uint8_t psLmDataBuffer[LM_BUFFER_SIZE];
LmHandlerAppData_t LmAppData;

static LmHandlerParams_t LmParameters;
static LmHandlerCallbacks_t LmCallbacks;

static bool TransmitPending;
static bool MacProcessing;

static void OnClassChange(DeviceClass_t deviceClass)
{
    DisplayClassUpdate(deviceClass);
}

static void OnMacProcess(void)
{
    // this is called inside an IRQ
    MacProcessing = true;
}

static void OnJoinRequest(LmHandlerJoinParams_t *params)
{
    am_util_stdio_printf("\r\n");
    DisplayJoinRequestUpdate(params);

    if (params->Status == LORAMAC_HANDLER_ERROR) {
        LmHandlerJoin();
    } else {
        am_util_stdio_printf("LoRaWAN join successful\r\n\r\n");
    }

    nm_console_print_prompt();
}

static void OnMacMlmeRequest(LoRaMacStatus_t status, MlmeReq_t *mlmeReq,
                             TimerTime_t nextTxDelay)
{
    am_util_stdio_printf("\r\n");
    DisplayMacMlmeRequestUpdate(status, mlmeReq, nextTxDelay);
    nm_console_print_prompt();
}

static void OnMacMcpsRequest(LoRaMacStatus_t status, McpsReq_t *mcpsReq,
                             TimerTime_t nextTxDelay)
{
    am_util_stdio_printf("\r\n");
    DisplayMacMcpsRequestUpdate(status, mcpsReq, nextTxDelay);
    nm_console_print_prompt();
}

static void OnNetworkParametersChange(CommissioningParams_t *params)
{
    am_util_stdio_printf("\r\n");
    DisplayNetworkParametersUpdate(params);
    nm_console_print_prompt();
}

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
    am_util_stdio_printf("\r\n");
    DisplayRxUpdate(appData, params);
    nm_console_print_prompt();

    switch (appData->Port) {
    case 3:
        if (appData->BufferSize == 1) {
            switch (appData->Buffer[0]) {
            case 0:
                LmHandlerRequestClass(CLASS_A);
                break;
            case 1:
                LmHandlerRequestClass(CLASS_B);
                break;
            case 2:
                LmHandlerRequestClass(CLASS_C);
                break;
            default:
                break;
            }
        }
        break;

    case LM_APPLICATION_PORT:
        break;
    }
}

static void OnTxData(LmHandlerTxParams_t *params)
{
    am_util_stdio_printf("\r\n");
    DisplayTxUpdate(params);
    nm_console_print_prompt();
}

void application_handle_uplink()
{
    if (TransmitPending) {
        if (LmHandlerIsBusy() == true) {
            return;
        }
        TransmitPending = false;
        LmHandlerSend(&LmAppData, LORAMAC_HANDLER_UNCONFIRMED_MSG);
    }
}

void application_handle_command()
{
    task_message_t TaskMessage;

    // do not block on message receive as the LoRa MAC state machine decides
    // when it is appropriate to sleep.  We also do not explicitly go to
    // sleep directly and simply do a task yield.  This allows other timing
    // critical radios such as BLE to run their state machines.
    if (xQueueReceive(ApplicationTaskQueue, &TaskMessage, 0) == pdPASS) {
        switch (TaskMessage.ui32Event) {
        case JOIN:
            LmHandlerJoin();
            break;
        case SEND:
            TransmitPending = true;
            break;
        }
    }
}

void application_timer_isr()
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	task_message_t TaskMessage;

	am_hal_ctimer_int_clear(APPLICATION_TIMER_INT);


	sprintf(psLmDataBuffer, "%d", gui32Counter);

    LmAppData.Port = LM_APPLICATION_PORT;
    LmAppData.BufferSize = strlen(psLmDataBuffer);
    LmAppData.Buffer = psLmDataBuffer;
    gui32Counter++;

	TaskMessage.ui32Event = SEND;
    TaskMessage.psContent = &LmAppData;

	xQueueSendFromISR(ApplicationTaskQueue, &TaskMessage, &xHigherPriorityTaskWoken);

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void application_timer_setup()
{
	am_hal_ctimer_config_t ApplicationTimer =
	{
		0,
		(AM_HAL_CTIMER_FN_REPEAT |
		AM_HAL_CTIMER_INT_ENABLE |
		APPLICATION_CLOCK_SOURCE),
		0,
	};

	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_LFRC_START, 0);
	am_hal_ctimer_clear(0, APPLICATION_TIMER_SOURCE);
	am_hal_ctimer_config(0, &ApplicationTimer);

	am_hal_ctimer_int_register(APPLICATION_TIMER_INT, application_timer_isr);
	am_hal_ctimer_int_clear(APPLICATION_TIMER_INT);
	NVIC_SetPriority(CTIMER_IRQn, NVIC_configKERNEL_INTERRUPT_PRIORITY);
	am_hal_ctimer_int_enable(APPLICATION_TIMER_INT);
	NVIC_EnableIRQ(CTIMER_IRQn);
}

void application_setup()
{
    BoardInitMcu();
    BoardInitPeriph();

    LmParameters.Region = LORAMAC_REGION_US915;
    LmParameters.AdrEnable = true;
    LmParameters.TxDatarate = DR_0;
    LmParameters.PublicNetworkEnable = true;
    LmParameters.DataBufferMaxSize = LM_BUFFER_SIZE;
    LmParameters.DataBuffer = psLmDataBuffer;

    switch (LmParameters.Region) {
    case LORAMAC_REGION_EU868:
    case LORAMAC_REGION_RU864:
    case LORAMAC_REGION_CN779:
        LmParameters.DutyCycleEnabled = true;
        break;
    default:
        LmParameters.DutyCycleEnabled = false;
        break;
    }

    memset(&LmCallbacks, 0, sizeof(LmHandlerCallbacks_t));
    // these are mandatory
    LmCallbacks.OnMacProcess = OnMacProcess;
    LmCallbacks.OnJoinRequest = OnJoinRequest;
    LmCallbacks.OnNetworkParametersChange = OnNetworkParametersChange;
    LmCallbacks.OnMacMlmeRequest = OnMacMlmeRequest;
    LmCallbacks.OnMacMcpsRequest = OnMacMcpsRequest;
    LmCallbacks.OnTxData = OnTxData;
    LmCallbacks.OnRxData = OnRxData;

    // these are optional
    LmCallbacks.OnClassChange = OnClassChange;

    LmHandlerInit(&LmCallbacks, &LmParameters);
    LmHandlerPackageRegister(PACKAGE_ID_COMPLIANCE,
                             LmphCompliancePackageFactory());
}

void application_task(void *pvParameters)
{
    FreeRTOS_CLIRegisterCommand(&ApplicationCommandDefinition);
    ApplicationTaskQueue = xQueueCreate(10, sizeof(task_message_t));

    am_util_stdio_printf_init(nm_console_print);
    am_util_stdio_printf("\r\n\r\nLoRaWAN Application Demo\r\n\r\n");
    nm_console_print_prompt();

    application_setup();
    application_timer_setup();

    TransmitPending = false;
    while (1) {
        application_handle_command();
        application_handle_uplink();
        LmHandlerProcess();

        if (MacProcessing) {
            taskENTER_CRITICAL();
            MacProcessing = 0;
            taskEXIT_CRITICAL();
        } else {
            taskYIELD();
        }
    }
}

