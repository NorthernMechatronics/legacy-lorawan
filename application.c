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

#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>
#include <queue.h>

#include <LmHandler.h>
#include <LmHandlerMsgDisplay.h>
#include <LmhpCompliance.h>
#include <NvmCtxMgmt.h>
#include <board.h>
#include <timer.h>

#include "application.h"
#include "console_task.h"
#include "task_message.h"


#define APPLICATION_CLOCK_SOURCE    AM_HAL_CTIMER_LFRC_32HZ
#define APPLICATION_TIMER_PERIOD    32
#define APPLICATION_TIMER_SOURCE	AM_HAL_CTIMER_TIMERA
#define APPLICATION_TIMER_INT		AM_HAL_CTIMER_INT_TIMERA0

static uint32_t gui32ApplicationTimerPeriod = 10;
static uint32_t gui32Counter = 0;

typedef enum { JOIN = 0, SEND } application_command_e;

TaskHandle_t application_task_handle;
portBASE_TYPE prvApplicationCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                    const char *pcCommandString);

const CLI_Command_Definition_t prvApplicationCommandDefinition = {
    (const char *const) "lorawan",
    (const char *const) "lorawan:\tLoRaWAN Application Framework.\r\n",
    prvApplicationCommand, -1};

static QueueHandle_t ApplicationTaskQueue;


#define LM_APPLICATION_PORT 1
#define LM_BUFFER_SIZE 242
static uint8_t psLmDataBuffer[LM_BUFFER_SIZE];
static LmHandlerAppData_t LmAppData;

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
    FreeRTOS_CLIRegisterCommand(&prvApplicationCommandDefinition);
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

void prvApplicationHelpSubCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                  const char *pcCommandString)
{
    const char *pcParameterString;
    portBASE_TYPE xParameterStringLength;

    pcParameterString =
        FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength);

    if (pcParameterString == NULL) {
        strcat(pcWriteBuffer, "usage: lorawan [command] [<args>]\r\n");
        strcat(pcWriteBuffer, "\r\n");
        strcat(pcWriteBuffer, "Supported commands are:\r\n");
        strcat(pcWriteBuffer, "  join\r\n");
        strcat(pcWriteBuffer, "  send\r\n");
        strcat(pcWriteBuffer, "  periodic\r\n");
        strcat(pcWriteBuffer, "\r\n");
        strcat(
            pcWriteBuffer,
            "See 'lorawan help [command] for the details of each command.\r\n");
    } else if (strncmp(pcParameterString, "join", 4) == 0) {
        strcat(pcWriteBuffer, "usage: lorawan join\r\n");
        strcat(pcWriteBuffer, "Join a LoRaWAN network.\r\n");
    } else if (strncmp(pcParameterString, "send", 3) == 0) {
        strcat(pcWriteBuffer, "usage: lorawan send <port> <ack> [msg]\r\n");
        strcat(pcWriteBuffer, "\r\n");
        strcat(pcWriteBuffer, "Where:\r\n");
        strcat(pcWriteBuffer, "  port  is the uplink port number\r\n");
        strcat(pcWriteBuffer,
               "  ack   request message confirmation from the server\r\n");
        strcat(pcWriteBuffer, "  msg   payload content\r\n");
	} else if (strncmp(pcParameterString, "periodic", 8) == 0) {
		strcat(pcWriteBuffer, "usage: lorawan periodic [start <period>|stop]\r\n");
		strcat(pcWriteBuffer, "\r\n");
		strcat(pcWriteBuffer, "Where:\r\n");
		strcat(pcWriteBuffer, "  start   to begin transmitting periodically\r\n");
		strcat(pcWriteBuffer, "  stop    to stop transmitting\r\n");
		strcat(pcWriteBuffer, "  period  defines how often to transmit in seconds (default is 10s)\r\n");
	}
}

void prvApplicationSendSubCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                  const char *pcCommandString)
{
    const char *pcParameterString;
    portBASE_TYPE xParameterStringLength;

    uint8_t port = LM_APPLICATION_PORT;
    uint8_t argc = FreeRTOS_CLIGetNumberOfParameters(pcCommandString);

    pcParameterString =
        FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength);
    if (argc == 2) {
        memcpy(psLmDataBuffer, pcParameterString, xParameterStringLength);

        LmAppData.Port = port;
        LmAppData.BufferSize = xParameterStringLength;
        LmAppData.Buffer = psLmDataBuffer;
    } else {
        pcParameterString = FreeRTOS_CLIGetParameter(pcCommandString, 2,
                                                     &xParameterStringLength);
        if (pcParameterString == NULL) {
            strcat(pcWriteBuffer, "error: missing port number\r\n");
            return;
        } else {
            port = atoi(pcParameterString);
        }

        pcParameterString = FreeRTOS_CLIGetParameter(pcCommandString, 3,
                                                     &xParameterStringLength);
        memcpy(psLmDataBuffer, pcParameterString, xParameterStringLength);

        LmAppData.Port = port;
        LmAppData.BufferSize = xParameterStringLength;
        LmAppData.Buffer = psLmDataBuffer;
    }

    task_message_t TaskMessage;
    TaskMessage.ui32Event = SEND;
    TaskMessage.psContent = &LmAppData;
    xQueueSend(ApplicationTaskQueue, &TaskMessage, portMAX_DELAY);
}

void prvApplicationPeriodicSubCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                  const char *pcCommandString)
{
    const char *pcParameterString;
    portBASE_TYPE xParameterStringLength;

    pcParameterString =
        FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength);
    if (pcParameterString == NULL) {
        return;
    }

    if (strncmp(pcParameterString, "start", xParameterStringLength) == 0) {

        pcParameterString =
            FreeRTOS_CLIGetParameter(pcCommandString, 3, &xParameterStringLength);
        if (pcParameterString == NULL) {
        	gui32ApplicationTimerPeriod = 5;
        }
        else
        {
        	gui32ApplicationTimerPeriod = atoi(pcParameterString);
        }

    	uint32_t ui32Period = gui32ApplicationTimerPeriod * APPLICATION_TIMER_PERIOD;
		am_hal_ctimer_period_set(0, APPLICATION_TIMER_SOURCE, ui32Period, (ui32Period >> 1));
		am_hal_ctimer_start(0, APPLICATION_TIMER_SOURCE);
    }
    else if (strncmp(pcParameterString, "stop", xParameterStringLength) == 0) {
    	am_hal_ctimer_stop(0, APPLICATION_TIMER_SOURCE);
    }
}

portBASE_TYPE prvApplicationCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                    const char *pcCommandString)
{
    const char *pcParameterString;
    portBASE_TYPE xParameterStringLength;

    pcWriteBuffer[0] = 0x0;

    pcParameterString =
        FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (pcParameterString == NULL) {
        return pdFALSE;
    }

    if (strncmp(pcParameterString, "help", xParameterStringLength) == 0) {
        prvApplicationHelpSubCommand(pcWriteBuffer, xWriteBufferLen,
                                     pcCommandString);
    } else if (strncmp(pcParameterString, "join", xParameterStringLength) ==
               0) {
        task_message_t TaskMessage;
        TaskMessage.ui32Event = JOIN;
        xQueueSend(ApplicationTaskQueue, &TaskMessage, portMAX_DELAY);
    } else if (strncmp(pcParameterString, "send", xParameterStringLength) ==
               0) {
        prvApplicationSendSubCommand(pcWriteBuffer, xWriteBufferLen,
                                     pcCommandString);
    } else if (strncmp(pcParameterString, "periodic", xParameterStringLength) == 0) {
    	prvApplicationPeriodicSubCommand(pcWriteBuffer, xWriteBufferLen,
                pcCommandString);
    }

    return pdFALSE;
}
