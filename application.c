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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <am_bsp.h>
#include <am_mcu_apollo.h>
#include <am_util.h>

#include <FreeRTOS.h>
#include <queue.h>

#include <LmHandler.h>
#include <LmHandlerMsgDisplay.h>
#include <LmhpClockSync.h>
#include <LmhpCompliance.h>
#include <LmhpRemoteMcastSetup.h>
#include <NvmDataMgmt.h>
#include <board.h>
#include <timer.h>

#include "application.h"
#include "application_cli.h"
#include "console_task.h"
#include "task_message.h"

#define LORAWAN_DEFAULT_CLASS CLASS_A

uint32_t gui32ApplicationTimerPeriod;
static uint32_t gui32Counter;

TaskHandle_t application_task_handle;
QueueHandle_t ApplicationTaskQueue;

uint8_t psLmDataBuffer[LM_BUFFER_SIZE];
LmHandlerAppData_t LmAppData;
LmHandlerMsgTypes_t LmMsgType;

static LmHandlerParams_t LmParameters;
static LmHandlerCallbacks_t LmCallbacks;

static LmhpComplianceParams_t LmComplianceParams;

static volatile bool TransmitPending = false;
static volatile bool MacProcessing = false;
static volatile bool ClockSynchronized = false;
static volatile bool McSessionStarted = false;

static uint32_t timeout = portMAX_DELAY;
/*
 * Board ID is called by the LoRaWAN stack to
 * uniquely identify this device.
 * 
 * This example uses 4 bytes from the processor ID
 * and another 4 bytes that are user-defined. 
 */
void BoardGetUniqueId(uint8_t *id)
{
    am_util_id_t i;

    am_util_id_device(&i);

    id[0] = 0x01;
    id[1] = 0x02;
    id[2] = 0x03;
    id[3] = 0x04;
    id[4] = (uint8_t)(i.sMcuCtrlDevice.ui32ChipID0);
    id[5] = (uint8_t)(i.sMcuCtrlDevice.ui32ChipID0 >> 8);
    id[6] = (uint8_t)(i.sMcuCtrlDevice.ui32ChipID0 >> 16);
    id[7] = (uint8_t)(i.sMcuCtrlDevice.ui32ChipID0 >> 24);
}

static void TclProcessCommand(LmHandlerAppData_t *appData)
{
    // Only handle the reset command as that is indicative of the
    // beginning of compliance testing.  All the other compliance test
    // commands are handle by the Compliance state machine
    switch (appData->Buffer[0]) {
    case 0x01:
        am_util_stdio_printf("Tcl: LoRaWAN MAC layer reset requested\r\n");
        break;
    case 0x05:
        am_util_stdio_printf("Tcl: Duty cycle set to %d\r\n",
                             appData->Buffer[1]);
        break;
    }
}

static void application_rtc_set(uint8_t* time_correction)
{
    time_t epoch_time;

    epoch_time =
                time_correction[0]
            | (time_correction[1] << 8)
            | (time_correction[2] << 16)
            | (time_correction[3] << 24);

    struct tm ts;

    if (localtime_r(&epoch_time, &ts))
    {
        am_hal_rtc_time_t hal_rtc_time;
        hal_rtc_time.ui32Hour       = ts.tm_hour; // 0 to 23
        hal_rtc_time.ui32Minute     = ts.tm_min; // 0 to 59
        hal_rtc_time.ui32Second     = ts.tm_sec; // 0 to 59

        hal_rtc_time.ui32DayOfMonth = ts.tm_mday; // 1 to 31
        hal_rtc_time.ui32Month      = ts.tm_mon; // 0 to 11
        hal_rtc_time.ui32Year       = ts.tm_year + 1900 - 2000; // years since 2000
        hal_rtc_time.ui32Century    = 0;

        am_hal_rtc_time_set(&hal_rtc_time);
    }
}

/*
 * LoRaMAC Application Layer Callbacks
 */
static void OnClassChange(DeviceClass_t deviceClass)
{
    DisplayClassUpdate(deviceClass);
    switch (deviceClass) {
    default:
    case CLASS_A: {
        McSessionStarted = false;
    } break;
    case CLASS_B: {
        LmHandlerAppData_t appData = {
            .Buffer = NULL,
            .BufferSize = 0,
            .Port = 0,
        };
        LmHandlerSend(&appData, LORAMAC_HANDLER_UNCONFIRMED_MSG);
        McSessionStarted = true;
    } break;
    case CLASS_C: {
        McSessionStarted = true;
    } break;
    }
}

static void OnMacProcess(void)
{
    // this is called inside an IRQ
    MacProcessing = true;
    timeout = 0;
    am_hal_gpio_state_write(AM_BSP_GPIO_LED4, AM_HAL_GPIO_OUTPUT_SET);
}

static void OnJoinRequest(LmHandlerJoinParams_t *params)
{
    am_util_stdio_printf("\r\n");
    DisplayJoinRequestUpdate(params);

    if (params->Status == LORAMAC_HANDLER_ERROR) {
        LmHandlerJoin();
    } else {
        //LmHandlerRequestClass(LORAWAN_DEFAULT_CLASS);
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

static void OnNvmDataChange(LmHandlerNvmContextStates_t state, uint16_t size)
{
    am_util_stdio_printf("\r\n");
    DisplayNvmDataChange(state, size);
    nm_console_print_prompt();
}

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
    am_util_stdio_printf("\r\n");
    DisplayRxUpdate(appData, params);

    switch (appData->Port) {
    case 0:
        am_util_stdio_printf("MAC command received\r\n");
        break;

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
        // process application specific data here
        break;


    case 202:
        // process MAC layer time synchronization
        application_rtc_set(&(appData->Buffer[1]));
        break;

    case 224:
        TclProcessCommand(appData);
        break;
    }

    nm_console_print_prompt();
}

static void OnSysTimeUpdate(bool isSynchronized, int32_t timeCorrection)
{
    ClockSynchronized = isSynchronized;
    am_util_stdio_printf("\r\n");
    am_util_stdio_printf("Clock Synchronized: %d\r\n", ClockSynchronized);
    am_util_stdio_printf("Correction: %d\r\n", timeCorrection);
    am_util_stdio_printf("\r\n");
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
        LmHandlerSend(&LmAppData, LmMsgType);
    }
}

void application_handle_command()
{
    task_message_t TaskMessage;

    // do not block on message receive as the LoRa MAC state machine decides
    // when it is appropriate to sleep.  We also do not explicitly go to
    // sleep directly and simply do a task yield.  This allows other timing
    // critical radios such as BLE to run their state machines.
    if (xQueueReceive(ApplicationTaskQueue, &TaskMessage, timeout) == pdPASS) {
        switch (TaskMessage.ui32Event) {
        case JOIN:
            LmHandlerJoin();
            break;
        case SEND:
            TransmitPending = true;
            break;
        case SYNC:
            LmhpClockSyncAppTimeReq();
            break;
        case WAKE:
            break;
        }
    }
}

void application_timer_isr()
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    task_message_t TaskMessage;

    am_hal_ctimer_int_clear(APPLICATION_TIMER_INT);

    TaskMessage.ui32Event = SEND;
    xQueueSendFromISR(ApplicationTaskQueue, &TaskMessage,
                      &xHigherPriorityTaskWoken);

    sprintf((char *)psLmDataBuffer, "%lu", gui32Counter);
    LmAppData.BufferSize = strlen((char *)psLmDataBuffer);
    LmAppData.Buffer = psLmDataBuffer;
    gui32Counter++;

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void application_timer_setup()
{
    am_hal_ctimer_config_t ApplicationTimer = {
        0,
        (AM_HAL_CTIMER_FN_REPEAT | AM_HAL_CTIMER_INT_ENABLE |
         APPLICATION_CLOCK_SOURCE),
        0,
    };

    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_LFRC_START, 0);
    am_hal_ctimer_clear(APPLICATION_TIMER_NUMBER, APPLICATION_TIMER_SEGMENT);
    am_hal_ctimer_config(APPLICATION_TIMER_NUMBER, &ApplicationTimer);

    am_hal_ctimer_int_register(APPLICATION_TIMER_INT, application_timer_isr);
    am_hal_ctimer_int_clear(APPLICATION_TIMER_INT);
    NVIC_SetPriority(CTIMER_IRQn, NVIC_configKERNEL_INTERRUPT_PRIORITY);
    am_hal_ctimer_int_enable(APPLICATION_TIMER_INT);
    NVIC_EnableIRQ(CTIMER_IRQn);
}

void application_setup()
{
    LmMsgType = LORAMAC_HANDLER_UNCONFIRMED_MSG;

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
    LmCallbacks.OnSysTimeUpdate = OnSysTimeUpdate;
    LmCallbacks.OnTxData = OnTxData;
    LmCallbacks.OnRxData = OnRxData;
    LmCallbacks.OnClassChange = OnClassChange;
    LmCallbacks.OnNvmDataChange = OnNvmDataChange;

    LmHandlerErrorStatus_t status = LmHandlerInit(&LmCallbacks, &LmParameters);
    if (status != LORAMAC_HANDLER_SUCCESS) {
        am_util_stdio_printf("\r\n\r\nLoRaWAN application framework "
                             "initialization failed\r\n\r\n");
        nm_console_print_prompt();
    }
    LmHandlerSetSystemMaxRxError(20);
    LmHandlerPackageRegister(PACKAGE_ID_COMPLIANCE, &LmComplianceParams);
    LmHandlerPackageRegister(PACKAGE_ID_CLOCK_SYNC, NULL);

    gui32ApplicationTimerPeriod = APPLICATION_TRANSMIT_PERIOD;
}

void application_rtc_setup()
{
    am_hal_rtc_time_t hal_rtc_time;

    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_XTAL_START, 0);
    am_hal_rtc_osc_select(AM_HAL_RTC_OSC_XT);
    am_hal_rtc_osc_enable();

    hal_rtc_time.ui32Hour       = 0; // 0 to 23
    hal_rtc_time.ui32Minute     = 0; // 0 to 59
    hal_rtc_time.ui32Second     = 0; // 0 to 59
    hal_rtc_time.ui32Hundredths = 00;

    hal_rtc_time.ui32DayOfMonth = 1; // 1 to 31
    hal_rtc_time.ui32Month      = 0; // 0 to 11
    hal_rtc_time.ui32Year       = 0; // years since 2000
    hal_rtc_time.ui32Century    = 0;

    am_hal_rtc_time_set(&hal_rtc_time);
}

void application_task(void *pvParameters)
{
    FreeRTOS_CLIRegisterCommand(&ApplicationCommandDefinition);
    ApplicationTaskQueue = xQueueCreate(10, sizeof(task_message_t));

    am_util_stdio_printf("\r\n\r\nLoRaWAN Application Demo\r\n\r\n");
    nm_console_print_prompt();

    application_setup();
    application_rtc_setup();
    application_timer_setup();

    TransmitPending = false;
    timeout = portMAX_DELAY;
    while (1) {
        LmHandlerProcess();
        application_handle_uplink();

        if (MacProcessing) {
            taskENTER_CRITICAL();
            MacProcessing = false;
            timeout = portMAX_DELAY;
            taskEXIT_CRITICAL();
        } else {
            am_hal_gpio_state_write(AM_BSP_GPIO_LED4, AM_HAL_GPIO_OUTPUT_CLEAR);
            application_handle_command();
        }
    }
}
