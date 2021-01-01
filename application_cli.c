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
#include <NvmDataMgmt.h>
#include <board.h>
#include <timer.h>

#include "application.h"
#include "application_cli.h"
#include "console_task.h"
#include "task_message.h"

portBASE_TYPE prvApplicationCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                    const char *pcCommandString);

CLI_Command_Definition_t ApplicationCommandDefinition = {
    (const char *const) "lorawan",
    (const char *const) "lorawan:\tLoRaWAN Application Framework.\r\n",
    prvApplicationCommand, -1};

static void ConvertHexString(const char *in, size_t inlen, uint8_t *out,
                             size_t *outlen)
{
    size_t n = 0;
    char cNum[3];
    *outlen = 0;
    while (n < inlen) {
        switch (in[n]) {
        case '\\':
            n++;
            switch (in[n]) {
            case 'x':
                n++;
                memset(cNum, 0, 3);
                memcpy(cNum, &in[n], 2);
                n++;
                out[*outlen] = strtol(cNum, NULL, 16);
                break;
            }
            break;
        default:
            out[*outlen] = in[n];
            break;
        }
        *outlen = *outlen + 1;
        n++;
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
        strcat(pcWriteBuffer, "  reset\r\n");
        strcat(pcWriteBuffer, "  send\r\n");
        strcat(pcWriteBuffer, "  periodic\r\n");
        strcat(pcWriteBuffer, "  set\r\n");
        strcat(pcWriteBuffer, "  get\r\n");
        strcat(pcWriteBuffer, "\r\n");
        strcat(
            pcWriteBuffer,
            "See 'lorawan help [command] for the details of each command.\r\n");
    } else if (strncmp(pcParameterString, "join", 4) == 0) {
        strcat(pcWriteBuffer, "usage: lorawan join\r\n");
        strcat(pcWriteBuffer, "Join a LoRaWAN network.\r\n");
    } else if (strncmp(pcParameterString, "reset", 5) == 0) {
        strcat(pcWriteBuffer, "usage: lorawan reset\r\n");
        strcat(pcWriteBuffer, "Stop and reset the LoRaMac stack.\r\n");
    } else if (strncmp(pcParameterString, "send", 3) == 0) {
        strcat(pcWriteBuffer, "usage: lorawan send <port> <ack> [msg]\r\n");
        strcat(pcWriteBuffer, "\r\n");
        strcat(pcWriteBuffer, "Where:\r\n");
        strcat(pcWriteBuffer, "  port  is the uplink port number\r\n");
        strcat(pcWriteBuffer,
               "  ack   request message confirmation from the server\r\n");
        strcat(pcWriteBuffer, "  msg   payload content\r\n");
    } else if (strncmp(pcParameterString, "periodic", 8) == 0) {
        strcat(pcWriteBuffer,
               "usage: lorawan periodic [start <period>|stop]\r\n");
        strcat(pcWriteBuffer, "\r\n");
        strcat(pcWriteBuffer, "Where:\r\n");
        strcat(pcWriteBuffer,
               "  start   to begin transmitting periodically\r\n");
        strcat(pcWriteBuffer, "  stop    to stop transmitting\r\n");
        strcat(pcWriteBuffer, "  period  defines how often to transmit in "
                              "seconds (default is 10s)\r\n");
    } else if (strncmp(pcParameterString, "set", 3) == 0) {
        //TODO
    } else if (strncmp(pcParameterString, "get", 3) == 0) {
        //TODO
    }
}

void prvApplicationSendSubCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                  const char *pcCommandString)
{
    const char *pcParameterString;
    portBASE_TYPE xParameterStringLength;

    LmHandlerMsgTypes_t ack = LmMsgType;
    uint8_t port = LM_APPLICATION_PORT;
    uint8_t argc = FreeRTOS_CLIGetNumberOfParameters(pcCommandString);

    pcParameterString =
        FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength);
    switch (argc) {
    case 2: {
        memcpy(psLmDataBuffer, pcParameterString, xParameterStringLength);
    } break;
    case 3: {
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
    }
    case 4: {
        pcParameterString = FreeRTOS_CLIGetParameter(pcCommandString, 2,
                                                     &xParameterStringLength);
        if (pcParameterString == NULL) {
            strcat(pcWriteBuffer,
                   "error: missing acknowledgement parameter\r\n");
            return;
        } else {
            ack = atoi(pcParameterString) > 0 ? LORAMAC_HANDLER_CONFIRMED_MSG
                                              : LORAMAC_HANDLER_UNCONFIRMED_MSG;
            LmMsgType = ack;
        }
    } break;
    }

    size_t length;
    ConvertHexString(pcParameterString, xParameterStringLength, psLmDataBuffer,
                     &length);

    LmAppData.Port = port;
    LmAppData.BufferSize = length;
    LmAppData.Buffer = psLmDataBuffer;

    am_util_stdio_printf(psLmDataBuffer);
    am_util_stdio_printf("\r\n");

    task_message_t TaskMessage;
    TaskMessage.ui32Event = SEND;
    TaskMessage.psContent = &LmAppData;
    xQueueSend(ApplicationTaskQueue, &TaskMessage, portMAX_DELAY);
}

void prvApplicationPeriodicSubCommand(char *pcWriteBuffer,
                                      size_t xWriteBufferLen,
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

        pcParameterString = FreeRTOS_CLIGetParameter(pcCommandString, 3,
                                                     &xParameterStringLength);
        if (pcParameterString == NULL) {
            gui32ApplicationTimerPeriod = 5;
        } else {
            gui32ApplicationTimerPeriod = atoi(pcParameterString);
        }

        uint32_t ui32Period =
            gui32ApplicationTimerPeriod * APPLICATION_TIMER_PERIOD;
        am_hal_ctimer_period_set(APPLICATION_TIMER_NUMBER,
                                 APPLICATION_TIMER_SEGMENT, ui32Period,
                                 (ui32Period >> 1));
        am_hal_ctimer_start(APPLICATION_TIMER_NUMBER,
                            APPLICATION_TIMER_SEGMENT);
    } else if (strncmp(pcParameterString, "stop", xParameterStringLength) ==
               0) {
        am_hal_ctimer_stop(APPLICATION_TIMER_NUMBER, APPLICATION_TIMER_SEGMENT);
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
    } else if (strncmp(pcParameterString, "reset", xParameterStringLength) ==
               0) {
        LoRaMacStop();
    } else if (strncmp(pcParameterString, "send", xParameterStringLength) ==
               0) {
        prvApplicationSendSubCommand(pcWriteBuffer, xWriteBufferLen,
                                     pcCommandString);
    } else if (strncmp(pcParameterString, "periodic", xParameterStringLength) ==
               0) {
        prvApplicationPeriodicSubCommand(pcWriteBuffer, xWriteBufferLen,
                                         pcCommandString);
    } else if (strncmp(pcParameterString, "set", xParameterStringLength) == 0) {
        // TODO
    } else if (strncmp(pcParameterString, "get", xParameterStringLength) == 0) {
        // TODO
    }

    return pdFALSE;
}
