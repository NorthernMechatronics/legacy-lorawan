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

#include "eeprom_emulation.h"
#include "eeprom_emulation_conf.h"

#include "ota_config.h"

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
    while (n < inlen)
    {
        switch (in[n])
        {
        case '\\':
            n++;
            switch (in[n])
            {
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

    if (pcParameterString == NULL)
    {
        strcat(pcWriteBuffer, "usage: lorawan [command] [<args>]\r\n");
        strcat(pcWriteBuffer, "\r\n");
        strcat(pcWriteBuffer, "Supported commands are:\r\n");
        strcat(pcWriteBuffer, "  join\r\n");
        strcat(pcWriteBuffer, "  reset\r\n");
        strcat(pcWriteBuffer, "  class\r\n");
        strcat(pcWriteBuffer, "  send\r\n");
        strcat(pcWriteBuffer, "  periodic\r\n");
        strcat(pcWriteBuffer, "  format\r\n");
        strcat(pcWriteBuffer, "  sync\r\n");
        strcat(pcWriteBuffer, "  datetime\r\n");
        strcat(pcWriteBuffer, "\r\n");
        strcat(pcWriteBuffer, "See 'lorawan help [command] for the details "
                              "of each command.\r\n");
    }
    else if (strncmp(pcParameterString, "join", 4) == 0)
    {
        strcat(pcWriteBuffer, "usage: lorawan join\r\n");
        strcat(pcWriteBuffer, "Join a LoRaWAN network.\r\n");
    }
    else if (strncmp(pcParameterString, "reset", 5) == 0)
    {
        strcat(pcWriteBuffer, "usage: lorawan reset\r\n");
        strcat(pcWriteBuffer, "Stop and reset the LoRaMac stack.\r\n");
    }
    else if (strncmp(pcParameterString, "class", 5) == 0)
    {
        strcat(pcWriteBuffer, "usage: lorawan class [get | set <class>]\r\n");
        strcat(pcWriteBuffer, "\r\n");
    }
    else if (strncmp(pcParameterString, "send", 3) == 0)
    {
        strcat(pcWriteBuffer, "usage: lorawan send <port> <ack> [msg]\r\n");
        strcat(pcWriteBuffer, "\r\n");
        strcat(pcWriteBuffer, "Where:\r\n");
        strcat(pcWriteBuffer, "  port  is the uplink port number\r\n");
        strcat(pcWriteBuffer,
               "  ack   request message confirmation from the server\r\n");
        strcat(pcWriteBuffer, "  msg   payload content\r\n");
    }
    else if (strncmp(pcParameterString, "periodic", 8) == 0)
    {
        strcat(pcWriteBuffer,
               "usage: lorawan periodic [start <period>|stop]\r\n");
        strcat(pcWriteBuffer, "\r\n");
        strcat(pcWriteBuffer, "Where:\r\n");
        strcat(pcWriteBuffer,
               "  start   to begin transmitting periodically\r\n");
        strcat(pcWriteBuffer, "  stop    to stop transmitting\r\n");
        strcat(pcWriteBuffer, "  period  defines how often to transmit in "
                              "seconds (default is 10s)\r\n");
    }
    else if (strncmp(pcParameterString, "format", 3) == 0)
    {
        strcat(pcWriteBuffer, "usage: lorawan format\r\n");
        strcat(pcWriteBuffer, "delete and format context storage.\r\n");
    }
    else if (strncmp(pcParameterString, "sync", 3) == 0)
    {
        strcat(pcWriteBuffer, "usage: lorawan sync [app|mac]\r\n");
        strcat(pcWriteBuffer,
               "app  time synchronization using the application layer.\r\n");
        strcat(pcWriteBuffer,
               "mac  time synchronization using the MAC layer.\r\n");
        strcat(pcWriteBuffer, "Note:  Only valid if the network server "
                              "supports the command.\r\n");
    }
    else if (strncmp(pcParameterString, "datetime", 3) == 0)
    {
        strcat(pcWriteBuffer, "usage: lorawan datetime\r\n");
        strcat(pcWriteBuffer, "Display the current system time.\r\n");
    }
}

void prvApplicationSendSubCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                  const char *pcCommandString)
{
    const char *pcParameterString;
    portBASE_TYPE xParameterStringLength;

    LmHandlerMsgTypes_t ack = LORAMAC_HANDLER_UNCONFIRMED_MSG;
    uint8_t port = LM_APPLICATION_PORT;
    uint8_t argc = FreeRTOS_CLIGetNumberOfParameters(pcCommandString);

    pcParameterString =
        FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength);
    switch (argc)
    {
    case 2:
    {
        memcpy(psLmDataBuffer, pcParameterString, xParameterStringLength);
    }
    break;
    case 3:
    {
        pcParameterString = FreeRTOS_CLIGetParameter(pcCommandString, 2,
                                                     &xParameterStringLength);
        if (pcParameterString == NULL)
        {
            strcat(pcWriteBuffer, "error: missing port number\r\n");
            return;
        }
        else
        {
            port = atoi(pcParameterString);
        }

        pcParameterString = FreeRTOS_CLIGetParameter(pcCommandString, 3,
                                                     &xParameterStringLength);
        memcpy(psLmDataBuffer, pcParameterString, xParameterStringLength);
    }
    break;
    case 4:
    {
        pcParameterString = FreeRTOS_CLIGetParameter(pcCommandString, 2,
                                                     &xParameterStringLength);
        if (pcParameterString == NULL)
        {
            strcat(pcWriteBuffer, "error: missing port number\r\n");
            return;
        }
        else
        {
            port = atoi(pcParameterString);
        }

        pcParameterString = FreeRTOS_CLIGetParameter(pcCommandString, 3,
                                                     &xParameterStringLength);
        if (pcParameterString == NULL)
        {
            strcat(pcWriteBuffer,
                   "error: missing acknowledgement parameter\r\n");
            return;
        }
        else
        {
            ack = atoi(pcParameterString) > 0 ? LORAMAC_HANDLER_CONFIRMED_MSG
                                              : LORAMAC_HANDLER_UNCONFIRMED_MSG;
        }
        pcParameterString = FreeRTOS_CLIGetParameter(pcCommandString, 4,
                                                     &xParameterStringLength);
        memcpy(psLmDataBuffer, pcParameterString, xParameterStringLength);
    }
    break;
    }

    size_t length;
    ConvertHexString(pcParameterString, xParameterStringLength, psLmDataBuffer,
                     &length);

    LmAppData.Port = port;
    LmAppData.BufferSize = length;
    LmAppData.Buffer = psLmDataBuffer;
    LmMsgType = ack;

    psLmDataBuffer[length] = 0;
    am_util_stdio_printf((char *)psLmDataBuffer);
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
    if (pcParameterString == NULL)
    {
        return;
    }

    if (strncmp(pcParameterString, "start", xParameterStringLength) == 0)
    {

        pcParameterString = FreeRTOS_CLIGetParameter(pcCommandString, 3,
                                                     &xParameterStringLength);
        if (pcParameterString == NULL)
        {
            gui32ApplicationTimerPeriod = 5;
        }
        else
        {
            gui32ApplicationTimerPeriod = atoi(pcParameterString);
        }

        uint32_t ui32Period =
            gui32ApplicationTimerPeriod * APPLICATION_TIMER_PERIOD;
        am_hal_ctimer_period_set(APPLICATION_TIMER_NUMBER,
                                 APPLICATION_TIMER_SEGMENT, ui32Period,
                                 (ui32Period >> 1));
        am_hal_ctimer_start(APPLICATION_TIMER_NUMBER,
                            APPLICATION_TIMER_SEGMENT);
    }
    else if (strncmp(pcParameterString, "stop", xParameterStringLength) == 0)
    {
        am_hal_ctimer_stop(APPLICATION_TIMER_NUMBER, APPLICATION_TIMER_SEGMENT);
    }
}

void prvApplicationSyncSubCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                  const char *pcCommandString)
{
    task_message_t TaskMessage;

    const char *pcParameterString;
    portBASE_TYPE xParameterStringLength;

    pcParameterString =
        FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength);
    if (pcParameterString == NULL)
    {
        return;
    }

    if (strncmp(pcParameterString, "app", xParameterStringLength) == 0)
    {
        TaskMessage.ui32Event = SYNC_APP;
        xQueueSend(ApplicationTaskQueue, &TaskMessage, portMAX_DELAY);
    }
    else if (strncmp(pcParameterString, "mac", xParameterStringLength) == 0)
    {
        TaskMessage.ui32Event = SYNC_MAC;
        xQueueSend(ApplicationTaskQueue, &TaskMessage, portMAX_DELAY);
    }
}

void prvApplicationDatetimeSubCommand(char *pcWriteBuffer,
                                      size_t xWriteBufferLen,
                                      const char *pcCommandString)
{
    am_hal_rtc_time_t hal_rtc_time;
    am_hal_rtc_time_get(&hal_rtc_time);

    struct tm ts;

    ts.tm_hour = hal_rtc_time.ui32Hour;
    ts.tm_min = hal_rtc_time.ui32Minute;
    ts.tm_sec = hal_rtc_time.ui32Second;
    ts.tm_year = hal_rtc_time.ui32Year + 2000 - 1900;
    ts.tm_mon = hal_rtc_time.ui32Month;
    ts.tm_mday = hal_rtc_time.ui32DayOfMonth;

    char *buf = pcWriteBuffer + strlen(pcWriteBuffer);
    strftime(buf, 64, "Hardware Time: %Y-%m-%d %H:%M:%S %Z\r\n", &ts);

    SysTime_t curtime = SysTimeGet();
    buf += strlen(buf);
    am_util_stdio_sprintf(buf, "LoRaMAC Stack Time: %d\r\n", curtime.Seconds);
}

void prvApplicationClassSubCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                  const char *pcCommandString)
{
    task_message_t TaskMessage;

    const char *pcParameterString;
    portBASE_TYPE xParameterStringLength;

    pcParameterString =
        FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength);
    if (pcParameterString == NULL)
    {
        return;
    }

    if (strncmp(pcParameterString, "get", xParameterStringLength) == 0)
    {
        DeviceClass_t cls = LmHandlerGetCurrentClass();
        switch(cls)
        {
        case CLASS_A:
            am_util_stdio_printf("Class A\r\n");
            break;
        case CLASS_B:
            am_util_stdio_printf("Class B\r\n");
            break;
        case CLASS_C:
            am_util_stdio_printf("Class C\r\n");
            break;
        }
    }
    else if (strncmp(pcParameterString, "set", xParameterStringLength) == 0)
    {
        pcParameterString =
            FreeRTOS_CLIGetParameter(pcCommandString, 3, &xParameterStringLength);
        if (pcParameterString == NULL)
        {
            am_util_stdio_printf("Missing class specifier\r\n");
            return;
        }

        if (strcmp(pcParameterString, "A") == 0)
        {
            LmHandlerRequestClass(CLASS_A);
            am_util_stdio_printf("Class A requested\r\n");
        }
        else if (strcmp(pcParameterString, "B") == 0)
        {
            LmHandlerRequestClass(CLASS_B);
            am_util_stdio_printf("Class B requested\r\n");
        }
        else if (strcmp(pcParameterString, "C") == 0)
        {
            LmHandlerRequestClass(CLASS_C);
            am_util_stdio_printf("Class C requested\r\n");
        }
    }
}

void prvApplicationOtaSubCommand(char *pcWriteBuffer,
                                      size_t xWriteBufferLen,
                                      const char *pcCommandString)
{
    const char *pcParameterString;
    portBASE_TYPE xParameterStringLength;

    pcParameterString =
        FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength);

    if (pcParameterString == NULL)
    {
        uint32_t *pOtaDesc = (uint32_t *)(OTA_POINTER_LOCATION);
        am_hal_ota_init(AM_HAL_FLASH_PROGRAM_KEY, pOtaDesc);

        uint8_t  magic = ((uint8_t *)OTA_FLASH_ADDRESS)[3];
        am_hal_ota_add(AM_HAL_FLASH_PROGRAM_KEY, magic, (uint32_t *)(OTA_FLASH_ADDRESS));

        return;
    }

    if (strncmp(pcParameterString, "erase", xParameterStringLength) == 0)
    {
        pcParameterString =
            FreeRTOS_CLIGetParameter(pcCommandString, 3, &xParameterStringLength);
        if (pcParameterString == NULL)
        {
            am_util_stdio_printf("Missing address\r\n");
            return;
        }

        uint32_t address = strtol(pcParameterString, NULL, 0);

        am_hal_flash_page_erase(AM_HAL_FLASH_PROGRAM_KEY,
                AM_HAL_FLASH_ADDR2INST(address),
                AM_HAL_FLASH_ADDR2PAGE(address));
    }
    else if (strncmp(pcParameterString, "write", xParameterStringLength) == 0)
    {
        pcParameterString =
            FreeRTOS_CLIGetParameter(pcCommandString, 3, &xParameterStringLength);
        if (pcParameterString == NULL)
        {
            am_util_stdio_printf("Missing address\r\n");
            return;
        }
        uint32_t address = strtol(pcParameterString, NULL, 0);

        pcParameterString =
            FreeRTOS_CLIGetParameter(pcCommandString, 4, &xParameterStringLength);
        if (pcParameterString == NULL)
        {
            am_util_stdio_printf("Missing value\r\n");
            return;
        }

        uint32_t value = strtol(pcParameterString, NULL, 0);

        am_hal_flash_program_main(AM_HAL_FLASH_PROGRAM_KEY, &value,
                (uint32_t *)address, 1);
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
    if (pcParameterString == NULL)
    {
        return pdFALSE;
    }

    if (strncmp(pcParameterString, "help", xParameterStringLength) == 0)
    {
        prvApplicationHelpSubCommand(pcWriteBuffer, xWriteBufferLen,
                                     pcCommandString);
    }
    else if (strncmp(pcParameterString, "join", xParameterStringLength) == 0)
    {
        task_message_t TaskMessage;
        TaskMessage.ui32Event = JOIN;
        xQueueSend(ApplicationTaskQueue, &TaskMessage, portMAX_DELAY);
    }
    else if (strncmp(pcParameterString, "reset", xParameterStringLength) == 0)
    {
        LoRaMacDeInitialization();
    }
    else if (strncmp(pcParameterString, "class", xParameterStringLength) == 0)
    {
        prvApplicationClassSubCommand(pcWriteBuffer, xWriteBufferLen,
                pcCommandString);
    }
    else if (strncmp(pcParameterString, "send", xParameterStringLength) == 0)
    {
        prvApplicationSendSubCommand(pcWriteBuffer, xWriteBufferLen,
                                     pcCommandString);
    }
    else if (strncmp(pcParameterString, "periodic", xParameterStringLength) ==
             0)
    {
        prvApplicationPeriodicSubCommand(pcWriteBuffer, xWriteBufferLen,
                                         pcCommandString);
    }
    else if (strncmp(pcParameterString, "format", xParameterStringLength) == 0)
    {
        eeprom_format(EEPROM_EMULATION_FLASH_PAGES);
        eeprom_init(EEPROM_EMULATION_FLASH_PAGES);
    }
    else if (strncmp(pcParameterString, "sync", xParameterStringLength) == 0)
    {
        prvApplicationSyncSubCommand(pcWriteBuffer, xWriteBufferLen,
                                     pcCommandString);
    }
    else if (strncmp(pcParameterString, "datetime", xParameterStringLength) ==
             0)
    {
        prvApplicationDatetimeSubCommand(pcWriteBuffer, xWriteBufferLen,
                                         pcCommandString);
    }
    else if (strncmp(pcParameterString, "ota", xParameterStringLength) == 0)
    {
        prvApplicationOtaSubCommand(pcWriteBuffer, xWriteBufferLen,
                                         pcCommandString);
    }

    return pdFALSE;
}
