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
#include <am_devices_button.h>
#include <am_devices_led.h>
#include <am_hal_ctimer.h>
#include <am_mcu_apollo.h>
#include <am_util.h>

#include <FreeRTOS.h>
#include <task.h>

#include "task_message.h"

#include "console_task.h"
#include "gpio_service.h"
#include "iom_service.h"

#include "application.h"

//*****************************************************************************
//
// Sleep function called from FreeRTOS IDLE task.
// Do necessary application specific Power down operations here
// Return 0 if this function also incorporates the WFI, else return value same
// as idleTime
//
//*****************************************************************************
uint32_t am_freertos_sleep(uint32_t idleTime)
{
    am_hal_gpio_state_write(AM_BSP_GPIO_LED0, AM_HAL_GPIO_OUTPUT_CLEAR);
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    return 0;
}

//*****************************************************************************
//
// Recovery function called from FreeRTOS IDLE task, after waking up from Sleep
// Do necessary 'wakeup' operations here, e.g. to power up/enable peripherals etc.
//
//*****************************************************************************
void am_freertos_wakeup(uint32_t idleTime)
{
    am_hal_gpio_state_write(AM_BSP_GPIO_LED0, AM_HAL_GPIO_OUTPUT_SET);

    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    task_message_t task_message;
    task_message.ui32Event = WAKE;
    xQueueSendFromISR(ApplicationTaskQueue, &task_message,
                      &xHigherPriorityTaskWoken);
}

void am_gpio_isr(void)
{
    uint64_t ui64Status;

    am_hal_gpio_interrupt_status_get(true, &ui64Status);
    am_hal_gpio_interrupt_clear(ui64Status);
    am_hal_gpio_interrupt_service(ui64Status);
}

void am_ctimer_isr(void)
{
    uint32_t ui32Status;

    ui32Status = am_hal_ctimer_int_status_get(true);
    am_hal_ctimer_int_clear(ui32Status);
    am_hal_ctimer_int_service(ui32Status);
}

//*****************************************************************************
//
// FreeRTOS debugging functions.
//
//*****************************************************************************
void vApplicationMallocFailedHook(void)
{
    //
    // Called if a call to pvPortMalloc() fails because there is insufficient
    // free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    // internally by FreeRTOS API functions that create tasks, queues, software
    // timers, and semaphores.  The size of the FreeRTOS heap is set by the
    // configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h.
    //
    while (1)
        ;
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pcTaskName;
    (void)pxTask;

    //
    // Run time stack overflow checking is performed if
    // configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    // function is called if a stack overflow is detected.
    //
    while (1) {
        __asm("BKPT #0\n"); // Break into the debugger
    }
}

void system_setup(void)
{
    //
    // Set the clock frequency.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();

    am_hal_sysctrl_fpu_enable();
    am_hal_sysctrl_fpu_stacking_enable(true);

    //
    // Configure the board for low power.
    //
    am_hal_pwrctrl_low_power_init();
    am_hal_rtc_osc_disable();

    //
    // Initialize any board specific peripherals
    //
    am_devices_led_array_init(am_bsp_psLEDs, AM_BSP_NUM_LEDS);
    am_devices_led_array_out(am_bsp_psLEDs, AM_BSP_NUM_LEDS, 0x0);
    am_devices_button_array_init(am_bsp_psButtons, AM_BSP_NUM_BUTTONS);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_LED0, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_LED0, AM_HAL_GPIO_OUTPUT_SET);

    am_hal_interrupt_master_enable();
}

void system_start(void)
{
    // Setup tasks to register the GPIO and IOM commands in the console.
    // These are run at the highest priority to ensure that the commands
    // registered before the console starts.
    xTaskCreate(nm_gpio_task, "GPIO", 512, 0, 3, &nm_gpio_task_handle);
    xTaskCreate(nm_iom_task, "IOM", 512, 0, 3, &nm_iom_task_handle);

    xTaskCreate(nm_console_task, "Console", 512, 0, 2, &nm_console_task_handle);
    xTaskCreate(application_task, "Application", 512, 0, 2,
                &application_task_handle);

    //
    // Start the scheduler.
    //
    vTaskStartScheduler();
}

int main(void)
{
    system_setup();
    system_start();

    while (1) {
    }

    return 0;
}
