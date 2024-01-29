/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Northern Mechatronics, Inc.
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
#include <am_mcu_apollo.h>
#include <am_util.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <timers.h>

#include "am_bsp.h"

#include "ff.h"
#include "diskio.h"

#include "application_task.h"
#include "application_task_cli.h"
#include "fatfs_cli.h"

const TCHAR g_drive[3U] = {DEV_SDSPI_DISK + '0', ':', '/'};
FATFS g_fileSystem;
uint8_t g_fsWork[FF_MAX_SS];

static TaskHandle_t application_task_handle;
static QueueHandle_t application_queue_handle;
static TimerHandle_t application_timer_handle;

static void application_timer_callback(TimerHandle_t xTimer)
{
    application_message_t message = {.message = 0, .payload_size = 0, .payload = NULL};
    message.message = APP_MSG_ALIVE_LED_TOGGLE;
    application_task_send(&message);
}

static void sdcard_det_handler(void)
{
    application_message_t message = {.message = 0, .payload_size = 0, .payload = NULL};

    // disable interrupt to handle debounce in application context
    AM_HAL_GPIO_MASKCREATE(sMasknDET);
    AM_HAL_GPIO_MASKBIT(psMasknDET, AM_BSP_GPIO_SD_nDET);
    am_hal_gpio_interrupt_disable(psMasknDET);

    message.message = APP_MSG_CARD_STATUS_CHANGED;
    application_task_send(&message);
}

static void filesystem_setup(void)
{
    uint32_t card_status;

    // check if an SD card is inserted
    am_hal_gpio_state_read(AM_BSP_GPIO_SD_nDET, AM_HAL_GPIO_INPUT_READ, &card_status);
    if (card_status == 0)
    {
        // power on the SD Card
        am_hal_gpio_state_write(AM_BSP_GPIO_SD_EN, AM_HAL_GPIO_OUTPUT_SET);
        vTaskDelay(pdMS_TO_TICKS(10));

        FRESULT fr = f_mount(&g_fileSystem, g_drive, 0);
        if ((fr == FR_INVALID_DRIVE) || (fr == FR_NOT_READY))
        {
            am_util_stdio_printf("\r\nDrive not found\r\n");
            return;
        }
        else if (fr == FR_NO_FILESYSTEM)
        {
            MKFS_PARM g_formatOptions = {FM_EXFAT, 1, 0, 0, 0};

            am_util_stdio_printf("\r\nNo filesystem, formatting...\r\n");
            taskENTER_CRITICAL();
            f_mkfs(g_drive, &g_formatOptions, g_fsWork, FF_MAX_SS);
            taskEXIT_CRITICAL();
            am_util_stdio_printf("Format completed.\r\n");
        }
        else if (fr == FR_DISK_ERR)
        {
            am_util_stdio_printf("\r\nDisk Error\r\n");
            return;
        }
        else
        {
            am_util_stdio_printf("\r\nFilesystem Mounted\r\n");
        }

        f_chdrive(g_drive);
    }
    else
    {
        // power off the SD card
        am_hal_gpio_state_write(AM_BSP_GPIO_SD_EN, AM_HAL_GPIO_OUTPUT_CLEAR);
        am_util_stdio_printf("\r\nSD card removed\r\n");

        // umount the file system
        f_unmount(g_drive);
    }
}

static void application_setup()
{
    // All the Petal development board IO pins can be turned on and off
    // by GPIO 30.
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IO_EN, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_IO_EN, AM_HAL_GPIO_OUTPUT_SET);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_LED0, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_LED0, AM_HAL_GPIO_OUTPUT_CLEAR);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_LED1, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_LED1, AM_HAL_GPIO_OUTPUT_CLEAR);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_LED2, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_LED2, AM_HAL_GPIO_OUTPUT_CLEAR);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_LED3, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_LED3, AM_HAL_GPIO_OUTPUT_CLEAR);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_LED4, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_LED4, AM_HAL_GPIO_OUTPUT_CLEAR);
}

static void sdcard_setup(void)
{
    // SD card power control is performed inside filesystem_setup.
    // Defer SD card power up until we have confirmation that an SD
    // is inserted.
    am_hal_gpio_pinconfig(AM_BSP_GPIO_SD_EN, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_SD_EN, AM_HAL_GPIO_OUTPUT_CLEAR);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_SD_nDET, g_AM_BSP_GPIO_SD_nDET);
    AM_HAL_GPIO_MASKCREATE(sMasknDET);
    AM_HAL_GPIO_MASKBIT(psMasknDET, AM_BSP_GPIO_SD_nDET);
    am_hal_gpio_interrupt_register(AM_BSP_GPIO_SD_nDET, sdcard_det_handler);
    am_hal_gpio_interrupt_clear(psMasknDET);
    am_hal_gpio_interrupt_enable(psMasknDET);

    NVIC_EnableIRQ(GPIO_IRQn);
    am_hal_interrupt_master_enable();
}

static void application_task(void *parameter)
{
    application_message_t message;

    application_task_cli_register();
    fatfs_cli_register();

    application_setup();
    sdcard_setup();
    filesystem_setup();

    xTimerStart(application_timer_handle, portMAX_DELAY);
    while (1)
    {
        if (xQueueReceive(application_queue_handle, &message, portMAX_DELAY) == pdTRUE)
        {
            switch (message.message)
            {
            case APP_MSG_CARD_STATUS_CHANGED:
            {
                // debounce for 100ms
                vTaskDelay(pdMS_TO_TICKS(100));
                AM_HAL_GPIO_MASKCREATE(sMasknDET);
                AM_HAL_GPIO_MASKBIT(psMasknDET, AM_BSP_GPIO_SD_nDET);
                am_hal_gpio_interrupt_clear(psMasknDET);
                am_hal_gpio_interrupt_enable(psMasknDET);

                filesystem_setup();
            }
            break;
            case APP_MSG_ALIVE_LED_TOGGLE:
                am_hal_gpio_state_write(AM_BSP_GPIO_LED0, AM_HAL_GPIO_OUTPUT_TOGGLE);
                break;
            }
        }
    }
}

void application_task_create(uint32_t priority)
{
    application_queue_handle = xQueueCreate(8, sizeof(uint32_t));
    application_timer_handle = xTimerCreate(
        "Alive LED Timer", pdMS_TO_TICKS(1000), pdTRUE, NULL, application_timer_callback);
    xTaskCreate(application_task, "application", 512, 0, priority, &application_task_handle);
}

void application_task_send(application_message_t *message)
{
    if (application_queue_handle)
    {
        BaseType_t xHigherPriorityTaskWoken;

        if (xPortIsInsideInterrupt() == pdTRUE)
        {
            xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(application_queue_handle, message, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        else
        {
            xQueueSend(application_queue_handle, message, portMAX_DELAY);
        }
    }
}