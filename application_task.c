/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Northern Mechatronics, Inc.
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
#include <task.h>

#include "am_bsp.h"

#include "ff.h"
#include "diskio.h"

#include "application_task.h"
#include "application_task_cli.h"
#include "fatfs_cli.h"

const TCHAR g_drive[3U] = { DEV_SDSPI_DISK + '0', ':', '/'};
//const TCHAR g_drive[3U] = { DEV_RAM_DISK + '0', ':', '/'};
FATFS g_fileSystem;
uint8_t g_fsWork[FF_MAX_SS];
char buffer[128];

static TaskHandle_t application_task_handle;

static void application_setup_task()
{
    am_hal_gpio_pinconfig(30, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(30, AM_HAL_GPIO_OUTPUT_SET);

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

    am_hal_gpio_pinconfig(AM_BSP_GPIO_SD_EN, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_SD_EN, AM_HAL_GPIO_OUTPUT_SET);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_SD_CS, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_SD_CS, AM_HAL_GPIO_OUTPUT_CLEAR);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_SD_MOSI, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_SD_MOSI, AM_HAL_GPIO_OUTPUT_CLEAR);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_SD_MISO, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_SD_MISO, AM_HAL_GPIO_OUTPUT_CLEAR);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_SD_SCK, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_SD_SCK, AM_HAL_GPIO_OUTPUT_CLEAR);
}

static void filesystem_setup(void)
{
    MKFS_PARM g_formatOptions = { FM_EXFAT, 1, 0, 0, 0 };

    FRESULT fr;
    FIL g_file;
    UINT bw;

    fr = f_mount(&g_fileSystem, g_drive, 1);
    if ((fr == FR_INVALID_DRIVE) || (fr == FR_NOT_READY))
    {
        am_util_stdio_printf("\r\nDrive not found\r\n");
    }
    else if (fr == FR_NO_FILESYSTEM)
    {
        am_util_stdio_printf("\r\nNo filesystem, formatting...\r\n");
        f_mkfs(g_drive, &g_formatOptions, g_fsWork, FF_MAX_SS);
        am_util_stdio_printf("Format completed.\r\n");
    }
    else
    {
        am_util_stdio_printf("\r\nFilesystem Mounted\r\n");
    }
}

static void gpio_toggle()
{
    am_hal_gpio_state_write(AM_BSP_GPIO_SD_CS, AM_HAL_GPIO_OUTPUT_TOGGLE);
    am_hal_gpio_state_write(AM_BSP_GPIO_SD_MOSI, AM_HAL_GPIO_OUTPUT_TOGGLE);
    am_hal_gpio_state_write(AM_BSP_GPIO_SD_MISO, AM_HAL_GPIO_OUTPUT_TOGGLE);
    am_hal_gpio_state_write(AM_BSP_GPIO_SD_SCK, AM_HAL_GPIO_OUTPUT_TOGGLE);
}

static void application_task(void *parameter)
{
    uint32_t card;
    application_task_cli_register();
    fatfs_cli_register();

    application_setup_task();
    filesystem_setup();
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        am_hal_gpio_state_write(AM_BSP_GPIO_LED0, AM_HAL_GPIO_OUTPUT_TOGGLE);
    }
}

void application_task_create(uint32_t priority)
{
    xTaskCreate(application_task, "application", 512, 0, priority, &application_task_handle);
}