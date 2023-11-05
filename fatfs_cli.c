/*
 *  BSD 3-Clause License
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
#include <stdlib.h>
#include <string.h>

#include <am_mcu_apollo.h>
#include <am_util.h>

#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>

#include "ff.h"
#include "diskio.h"
#include "fatfs_cli.h"

static size_t argc;
static char *argv[8];
static char argz[128];
static char path[128];

static portBASE_TYPE fatfs_cli_cat_entry(char *pui8OutBuffer,
                                        size_t ui32OutBufferLength,
                                        const char *pui8Command);
static CLI_Command_Definition_t fatfs_cli_cat_definition = {
    (const char *const) "cat",
    (const char *const) "cat     :  concatenate FILE to standard output.\r\n",
    fatfs_cli_cat_entry,
    -1
};

static portBASE_TYPE fatfs_cli_cd_entry(char *pui8OutBuffer,
                                        size_t ui32OutBufferLength,
                                        const char *pui8Command);
static CLI_Command_Definition_t fatfs_cli_cd_definition = {
    (const char *const) "cd",
    (const char *const) "cd     :  change directory.\r\n",
    fatfs_cli_cd_entry,
    -1
};

static portBASE_TYPE fatfs_cli_ls_entry(char *pui8OutBuffer,
                                        size_t ui32OutBufferLength,
                                        const char *pui8Command);
static CLI_Command_Definition_t fatfs_cli_ls_definition = {
    (const char *const) "ls",
    (const char *const) "ls     :  list files.\r\n",
    fatfs_cli_ls_entry,
    -1
};

static portBASE_TYPE fatfs_cli_pwd_entry(char *pui8OutBuffer,
                                        size_t ui32OutBufferLength,
                                        const char *pui8Command);
static CLI_Command_Definition_t fatfs_cli_pwd_definition = {
    (const char *const) "pwd",
    (const char *const) "pwd    :  get current path.\r\n",
    fatfs_cli_pwd_entry,
    -1
};

void fatfs_cli_register(void)
{
    argc = 0;
    FreeRTOS_CLIRegisterCommand(&fatfs_cli_cat_definition);
    FreeRTOS_CLIRegisterCommand(&fatfs_cli_cd_definition);
    FreeRTOS_CLIRegisterCommand(&fatfs_cli_ls_definition);
    FreeRTOS_CLIRegisterCommand(&fatfs_cli_pwd_definition);
}

static portBASE_TYPE fatfs_cli_cat_entry(char *pui8OutBuffer,
                                         size_t ui32OutBufferLength,
                                         const char *pui8Command)
{
    static uint32_t cat_state = 0;
    static FIL file;
    FRESULT res;

    pui8OutBuffer[0] = 0;
    strcpy(argz, pui8Command);
    FreeRTOS_CLIExtractParameters(argz, &argc, argv);

    if (argc == 1)
    {
        return pdFALSE;
    }

    switch (cat_state)
    {
    case 0:
        res = f_open(&file, argv[1], FA_READ);
        if (res != FR_OK)
        {
            am_util_stdio_printf("cat: '%s': Is a directory or no such file\r\n", argv[1]);
            return pdFALSE;
        }
        else
        {
            cat_state = 1;
            return pdTRUE;
        }
        break;

    case 1:
        if (f_gets(pui8OutBuffer, ui32OutBufferLength, &file))
        {
            return pdTRUE;
        }
        else
        {
            f_close(&file);
            cat_state = 0;
            return pdFALSE;
        }
    }

    return pdFALSE;
}

static portBASE_TYPE fatfs_cli_cd_entry(char *pui8OutBuffer,
                                        size_t ui32OutBufferLength,
                                        const char *pui8Command)
{
    FRESULT res;

    pui8OutBuffer[0] = 0;
    strcpy(argz, pui8Command);
    FreeRTOS_CLIExtractParameters(argz, &argc, argv);

    if (argc > 1)
    {
        strcpy(path, argv[argc-1]);
    }
    else
    {
        strcpy(path, ".");
    }

    res = f_chdir(path);
    if (res != FR_OK)
    {
        am_util_stdio_printf("cd: '%s': No such file or directory\r\n", path);
    }

    return pdFALSE;
}


static portBASE_TYPE fatfs_cli_ls_entry(char *pui8OutBuffer,
                                        size_t ui32OutBufferLength,
                                        const char *pui8Command)
{
    FRESULT res;
    DIR dir;


    pui8OutBuffer[0] = 0;
    strcpy(argz, pui8Command);
    FreeRTOS_CLIExtractParameters(argz, &argc, argv);
    if (argc > 1)
    {
        strcpy(path, argv[argc-1]);
    }
    else
    {
        strcpy(path, ".");
    }

    res = f_opendir(&dir, path);
    if (res == FR_OK)
    {
        for (;;)
        {
            FILINFO fno;
            res = f_readdir(&dir, &fno);
            if (res != FR_OK || fno.fname[0] == 0)
                break;

            if (fno.fattrib & AM_DIR)
            {
                am_util_stdio_printf(" %s/\n", fno.fname);
            }
            else
            {
                am_util_stdio_printf(" %s\n", fno.fname);
            }
        }
    }
    else if (res == FR_NO_PATH)
    {
        FILINFO fno;
        res = f_stat(path, &fno);
        if (fno.fattrib != AM_DIR)
        {
            am_util_stdio_sprintf(pui8OutBuffer, " %10u %s\n", fno.fsize, fno.fname);
        }
    }
    else
    {
        goto error;
    }
    f_closedir(&dir);
    return pdFALSE;

error:
    am_util_stdio_printf("ls: cannot access '%s': No such file or directory\r\n", path);
    return pdFALSE;
}

static portBASE_TYPE fatfs_cli_pwd_entry(char *pui8OutBuffer,
                                         size_t ui32OutBufferLength,
                                         const char *pui8Command)
{
    pui8OutBuffer[0] = 0;
    f_getcwd(pui8OutBuffer, ui32OutBufferLength);
    return pdFALSE;
}
