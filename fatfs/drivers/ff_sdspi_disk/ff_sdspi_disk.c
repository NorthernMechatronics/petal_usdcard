#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <am_mcu_apollo.h>
#include <am_util.h>
#include <am_bsp.h>

#include "ff.h"
#include "diskio.h"
#include "sdspi.h"

/*******************************************************************************
 * Definitons
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void spi_init(void);
static status_t spi_set_frequency(uint32_t frequency);
static status_t spi_exchange(uint8_t *in, uint8_t *out, uint32_t size);
static void sdspi_host_init(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static sdspi_card_t g_card;
static sdspi_host_t g_host;

/*******************************************************************************
 * Code - SD disk interface
 ******************************************************************************/

DRESULT sdspi_disk_write(uint8_t physicalDrive, const uint8_t *buffer, uint32_t sector, uint8_t count)
{
    if (physicalDrive != DEV_SDSPI_DISK)
    {
        return RES_PARERR;
    }

    if (kStatus_Success != SDSPI_WriteBlocks(&g_card, (uint8_t *)buffer, sector, count))
    {
        return RES_ERROR;
    }
    return RES_OK;
}

DRESULT sdspi_disk_read(uint8_t physicalDrive, uint8_t *buffer, uint32_t sector, uint8_t count)
{
    if (physicalDrive != DEV_SDSPI_DISK)
    {
        return RES_PARERR;
    }

    if (kStatus_Success != SDSPI_ReadBlocks(&g_card, buffer, sector, count))
    {
        return RES_ERROR;
    }
    return RES_OK;
}

DRESULT sdspi_disk_ioctl(uint8_t physicalDrive, uint8_t command, void *buffer)
{
    DRESULT result = RES_OK;

    if (physicalDrive != DEV_SDSPI_DISK)
    {
        return RES_PARERR;
    }

    switch (command)
    {
        case GET_SECTOR_COUNT:
            if (buffer)
            {
                *(uint32_t *)buffer = g_card.blockCount;
            }
            else
            {
                result = RES_PARERR;
            }
            break;
        case GET_SECTOR_SIZE:
            if (buffer)
            {
                *(uint32_t *)buffer = g_card.blockSize;
            }
            else
            {
                result = RES_PARERR;
            }
            break;
        case GET_BLOCK_SIZE:
            if (buffer)
            {
                *(uint32_t *)buffer = g_card.csd.eraseSectorSize;
            }
            else
            {
                result = RES_PARERR;
            }
            break;
        case CTRL_SYNC:
            result = RES_OK;
            break;
        default:
            result = RES_PARERR;
            break;
    }

    return result;
}

DSTATUS sdspi_disk_status(uint8_t physicalDrive)
{
    if (physicalDrive != DEV_SDSPI_DISK)
    {
        return STA_NOINIT;
    }
    return RES_OK;
}

DSTATUS sdspi_disk_initialize(uint8_t physicalDrive)
{
    if (physicalDrive == DEV_SDSPI_DISK)
    {
        spi_init();
        sdspi_host_init();
        SDSPI_Init(&g_card);
        g_card.host = &g_host;
        return RES_OK;
    }
    return STA_NOINIT;
}

/*******************************************************************************
 * Code - SPI interface
 ******************************************************************************/

void spi_init(void)
{
    am_hal_gpio_pinconfig(AM_BSP_GPIO_SD_EN, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_SD_EN, AM_HAL_GPIO_OUTPUT_SET);

    // uint32_t sourceClock;

    // dspi_master_config_t masterConfig;

    // /*Master config*/
    // masterConfig.whichCtar = DSPI_MASTER_CTAR;
    // masterConfig.ctarConfig.baudRate = DSPI_BUS_BAUDRATE;
    // masterConfig.ctarConfig.bitsPerFrame = 8;
    // masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
    // masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;
    // masterConfig.ctarConfig.direction = kDSPI_MsbFirst;
    // masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 0;
    // masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 0;
    // masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 0;

    // masterConfig.whichPcs = DSPI_MASTER_PCS_CONFIG;
    // masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

    // masterConfig.enableContinuousSCK = false;
    // masterConfig.enableRxFifoOverWrite = false;
    // masterConfig.enableModifiedTimingFormat = false;
    // masterConfig.samplePoint = kDSPI_SckToSin0Clock;

    // sourceClock = CLOCK_GetFreq(DSPI_MASTER_CLK_SRC);
    // DSPI_MasterInit((SPI_Type *)BOARD_SDSPI_SPI_BASE, &masterConfig, sourceClock);
}

status_t spi_set_frequency(uint32_t frequency)
{
    // uint32_t sourceClock;

    // sourceClock = CLOCK_GetFreq(DSPI_MASTER_CLK_SRC);
    // /* If returns 0, indicates failed. */
    // if (DSPI_MasterSetBaudRate((SPI_Type *)BOARD_SDSPI_SPI_BASE, DSPI_MASTER_CTAR, frequency, sourceClock))
    // {
    //     return kStatus_Success;
    // }

    return kStatus_Fail;
}

status_t spi_exchange(uint8_t *in, uint8_t *out, uint32_t size)
{
    // dspi_transfer_t masterTransfer;

    // masterTransfer.txData = in;
    // masterTransfer.rxData = out;
    // masterTransfer.dataSize = size;
    // masterTransfer.configFlags = (kDSPI_MasterCtar0 | DSPI_MASTER_PCS_TRANSFER | kDSPI_MasterPcsContinuous);
    // return DSPI_MasterTransferBlocking((SPI_Type *)BOARD_SDSPI_SPI_BASE, &masterTransfer);

    return kStatus_Success;
}


void sdspi_host_init(void)
{
    /* Saves host state and callback. */
    g_host.busBaudRate = SD_CLOCK_48MHZ;
    g_host.setFrequency = spi_set_frequency;
    g_host.exchange = spi_exchange;

    /* Saves card state. */
    g_card.host = &g_host;
}