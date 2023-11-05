#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <am_mcu_apollo.h>
#include <am_util.h>
#include <am_bsp.h>

#include "ff.h"
#include "diskio.h"
#include "spi_sdcard_driver.h"

/*******************************************************************************
 * Definitons
 ******************************************************************************/
#define SDSPI_IOM_MODULE    (0)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void spi_init(void);
static void spi_set_speed(uint32_t frequency);
static void spi_select(void);
static void spi_release(void);
static bool spi_is_present(void);
static uint8_t spi_wr_rd_byte(uint8_t byte);
static void spi_write(uint8_t const *buffer, uint32_t size);
static void spi_read(uint8_t *buffer, uint32_t size);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static void *sdspi_iom_handle = NULL;
static spisd_interface_t spisd_interface;
static spisd_info_t spisd_info;

/*******************************************************************************
 * Code - SD disk interface
 ******************************************************************************/

DRESULT sdspi_disk_write(uint8_t physicalDrive, const uint8_t *buffer, uint32_t sector, uint8_t count)
{
    if (physicalDrive != DEV_SDSPI_DISK)
    {
        return RES_PARERR;
    }

    if (SPISD_RESULT_OK != spisd_write_multi_block(sector, buffer, count))
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

    if (SPISD_RESULT_OK != spisd_read_multi_block_begin(sector))
    {
        return RES_ERROR;
    }
    if (SPISD_RESULT_OK != spisd_read_multi_block_read(buffer, count))
    {
        return RES_ERROR;
    }
    if (SPISD_RESULT_OK != spisd_read_multi_block_end())
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
//                *(uint32_t *)buffer = g_card.blockCount;
            }
            else
            {
                result = RES_PARERR;
            }
            break;
        case GET_SECTOR_SIZE:
            if (buffer)
            {
//                *(uint32_t *)buffer = g_card.blockSize;
            }
            else
            {
                result = RES_PARERR;
            }
            break;
        case GET_BLOCK_SIZE:
            if (buffer)
            {
//                *(uint32_t *)buffer = g_card.csd.eraseSectorSize;
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

        spisd_interface.set_speed = spi_set_speed;
        spisd_interface.select = spi_select;
        spisd_interface.release = spi_release;
        spisd_interface.is_present = spi_is_present;
        spisd_interface.wr_rd_byte = spi_wr_rd_byte;
        spisd_interface.write = spi_write;
        spisd_interface.read = spi_read;

        spisd_init(&spisd_interface);
        spisd_get_card_info(&spisd_info);

        return RES_OK;
    }
    return STA_NOINIT;
}

/*******************************************************************************
 * Code - SPI interface
 ******************************************************************************/

void spi_init(void)
{
    am_hal_iom_config_t iom_spi_config = {
        .eInterfaceMode = AM_HAL_IOM_SPI_MODE,
        .ui32ClockFreq = AM_HAL_IOM_400KHZ,
        .eSpiMode = AM_HAL_IOM_SPI_MODE_0,
    };

    am_hal_gpio_pinconfig(AM_BSP_GPIO_SD_EN, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_SD_EN, AM_HAL_GPIO_OUTPUT_SET);

    am_hal_iom_initialize(SDSPI_IOM_MODULE, &sdspi_iom_handle);
    am_hal_iom_power_ctrl(sdspi_iom_handle, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_iom_configure(sdspi_iom_handle, &iom_spi_config);
    am_hal_iom_enable(sdspi_iom_handle);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_SD_MISO, g_AM_BSP_GPIO_SD_MISO);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_SD_MOSI, g_AM_BSP_GPIO_SD_MOSI);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_SD_SCK, g_AM_BSP_GPIO_SD_SCK);
//    am_hal_gpio_pinconfig(AM_BSP_GPIO_SD_CS, g_AM_BSP_GPIO_SD_CS);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_SD_CS, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_SD_CS, AM_HAL_GPIO_OUTPUT_SET);
}

static void spi_set_speed(uint32_t frequency)
{
    am_hal_iom_config_t iom_spi_config = {
        .eInterfaceMode = AM_HAL_IOM_SPI_MODE,
        .ui32ClockFreq = frequency,
        .eSpiMode = AM_HAL_IOM_SPI_MODE_0,
    };

    am_hal_iom_disable(sdspi_iom_handle);
    am_hal_iom_configure(sdspi_iom_handle, &iom_spi_config);
    am_hal_iom_enable(sdspi_iom_handle);
}

static void spi_select(void)
{
    am_hal_gpio_state_write(AM_BSP_GPIO_SD_CS, AM_HAL_GPIO_OUTPUT_CLEAR);
}

static void spi_release(void)
{
    am_hal_gpio_state_write(AM_BSP_GPIO_SD_CS, AM_HAL_GPIO_OUTPUT_SET);
}

static bool spi_is_present(void)
{
    return true;
}

static uint8_t spi_wr_rd_byte(uint8_t byte)
{
    am_hal_iom_transfer_t transfer;
    uint32_t tx = byte;
    uint32_t rx;

    transfer.ui32InstrLen = 0;
    transfer.ui32Instr    = 0;
    transfer.eDirection   = AM_HAL_IOM_FULLDUPLEX;
    transfer.ui32NumBytes = 1;
    transfer.pui32TxBuffer = &tx;
    transfer.pui32RxBuffer = &rx;
    transfer.bContinue      = false;
    transfer.ui8RepeatCount = 0;
    transfer.ui32PauseCondition = 0;
    transfer.ui32StatusSetClr   = 0;
    transfer.uPeerInfo.ui32SpiChipSelect = 0;

    am_hal_iom_spi_blocking_fullduplex(sdspi_iom_handle, &transfer);

    return rx;
}

static void spi_write(uint8_t const *buffer, uint32_t size)
{
    am_hal_iom_transfer_t transfer;

    transfer.ui32InstrLen = 0;
    transfer.ui32Instr    = 0;
    transfer.eDirection   = AM_HAL_IOM_TX;
    transfer.ui32NumBytes = size;
    transfer.pui32TxBuffer = (uint32_t *)buffer;
    transfer.pui32RxBuffer = 0;
    transfer.bContinue      = false;
    transfer.ui8RepeatCount = 0;
    transfer.ui32PauseCondition = 0;
    transfer.ui32StatusSetClr   = 0;
    transfer.uPeerInfo.ui32SpiChipSelect = 0;

    am_hal_iom_blocking_transfer(sdspi_iom_handle, &transfer);
}

static void spi_read(uint8_t *buffer, uint32_t size)
{
    am_hal_iom_transfer_t transfer;

    transfer.ui32InstrLen = 0;
    transfer.ui32Instr    = 0;
    transfer.eDirection   = AM_HAL_IOM_RX;
    transfer.ui32NumBytes = size;
    transfer.pui32TxBuffer = 0;
    transfer.pui32RxBuffer = (uint32_t *)buffer;
    transfer.bContinue      = false;
    transfer.ui8RepeatCount = 0;
    transfer.ui32PauseCondition = 0;
    transfer.ui32StatusSetClr   = 0;
    transfer.uPeerInfo.ui32SpiChipSelect = 0;

    am_hal_iom_blocking_transfer(sdspi_iom_handle, &transfer);
}
