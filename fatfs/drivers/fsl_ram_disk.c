#include "ffconf.h"
#include "fsl_common.h"
#include "fsl_ram_disk.h"

/*******************************************************************************
 * Globals
 ******************************************************************************/
static uint8_t disk_space[FF_RAMDISK_DISK_SIZE];

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Get RAM disk status.
 */
DSTATUS ram_disk_status(BYTE pdrv)
{
    if (pdrv != RAMDISK)
    {
        return STA_NOINIT;
    }
    return 0;
}

/*!
 * @brief Inidialize a RAM disk.
 */
DSTATUS ram_disk_initialize(BYTE pdrv)
{
    if (pdrv != RAMDISK)
    {
        return STA_NOINIT;
    }
    return 0;
}

/*!
 * @brief Read Sector(s) from RAM disk.
 */
DRESULT ram_disk_read(BYTE pdrv, BYTE* buff, LBA_t sector, UINT count)
{
    if (pdrv != RAMDISK)
    {
        return RES_PARERR;
    }
    memcpy(buff, disk_space + sector * FF_RAMDISK_SECTOR_SIZE, FF_RAMDISK_SECTOR_SIZE * count);
    return RES_OK;
}

/*!
 * @brief Write Sector(s) to RAM disk.
 */
DRESULT ram_disk_write(BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count)
{
    if (pdrv != RAMDISK)
    {
        return RES_PARERR;
    }
    memcpy(disk_space + sector * FF_RAMDISK_SECTOR_SIZE, buff, FF_RAMDISK_SECTOR_SIZE * count);
    return RES_OK;
}

/*!
 * @brief Miscellaneous RAM disk Functions.
 */
DRESULT ram_disk_ioctl(BYTE pdrv, BYTE cmd, void* buff)
{
    if (pdrv != RAMDISK)
    {
        return RES_PARERR;
    }
    switch (cmd)
    {
        case GET_SECTOR_COUNT:
            *(uint32_t *)buff = FF_RAMDISK_DISK_SIZE / FF_RAMDISK_SECTOR_SIZE;
            return RES_OK;
            break;
        case GET_SECTOR_SIZE:
            *(uint32_t *)buff = FF_RAMDISK_SECTOR_SIZE;
            return RES_OK;
            break;
        case CTRL_SYNC:
            return RES_OK;
            break;
        default:
            break;
    }
    return RES_PARERR;
}