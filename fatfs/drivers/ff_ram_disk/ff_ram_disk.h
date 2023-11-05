#ifndef __FF_RAMDISK_H__
#define __FF_RAMDISK_H__

#include "ff.h"
#include "diskio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
     
#ifndef FF_RAMDISK_SECTOR_SIZE
#define FF_RAMDISK_SECTOR_SIZE FF_MIN_SS     /* usualy 512 B */
#endif
     
#ifndef FF_RAMDISK_DISK_SIZE
#define FF_RAMDISK_DISK_SIZE 256 * FF_MIN_SS /* minmal disk size calculated as 256 * FF_MIN_SS (ff.c ln 4112) , 256*512=131072 */
#endif

#if defined(__cplusplus)
extern "C" {
#endif
    
/*******************************************************************************
 * API
 ******************************************************************************/
DSTATUS ram_disk_initialize(BYTE pdrv);
DSTATUS ram_disk_status(BYTE pdrv);
DRESULT ram_disk_read(BYTE pdrv, BYTE* buff, LBA_t sector, UINT count);
DRESULT ram_disk_write(BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count);
DRESULT ram_disk_ioctl(BYTE pdrv, BYTE cmd, void* buff);

#if defined(__cplusplus)
}
#endif

#endif
