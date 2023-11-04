/*-----------------------------------------------------------------------*/
/* Low level disk I/O module SKELETON for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv)
	{
#ifdef RAM_DISK_ENABLE
	case DEV_RAM_DISK:
		stat = ram_disk_status(pdrv);
		return stat;
#endif
#ifdef SDSPI_DISK_ENABLE
	case DEV_SDSPI_DISK:
		stat = sdspi_disk_status(pdrv);
		return stat;
#endif
#ifdef NAND_DISK_ENABLE
	case DEV_NAND_DISK:
		stat = nand_disk_status(pdrv);
		return stat;
#endif
        default:
            break;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv)
	{
#ifdef RAM_DISK_ENABLE
	case DEV_RAM_DISK:
		stat = ram_disk_initialize(pdrv);
		return stat;
#endif
#ifdef SDSPI_DISK_ENABLE
	case DEV_SDSPI_DISK:
		stat = sdspi_disk_initialize(pdrv);
		return stat;
#endif
#ifdef NAND_DISK_ENABLE
	case DEV_NAND_DISK:
		stat = nand_disk_initialize(pdrv);
		return stat;
#endif
	default:
		break;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res;
	int result;

	switch (pdrv)
	{
#ifdef RAM_DISK_ENABLE
	case DEV_RAM_DISK:
		res = ram_disk_read(pdrv, buff, sector, count);
		return res;
#endif
#ifdef SDSPI_DISK_ENABLE
	case DEV_SDSPI_DISK:
		res = sdspi_disk_read(pdrv, buff, sector, count);
		return res;
#endif
#ifdef NAND_DISK_ENABLE
	case DEV_NAND_DISK:
		res = nand_disk_read(pdrv, buff, sector, count);
		return res;
#endif
	default:
		break;
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT res;
	int result;

	switch (pdrv)
	{
#ifdef RAM_DISK_ENABLE
	case DEV_RAM_DISK:
		res = ram_disk_write(pdrv, buff, sector, count);
		return res;
#endif
#ifdef SDSPI_DISK_ENABLE
	case DEV_SDSPI_DISK:
		res = sdspi_disk_write(pdrv, buff, sector, count);
		return res;
#endif
#ifdef NAND_DISK_ENABLE
	case DEV_NAND_DISK:
		res = nand_disk_write(pdrv, buff, sector, count);
		return res;
#endif
	default:
		break;
	}

	return RES_PARERR;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	int result;

	switch (pdrv)
	{
#ifdef RAM_DISK_ENABLE
	case RAMDISK:
		res = ram_disk_ioctl(pdrv, cmd, buff);
		return res;
#endif
#ifdef SDSPI_DISK_ENABLE
	case SDSPIDISK:
		res = sdspi_disk_ioctl(pdrv, cmd, buff);
		return res;
#endif
#ifdef NAND_DISK_ENABLE
	case NANDDISK:
		res = nand_disk_ioctl(pdrv, cmd, buff);
		return res;
#endif
	default:
		break;
	}

	return RES_PARERR;
}

