#ifndef _FF_SDSPI_DISK_H_
#define _FF_SDSPI_DISK_H_

#include "ff.h"
#include "diskio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*************************************************************************************************
 * API - SD disk interface
 ************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name SD over SPI Disk Function
 * @{
 */

/*!
 * @brief Initializes SD disk over SPI.
 *
 * @param physicalDrive Physical drive number.
 * @retval STA_NOINIT Failed.
 * @retval RES_OK Success.
 */
DSTATUS sdspi_disk_initialize(uint8_t physicalDrive);

/*!
 * Gets SD over SPI disk status
 *
 * @param physicalDrive Physical drive number.
 * @retval STA_NOINIT Failed.
 * @retval RES_OK Success.
 */
DSTATUS sdspi_disk_status(uint8_t physicalDrive);

/*!
 * @brief Reads SD disk over SPI.
 *
 * @param physicalDrive Physical drive number.
 * @param buffer The data buffer pointer to store read content.
 * @param sector The start sector number to be read.
 * @param count The sector count to be read.
 * @retval RES_PARERR Failed.
 * @retval RES_OK Success.
 */
DRESULT sdspi_disk_read(uint8_t physicalDrive, uint8_t *buffer, uint32_t sector, uint8_t count);

/*!
 * @brief Writes to SD disk over SPI.
 *
 * @param physicalDrive Physical drive number.
 * @param buffer The data buffer pointer to store write content.
 * @param sector The start sector number to be written.
 * @param count The sector count to be written.
 * @retval RES_PARERR Failed.
 * @retval RES_OK Success.
 */
DRESULT sdspi_disk_write(uint8_t physicalDrive, const uint8_t *buffer, uint32_t sector, uint8_t count);

/*!
 * @brief SD over SPI disk IO operation.
 *
 * @param physicalDrive Physical drive number.
 * @param command The command to be set.
 * @param buffer The buffer to store command result.
 * @retval RES_PARERR Failed.
 * @retval RES_OK Success.
 */
DRESULT sdspi_disk_ioctl(uint8_t physicalDrive, uint8_t command, void *buffer);

/* @} */
#if defined(__cplusplus)
}
#endif

#endif /* _FF_SDSPI_DISK_H_ */
