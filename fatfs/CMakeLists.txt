add_library(fatfs STATIC)

set(FATFS_DIR ${CMAKE_CURRENT_LIST_DIR})

set (FATFS_INCLUDES
    ${FATFS_DIR}/source
    ${FATFS_DIR}/drivers
    ${FATFS_DIR}/drivers/ff_ram_disk
    ${FATFS_DIR}/drivers/ff_sdspi_disk
    ${FATFS_DIR}/drivers/ff_sdspi_disk/sdcard_spi_driver
    CACHE PATH "FATFS includes"
)

target_compile_definitions(
    fatfs PRIVATE
    RAM_DISK_ENABLE
    SDSPI_DISK_ENABLE
)

target_include_directories(
    fatfs PRIVATE
    ${HAL_INCLUDES}
    ${RTOS_INCLUDES}
    ${FATFS_INCLUDES}
)

target_sources(
    fatfs PRIVATE 
    ${FATFS_DIR}/source/diskio.c
    ${FATFS_DIR}/source/ff.c
    ${FATFS_DIR}/source/ffsystem.c
    ${FATFS_DIR}/source/ffunicode.c
    ${FATFS_DIR}/drivers/ff_ram_disk/ff_ram_disk.c
    ${FATFS_DIR}/drivers/ff_sdspi_disk/ff_sdspi_disk.c
    ${FATFS_DIR}/drivers/ff_sdspi_disk/sdcard_spi_driver/fsl_sdspi.c
)