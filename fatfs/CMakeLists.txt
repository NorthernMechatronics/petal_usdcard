add_library(fatfs STATIC)

set(FATFS_DIR ${CMAKE_CURRENT_LIST_DIR})

set (FATFS_INCLUDES
    ${FATFS_DIR}/source
    ${FATFS_DIR}/drivers
    ${FATFS_DIR}/drivers/ff_ram_disk
    CACHE PATH "FATFS includes"
)

target_include_directories(
    fatfs PRIVATE
    ${HAL_INCLUDES}
    ${FATFS_INCLUDES}
)

target_sources(
    fatfs PRIVATE 
    ${FATFS_DIR}/source/diskio.c
    ${FATFS_DIR}/source/ff.c
    ${FATFS_DIR}/source/ffsystem.c
    ${FATFS_DIR}/source/ffunicode.c
    ${FATFS_DIR}/drivers/ff_ram_disk/ff_ram_disk.c
)