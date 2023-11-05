cmake_policy(SET CMP0048 NEW)
cmake_minimum_required(VERSION 3.13.0)

# if (NOT "$ENV{APPLICATION}" STREQUAL "")
#     set(APPLICATION $ENV{APPLICATION})
# else()
#     set(APPLICATION default_app_name)
# endif()

get_filename_component(APPLICATION ${CMAKE_CURRENT_SOURCE_DIR} NAME_WE)

if (NOT "$ENV{APPLICATION_NAME}" STREQUAL "")
    set(APPLICATION_NAME $ENV{APPLICATION_NAME})
else()
    set(APPLICATION_NAME ${APPLICATION})
endif()

set(TARGET "nm180100" CACHE STRING "" FORCE)

option(BSP_NM180100EVB "" OFF)
option(BSP_NM180310 "" OFF)
option(BSP_NM180410 "" OFF)

option(RAT_BLE_ENABLE "" OFF)
option(RAT_LORAWAN_ENABLE "" OFF)
option(TF_ENABLE "" OFF)

if (BSP_NM180100EVB)
add_definitions(-DBSP_NM180100EVB)
set(BSP_TARGET_DIR nm180100evb CACHE STRING "" FORCE)
endif()

if (BSP_NM180310)
add_definitions(-DBSP_NM180310)
set(BSP_TARGET_DIR nm180310 CACHE STRING "" FORCE)
endif()

if (BSP_NM180410)
add_definitions(-DBSP_NM180410)
set(BSP_TARGET_DIR nm180410 CACHE STRING "" FORCE)
endif()

add_subdirectory(nmsdk2)

project(${APPLICATION})

add_executable(${APPLICATION})
set_target_properties(
    ${APPLICATION}
    PROPERTIES
        OUTPUT_NAME ${APPLICATION_NAME}
        SUFFIX ".axf"
)

add_subdirectory(bsp)
add_subdirectory(fatfs)

target_link_libraries(
    ${APPLICATION}
    PUBLIC
        -T${PROJECT_SOURCE_DIR}/AMA3B1KK.ld
        -static
        -Wl,--start-group
        -L${CMSIS_LIB_DIR}
        rtos
        bsp
        hal
        fatfs
        -lm
        -lc
        -lgcc
        -lCMSISDSP
        -lcmsis-nn
        -Wl,--end-group
        -Wl,--gc-sections
)

target_compile_definitions(
    ${APPLICATION}
    PRIVATE
)

target_include_directories(
    ${APPLICATION}
    PRIVATE
    ${HAL_INCLUDES}
    ${RTOS_INCLUDES}
    ${FATFS_INCLUDES}
    ${PROJECT_SOURCE_DIR}/bsp
    ${PROJECT_SOURCE_DIR}/bsp/${BSP_TARGET_DIR}
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/utils/RTT
    ${PROJECT_SOURCE_DIR}/utils/RTT/Config
    ${PROJECT_SOURCE_DIR}/utils/RTT/RTT
)

target_sources(
    ${APPLICATION}
    PRIVATE
    startup_gcc.c
    main.c
    stub.c
    application_task_cli.c
    application_task.c
    console_task.c
    fatfs_cli.c

    utils/RTT/RTT/SEGGER_RTT.c
    utils/RTT/RTT/SEGGER_RTT_printf.c
)

add_custom_command(
    TARGET ${APPLICATION}
    POST_BUILD
    # COMMAND echo "Creating list file"
    # COMMAND ${CMAKE_OBJDUMP} -S $<TARGET_FILE_NAME:${APPLICATION}> > $<TARGET_FILE_BASE_NAME:${APPLICATION}>.lst
    COMMAND echo "Creating bin file"
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE_NAME:${APPLICATION}> $<TARGET_FILE_BASE_NAME:${APPLICATION}>.bin
)

add_dependencies(fatfs bsp hal rtos)
add_dependencies(${APPLICATION} hal bsp rtos fatfs)