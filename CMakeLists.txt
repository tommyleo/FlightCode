cmake_minimum_required(VERSION 3.24)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(FlightCode VERSION 1.0.0 LANGUAGES C ASM)

include(FetchContent)

set(BOARD "MAMBAF411" CACHE STRING "Flight controller target")
set_property(CACHE BOARD PROPERTY STRINGS MAMBAF411 CLRACINGF4 FLYWOOF405NANO FLYWOOF405NANO_ANALOG HDZERO_HALO)
string(TOUPPER "${BOARD}" BOARD)

if(BOARD STREQUAL "MAMBAF411")
    set(MCU_DEFINE STM32F411xE)
    set(STARTUP_FILE startup_stm32f411xe.s)
    set(LINKER_SCRIPT STM32F411CEUx_FLASH.ld)
elseif(BOARD STREQUAL "CLRACINGF4" OR BOARD STREQUAL "FLYWOOF405NANO" OR
       BOARD STREQUAL "FLYWOOF405NANO_ANALOG")
    set(MCU_DEFINE STM32F405xx)
    set(STARTUP_FILE startup_stm32f405xx.s)
    set(LINKER_SCRIPT STM32F405RGTx_FLASH.ld)
elseif(BOARD STREQUAL "HDZERO_HALO")
    set(MCU_DEFINE STM32H743xx)
    set(STARTUP_FILE startup_stm32h743xx.s)
    set(LINKER_SCRIPT STM32H743VITx_FLASH.ld)
    set(STM32_FAMILY_H7 TRUE)
else()
    message(FATAL_ERROR "Unsupported BOARD='${BOARD}'. Use MAMBAF411, CLRACINGF4, FLYWOOF405NANO, FLYWOOF405NANO_ANALOG, or HDZERO_HALO.")
endif()

FetchContent_Declare(cmsis_core
    GIT_REPOSITORY https://github.com/ARM-software/CMSIS_5.git
    GIT_TAG 5.9.0
    GIT_SHALLOW TRUE
)
if(STM32_FAMILY_H7)
    FetchContent_Declare(cmsis_device
        GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis-device-h7.git
        GIT_TAG v1.10.6
        GIT_SHALLOW TRUE
    )
    FetchContent_Declare(stm32_hal
        GIT_REPOSITORY https://github.com/STMicroelectronics/stm32h7xx-hal-driver.git
        GIT_TAG v1.11.5
        GIT_SHALLOW TRUE
    )
else()
    FetchContent_Declare(cmsis_device
        GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis-device-f4.git
        GIT_TAG v2.6.9
        GIT_SHALLOW TRUE
    )
    FetchContent_Declare(stm32_hal
        GIT_REPOSITORY https://github.com/STMicroelectronics/stm32f4xx-hal-driver.git
        GIT_TAG v1.8.5
        GIT_SHALLOW TRUE
    )
endif()
FetchContent_Declare(stm32_usb_device
    GIT_REPOSITORY https://github.com/STMicroelectronics/stm32-mw-usb-device.git
    GIT_TAG v2.11.4
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(cmsis_core cmsis_device stm32_hal stm32_usb_device)

set(HAL_SRC ${stm32_hal_SOURCE_DIR}/Src)

if(STM32_FAMILY_H7)
    set(SYSTEM_SOURCE ${cmsis_device_SOURCE_DIR}/Source/Templates/system_stm32h7xx.c)
    set(HAL_FAMILY_SOURCES
        ${HAL_SRC}/stm32h7xx_hal.c
        ${HAL_SRC}/stm32h7xx_hal_adc.c
        ${HAL_SRC}/stm32h7xx_hal_adc_ex.c
        ${HAL_SRC}/stm32h7xx_hal_cortex.c
        ${HAL_SRC}/stm32h7xx_hal_dma.c
        ${HAL_SRC}/stm32h7xx_hal_dma_ex.c
        ${HAL_SRC}/stm32h7xx_hal_exti.c
        ${HAL_SRC}/stm32h7xx_hal_flash.c
        ${HAL_SRC}/stm32h7xx_hal_flash_ex.c
        ${HAL_SRC}/stm32h7xx_hal_gpio.c
        ${HAL_SRC}/stm32h7xx_hal_hsem.c
        ${HAL_SRC}/stm32h7xx_hal_pwr.c
        ${HAL_SRC}/stm32h7xx_hal_pwr_ex.c
        ${HAL_SRC}/stm32h7xx_hal_rcc.c
        ${HAL_SRC}/stm32h7xx_hal_rcc_ex.c
        ${HAL_SRC}/stm32h7xx_hal_spi.c
        ${HAL_SRC}/stm32h7xx_hal_uart.c
        ${HAL_SRC}/stm32h7xx_hal_uart_ex.c
        ${HAL_SRC}/stm32h7xx_hal_pcd.c
        ${HAL_SRC}/stm32h7xx_hal_pcd_ex.c
        ${HAL_SRC}/stm32h7xx_ll_usb.c
    )
    set(CPU_FLAGS -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard)
    set(BOARD_SOURCE src/platform/board_h7.c)
    set(DSHOT_SOURCE src/drivers/motors/dshot_h7.c)
else()
    set(SYSTEM_SOURCE ${cmsis_device_SOURCE_DIR}/Source/Templates/system_stm32f4xx.c)
    set(HAL_FAMILY_SOURCES
        ${HAL_SRC}/stm32f4xx_hal.c
        ${HAL_SRC}/stm32f4xx_hal_adc.c
        ${HAL_SRC}/stm32f4xx_hal_cortex.c
        ${HAL_SRC}/stm32f4xx_hal_dma.c
        ${HAL_SRC}/stm32f4xx_hal_flash.c
        ${HAL_SRC}/stm32f4xx_hal_flash_ex.c
        ${HAL_SRC}/stm32f4xx_hal_gpio.c
        ${HAL_SRC}/stm32f4xx_hal_pwr.c
        ${HAL_SRC}/stm32f4xx_hal_pwr_ex.c
        ${HAL_SRC}/stm32f4xx_hal_rcc.c
        ${HAL_SRC}/stm32f4xx_hal_rcc_ex.c
        ${HAL_SRC}/stm32f4xx_hal_spi.c
        ${HAL_SRC}/stm32f4xx_hal_uart.c
        ${HAL_SRC}/stm32f4xx_hal_pcd.c
        ${HAL_SRC}/stm32f4xx_hal_pcd_ex.c
        ${HAL_SRC}/stm32f4xx_ll_usb.c
    )
    set(CPU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
    set(BOARD_SOURCE src/platform/board.c)
    set(DSHOT_SOURCE src/drivers/motors/dshot.c)
endif()

add_executable(FlightCode
    src/app/main.c
    src/control/flight_control.c
    src/drivers/imu/imu.c
    src/drivers/imu/icm42688p.c
    src/drivers/imu/mpu6000.c
    ${DSHOT_SOURCE}
    src/drivers/osd/max7456.c
    src/drivers/osd/msp_displayport.c
    src/drivers/osd/osd_tuning_menu.c
    src/drivers/receiver/sbus.c
    src/drivers/vtx/vtx_tramp.c
    src/drivers/usb/usb_cdc.c
    src/drivers/usb/usbd_conf.c
    src/drivers/usb/usbd_desc.c
    ${BOARD_SOURCE}
    src/platform/stm32f4xx_it.c
    src/protocol/config_protocol.c
    src/storage/flight_log.c
    src/storage/blackbox_sd.c
    src/storage/blackbox_flash.c
    src/storage/flight_settings.c
    ${SYSTEM_SOURCE}
    ${cmsis_device_SOURCE_DIR}/Source/Templates/gcc/${STARTUP_FILE}
    ${HAL_FAMILY_SOURCES}
    ${stm32_usb_device_SOURCE_DIR}/Core/Src/usbd_core.c
    ${stm32_usb_device_SOURCE_DIR}/Core/Src/usbd_ctlreq.c
    ${stm32_usb_device_SOURCE_DIR}/Core/Src/usbd_ioreq.c
    ${stm32_usb_device_SOURCE_DIR}/Class/CDC/Src/usbd_cdc.c
)
set_target_properties(FlightCode PROPERTIES SUFFIX ".elf")

target_include_directories(FlightCode PRIVATE
    src/app
    src/control
    src/drivers/imu
    src/drivers/motors
    src/drivers/osd
    src/drivers/receiver
    src/drivers/vtx
    src/drivers/usb
    src/platform
    src/protocol
    src/storage
    ${cmsis_core_SOURCE_DIR}/CMSIS/Core/Include
    ${cmsis_device_SOURCE_DIR}/Include
    ${stm32_hal_SOURCE_DIR}/Inc
    ${stm32_usb_device_SOURCE_DIR}/Core/Inc
    ${stm32_usb_device_SOURCE_DIR}/Class/CDC/Inc
)

target_compile_definitions(FlightCode PRIVATE
    ${MCU_DEFINE}
    BOARD_${BOARD}
    USE_HAL_DRIVER
    HSE_VALUE=8000000U
    $<$<BOOL:${STM32_FAMILY_H7}>:PLATFORM_STM32H7>
)

target_compile_options(FlightCode PRIVATE
    ${CPU_FLAGS}
    -ffunction-sections -fdata-sections
    -Wall -Wextra -Wshadow
    $<$<CONFIG:Debug>:-Og;-g3>
    $<$<CONFIG:Release>:-O2>
)

target_link_options(FlightCode PRIVATE
    ${CPU_FLAGS}
    -T${CMAKE_CURRENT_SOURCE_DIR}/linker/${LINKER_SCRIPT}
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/FlightCode.map
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,-u,_printf_float
    -Wl,-u,_scanf_float
)

target_link_libraries(FlightCode PRIVATE m)

add_custom_command(TARGET FlightCode POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:FlightCode> ${CMAKE_CURRENT_BINARY_DIR}/FlightCode-${BOARD}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:FlightCode> ${CMAKE_CURRENT_BINARY_DIR}/FlightCode-${BOARD}.bin
)
