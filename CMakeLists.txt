cmake_minimum_required(VERSION 3.24)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(FlightCode C ASM)

include(FetchContent)

set(BOARD "MAMBAF411" CACHE STRING "Flight controller target")
set_property(CACHE BOARD PROPERTY STRINGS MAMBAF411 CLRACINGF4)
string(TOUPPER "${BOARD}" BOARD)

if(BOARD STREQUAL "MAMBAF411")
    set(MCU_DEFINE STM32F411xE)
    set(STARTUP_FILE startup_stm32f411xe.s)
    set(LINKER_SCRIPT STM32F411CEUx_FLASH.ld)
elseif(BOARD STREQUAL "CLRACINGF4")
    set(MCU_DEFINE STM32F405xx)
    set(STARTUP_FILE startup_stm32f405xx.s)
    set(LINKER_SCRIPT STM32F405RGTx_FLASH.ld)
else()
    message(FATAL_ERROR "Unsupported BOARD='${BOARD}'. Use MAMBAF411 or CLRACINGF4.")
endif()

FetchContent_Declare(cmsis_core
    GIT_REPOSITORY https://github.com/ARM-software/CMSIS_5.git
    GIT_TAG 5.9.0
    GIT_SHALLOW TRUE
)
FetchContent_Declare(cmsis_device_f4
    GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis-device-f4.git
    GIT_TAG v2.6.9
    GIT_SHALLOW TRUE
)
FetchContent_Declare(stm32f4_hal
    GIT_REPOSITORY https://github.com/STMicroelectronics/stm32f4xx-hal-driver.git
    GIT_TAG v1.8.5
    GIT_SHALLOW TRUE
)
FetchContent_Declare(stm32_usb_device
    GIT_REPOSITORY https://github.com/STMicroelectronics/stm32-mw-usb-device.git
    GIT_TAG v2.11.4
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(cmsis_core cmsis_device_f4 stm32f4_hal stm32_usb_device)

set(HAL_SRC ${stm32f4_hal_SOURCE_DIR}/Src)

add_executable(FlightCode
    src/app/main.c
    src/control/flight_control.c
    src/drivers/imu/mpu6000.c
    src/drivers/motors/dshot.c
    src/drivers/osd/max7456.c
    src/drivers/receiver/sbus.c
    src/drivers/usb/usb_cdc.c
    src/drivers/usb/usbd_conf.c
    src/drivers/usb/usbd_desc.c
    src/platform/board.c
    src/platform/stm32f4xx_it.c
    src/protocol/config_protocol.c
    src/storage/flight_log.c
    src/storage/flight_settings.c
    ${cmsis_device_f4_SOURCE_DIR}/Source/Templates/system_stm32f4xx.c
    ${cmsis_device_f4_SOURCE_DIR}/Source/Templates/gcc/${STARTUP_FILE}
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
    src/drivers/usb
    src/platform
    src/protocol
    src/storage
    ${cmsis_core_SOURCE_DIR}/CMSIS/Core/Include
    ${cmsis_device_f4_SOURCE_DIR}/Include
    ${stm32f4_hal_SOURCE_DIR}/Inc
    ${stm32_usb_device_SOURCE_DIR}/Core/Inc
    ${stm32_usb_device_SOURCE_DIR}/Class/CDC/Inc
)

target_compile_definitions(FlightCode PRIVATE
    ${MCU_DEFINE}
    BOARD_${BOARD}
    USE_HAL_DRIVER
    HSE_VALUE=8000000U
)

target_compile_options(FlightCode PRIVATE
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    -ffunction-sections -fdata-sections
    -Wall -Wextra -Wshadow
    $<$<CONFIG:Debug>:-Og;-g3>
    $<$<CONFIG:Release>:-O2>
)

target_link_options(FlightCode PRIVATE
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
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
