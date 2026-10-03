# Artery BSP is kept external, pinned to an exact revision. Only its peripheral
# and USB device library is linked; no Betaflight flight-control code is linked.
FetchContent_Declare(artery
    GIT_REPOSITORY https://github.com/ArteryTek/AT32F435_437_Firmware_Library.git
    GIT_TAG 4aa5607a57e368541494dff247d06e84d94e3edc
)
FetchContent_MakeAvailable(artery)
set(AT32_DEVICE ${artery_SOURCE_DIR}/libraries/cmsis/cm4/device_support)
set(AT32_DRIVERS ${artery_SOURCE_DIR}/libraries/drivers)
set(AT32_USB ${artery_SOURCE_DIR}/middlewares/usb_drivers)
set(AT32_CDC ${artery_SOURCE_DIR}/middlewares/usbd_class/cdc)
set(AT32_VENDOR_SOURCES)
foreach(driver crm gpio spi usart tmr dma adc flash pwc misc usb)
    list(APPEND AT32_VENDOR_SOURCES ${AT32_DRIVERS}/src/at32f435_437_${driver}.c)
endforeach()
add_executable(FlightCode
    src/app/main.c
    src/control/flight_control.c
    src/drivers/imu/imu.c
    src/drivers/imu/icm42688p.c
    src/drivers/imu/mpu6000.c
    src/drivers/motors/dshot_at32.c
    src/protocol/esc_passthrough.c
    src/drivers/osd/max7456.c
    src/drivers/osd/msp_displayport.c
    src/drivers/osd/osd_tuning_menu.c
    src/drivers/receiver/sbus.c
    src/drivers/vtx/vtx_tramp.c
    src/drivers/vtx/vtx_smartaudio.c
    src/drivers/usb/usb_cdc_at32.c
    src/platform/at32/board_at32.c
    src/platform/at32/flightcode_at32.c
    src/platform/at32/cdc_desc.c
    src/protocol/config_protocol.c
    src/storage/flight_log.c
    src/storage/blackbox_sd.c
    src/storage/blackbox_flash.c
    src/storage/flight_settings.c
    ${AT32_DEVICE}/system_at32f435_437.c
    ${AT32_DEVICE}/startup/gcc/startup_at32f435_437.s
    ${AT32_VENDOR_SOURCES}
    ${AT32_USB}/src/usb_core.c
    ${AT32_USB}/src/usbd_core.c
    ${AT32_USB}/src/usbd_int.c
    ${AT32_USB}/src/usbd_sdr.c
    ${AT32_CDC}/cdc_class.c
)
set_target_properties(FlightCode PROPERTIES SUFFIX ".elf")
target_include_directories(FlightCode PRIVATE
    src/platform/at32
    src/app src/control src/drivers/imu src/drivers/motors src/drivers/osd
    src/drivers/receiver src/drivers/vtx src/drivers/usb src/platform
    src/protocol src/storage
    ${AT32_DEVICE}
    ${artery_SOURCE_DIR}/libraries/cmsis/cm4/core_support
    ${AT32_DRIVERS}/inc ${AT32_USB}/inc ${AT32_CDC}
)
target_compile_definitions(FlightCode PRIVATE
    AT32F435RGT7 PLATFORM_AT32 BOARD_HUMMINGBIRD_200RS
    HEXT_VALUE=8000000U FLIGHTCODE_VERSION="${PROJECT_VERSION}"
)
set(AT32_CPU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
target_compile_options(FlightCode PRIVATE
    ${AT32_CPU_FLAGS} -ffunction-sections -fdata-sections -Wall -Wextra -Wshadow
    $<$<CONFIG:Debug>:-Og;-g3> $<$<CONFIG:Release>:-O2>
)
target_link_options(FlightCode PRIVATE
    ${AT32_CPU_FLAGS}
    -T${CMAKE_CURRENT_SOURCE_DIR}/linker/AT32F435G_FLASH.ld
    -Wl,--gc-sections -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/FlightCode.map
    --specs=nano.specs --specs=nosys.specs
    -Wl,-u,_printf_float -Wl,-u,_scanf_float
)
target_link_libraries(FlightCode PRIVATE m)
add_custom_command(TARGET FlightCode POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:FlightCode> ${CMAKE_CURRENT_BINARY_DIR}/FlightCode-${BOARD}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:FlightCode> ${CMAKE_CURRENT_BINARY_DIR}/FlightCode-${BOARD}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:FlightCode>
)
