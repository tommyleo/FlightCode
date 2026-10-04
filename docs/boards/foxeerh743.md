# Foxeer H743 (`FOXEERH743`)

[Back to supported boards](../../README.md#supported-flight-controllers)

FlightCode target based on the Betaflight Foxeer H743 pin map. The MPU6000 and
ICM-42688-P drivers are probed automatically on SPI2. Betaflight also lists an
MPU6500 option; that sensor variant has not been separately validated here.
This target requires bench validation on a physical board before flight.

The [Foxeer H7 Mini MPU6000](foxeerh7-mini-mpu6000.md) uses this same target
and HEX image, as specified by Foxeer. Its MPU6000 gyro runs at 8 kHz.

| Function | Connection / default |
| --- | --- |
| MCU | STM32H743, 400 MHz, 8 MHz HSE; revision-Y-compatible clock setup |
| Motors 1–4 | PB4, PB5, PB0, PB1 / TIM3 CH1–4 |
| IMU | SPI2 PB13/PB14/PB15, CS PB12; yaw 0° / Betaflight CW0 |
| Analog OSD | MAX7456-compatible, SPI1 PA5/PA6/PA7, CS PA4 |
| Digital OSD | MSP DisplayPort on a free UART |
| Blackbox | W25Q128-compatible NOR flash, SPI3 PC10/PC11/PC12, CS PA15 |
| Voltage | ADC3 channel 1, PC3_C, 11:1 divider; Configurator calibration |
| Current | ADC3 channel 0, PC2_C; initial scale 100, offset 0 |
| LED / beeper | PC13 / PD2, active low; push-pull beeper |
| Receiver | ELRS/CRSF on UART1 by default |
| VTX | Off by default; UART2 selected for later configuration |

| UART | TX | RX |
| --- | --- | --- |
| 1 | PA9 | PA10 |
| 2 | PA2 | PA3 |
| 3 | PB10 | PB11 |
| 4 | PA0 | PA1 |
| 6 | PC6 | PC7 |
| 7 | PE8 | PE7 |
| 8 | PE1 | PE0 |

Receiver and VTX UARTs are selectable; do not assign both to the same port.
UART5 is excluded because its TX pad PC12 carries SPI3 flash data. SBUS uses
hardware RX inversion; ELRS/CRSF does not. CRSF telemetry transmission is not
implemented. Choose only UARTs exposed by the specific board revision and
leave any ESC-telemetry wiring disconnected when repurposing that port.

Build with `cmake --preset foxeerh743-release`, then
`cmake --build --preset foxeerh743-release`, or
`powershell -File tools/build.ps1 -Board FOXEERH743`.
Outputs: `build/foxeerh743-release/FlightCode-FOXEERH743.hex` and `.bin`.
Select **Foxeer H743 / H7 Mini MPU6000** in the Configurator flasher. Settings occupy bank 2
sector 7 at `0x081E0000`; the linker and flasher retain the H7 reserved area
starting at `0x081C0000`. Blackbox uses external flash.

Check USB/DFU recovery, saved settings, detected IMU and gyro rate, all axis
signs, receiver and failsafe, four motor outputs and ESC passthrough with
propellers removed, OSD and Blackbox. Verify voltage/current measurements
against a meter and the actual ESC current sensor before using them.
FlightCode operates four motors; the additional motor/servo outputs,
barometer, GPS, RSSI ADC, LED strip and camera-control hardware are not driven.

Reference: [Betaflight FOXEERH743 target](https://github.com/betaflight/config/blob/master/configs/FOXE/FOXEERH743/config.h).
