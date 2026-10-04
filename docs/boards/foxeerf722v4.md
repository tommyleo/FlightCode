# Foxeer F722 V4 (`FOXEERF722V4`)

[Back to supported boards](../../README.md#supported-flight-controllers)

One firmware image supports the MPU6000 and ICM-42688-P versions through
automatic IMU detection. This is a new STM32F7 port: compilation and software
tests do not replace validation on both physical board variants.

| Function | Connection / default |
| --- | --- |
| MCU | STM32F722RE, 216 MHz, 8 MHz HSE |
| Motors 1–4 | PA9 / TIM1 CH2, PA8 / TIM1 CH1, PC9 / TIM8 CH4, PC8 / TIM8 CH3 |
| IMU | SPI1 PA5/PA6/PA7, CS PB2; MPU6000 or ICM-42688-P |
| Sensor alignment | Yaw −90° (Betaflight CW270) |
| Analog OSD | MAX7456-compatible, SPI3 PC10/PC11/PB5, CS PC3 |
| Digital OSD | MSP DisplayPort on a free UART |
| Blackbox | W25Q128-compatible NOR flash, SPI2 PB13/PB14/PB15, CS PB12 |
| Voltage | ADC3 channel 10, PC0, 11:1 divider; Configurator calibration |
| Current | ADC3 channel 12, PC2; initial scale 400, offset 0 |
| LED / beeper | PC15 / PA4, active low; push-pull beeper |
| Receiver | ELRS/CRSF on UART1 by default; selectable UART1–6 |
| VTX | Off by default; UART2 selected for later configuration |

| UART | TX | RX |
| --- | --- | --- |
| 1 | PB6 | PB7 |
| 2 | PA2 | PA3 |
| 3 | PB10 | PB11 |
| 4 | PA0 | PA1 |
| 5 | PC12 | PD2 |
| 6 | PC6 | PC7 |

SBUS uses the MCU's RX inversion; CRSF leaves inversion disabled. Receiver and
VTX must use different UARTs. CRSF telemetry transmission is not implemented.
MPU6000 runs at 8 kHz; ICM-42688-P supports 8 or 16 kHz. DSHOT300/600/1200 uses
two timer DMA bursts, preserving the motor order above. DMA buffers reside in
SRAM, outside DTCM, and data cache remains disabled for coherence.

Build with `cmake --preset foxeerf722v4-release`, then
`cmake --build --preset foxeerf722v4-release`, or
`powershell -File tools/build.ps1 -Board FOXEERF722V4`.
Outputs: `build/foxeerf722v4-release/FlightCode-FOXEERF722V4.hex` and `.bin`.
The Configurator's **Foxeer F722 V4 (ICM / MPU6000)** selection accepts this
same image for either sensor variant. The application occupies at most
384 KiB; sector 7 at `0x08060000` stores settings. Blackbox uses external flash.

Before flight, check USB reconnection and DFU recovery, settings save/reload,
detected IMU and gyro rate, roll/pitch/yaw signs with the model, receiver and
failsafe, individual motor order/direction with propellers removed, OSD, and
Blackbox recording/download. Calibrate voltage against a meter and check the
current scale against the actual ESC current-sensor signal.
Barometer, GPS, RSSI ADC, LED strip and camera-control hardware are not driven.

Pin/IMU/timer references: [current Betaflight target](https://github.com/betaflight/config/blob/master/configs/FOXE/FOXEERF722V4/config.h)
and [legacy unified target](https://github.com/betaflight/unified-targets/blob/master/configs/default/FOXE-FOXEERF722V4.config).
