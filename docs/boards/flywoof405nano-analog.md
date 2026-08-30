# Flywoo GOKU GN405 Nano Analog

[Back to supported boards](../../README.md#supported-flight-controllers)

The `FLYWOOF405NANO_ANALOG` target covers legacy analog GN405 Nano revisions.
It is kept separate from the HD V3 target to prevent flashing the wrong OSD
configuration.

## Hardware

| Component | FlightCode support |
| --- | --- |
| MCU | STM32F405 at 168 MHz |
| IMU | MPU6000 or ICM-42688-P, detected automatically on SPI1, CS PB12 |
| Receiver | SBUS(5) on UART5 RX PD2; external CRSF on UART4 RX PA1 |
| Motors | M1 PB0, M2 PB1, M3 PA3, M4 PA2 |
| Battery voltage | ADC PC3, 11:1 base divider with calibration |
| Analog OSD | MAX7456/AT7456E on SPI3, CS PB14 |
| Status LED | PC14 |
| Buzzer | PC13 |
| Persistent Blackbox | 16 MiB W25Q128 on SPI3, CS PB3 |
| Firmware update | USB STM32 DFU |

The OSD and W25Q128 share SPI3 in mode 0 and use independent chip-select pins.
The internal FlightCode font and 30-column PAL/NTSC layout are supported.

## OSD tuning menu

With the quad disarmed and the ARM switch off, center roll and hold **throttle
at middle + yaw left + pitch up** for 0.8 seconds. Return every stick to center
after the menu opens. See the [Analog OSD guide](../../README.md#analog-osd) for
the menu controls.

## Build

```powershell
.\tools\build.ps1 -Board FLYWOOF405NANO_ANALOG
```

Firmware image:
`build/flywoof405nano-analog-release/FlightCode-FLYWOOF405NANO_ANALOG.hex`
