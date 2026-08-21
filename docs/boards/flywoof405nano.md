# Flywoo GOKU GN405 Nano HD V3

[Back to supported boards](../../README.md#supported-flight-controllers)

The `FLYWOOF405NANO` target is intended for the current HD/ELRS V3 hardware.
Do not use the Analog target firmware on this board.

## Hardware

| Component | FlightCode support |
| --- | --- |
| MCU | STM32F405 at 168 MHz |
| IMU | ICM-42688-P on SPI1, CS PB12 |
| Receiver | SBUS(5) on UART5 RX PD2; external CRSF on UART4 RX PA1 |
| Motors | M1 PB0, M2 PB1, M3 PA3, M4 PA2 |
| Battery voltage | ADC PC3, 11:1 base divider with calibration |
| Analog OSD | Not fitted on the HD V3 |
| Status LED | PC14 |
| Buzzer | PC13 |
| Persistent Blackbox | 16 MiB W25Q128 on SPI3, CS PB3 |
| Firmware update | USB STM32 DFU |

The dedicated SBUS(5) input uses the board's fixed hardware inverter. The
built-in ELRS receiver connected to UART6 is not used by FlightCode. Digital
video OSD requires MSP DisplayPort, which is not implemented yet.

## Build

```powershell
.\tools\build.ps1 -Board FLYWOOF405NANO
```

Firmware image:
`build/flywoof405nano-release/FlightCode-FLYWOOF405NANO.hex`
