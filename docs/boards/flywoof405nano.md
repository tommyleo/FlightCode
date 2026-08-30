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
| Digital OSD | MSP DisplayPort at 115200 baud on UART6 TX PC6 |
| Status LED | PC14 |
| Buzzer | PC13 |
| Persistent Blackbox | 16 MiB W25Q128 on SPI3, CS PB3 |
| Firmware update | USB STM32 DFU |

The dedicated SBUS(5) input uses the board's fixed hardware inverter. FlightCode
uses UART6 for the digital VTX connection, matching Flywoo's HD wiring. MSP
DisplayPort and the OSD overlay are enabled by default on a fresh configuration.

The FC transmits standard MSP v1 DisplayPort frames at 115200 baud. HDZero's
centered 30 × 16 compatibility canvas is used, preserving the layout edited in
FlightCode Configurator. Battery voltage, per-cell voltage, flight timer,
FlightCode label, pilot name, VTX channel, and VTX power are supported. Output
is queued and transmitted without blocking the flight-control loop.

Connect UART6 TX to the digital VTX RX input and share ground. Existing saved
settings are not overwritten; in that case select **HDZero V3 · MSP +
DisplayPort** and **UART6** in the Configurator VTX tab, save, and reboot.

## Build

```powershell
.\tools\build.ps1 -Board FLYWOOF405NANO
```

Firmware image:
`build/flywoof405nano-release/FlightCode-FLYWOOF405NANO.hex`
