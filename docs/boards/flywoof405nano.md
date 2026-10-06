# Flywoo GOKU GN405 Nano HD V3

[Back to supported boards](../../README.md#supported-flight-controllers)

The `FLYWOOF405NANO` target is intended for the current HD/ELRS V3 hardware.
Do not use the Analog target firmware on this board.

## Hardware

| Component | FlightCode support |
| --- | --- |
| MCU | STM32F405 at 168 MHz |
| IMU | ICM-42688-P on SPI1, CS PB12 |
| Receiver | Integrated ELRS on UART6 RX PC7 / TX PC6; SBUS(5) on UART5 RX PD2 |
| Motors | M1 PB0, M2 PB1, M3 PA3, M4 PA2 |
| Battery voltage | ADC PC3, 11:1 base divider with calibration |
| Digital OSD | MSP DisplayPort at 115200 baud on UART4 PA0 TX / PA1 RX |
| Status LED | PC14 |
| Buzzer | PC13 |
| Persistent Blackbox | 16 MiB W25Q128 on SPI3, CS PB3 |
| Firmware update | USB STM32 DFU |

The dedicated SBUS(5) input uses the board's fixed hardware inverter. FlightCode
reserves UART6 for integrated ELRS and uses UART4 for an external digital VTX connection. MSP
DisplayPort and the OSD overlay are enabled by default on a fresh configuration.
UART5 is reserved for inverted SBUS and cannot be assigned to MSP DisplayPort
or VTX control or used for ELRS; the Configurator and firmware reject those
assignments. Use UART6 for the integrated ELRS receiver. The Analog target retains external ELRS on UART4.

The FC transmits standard MSP v1 DisplayPort frames at 115200 baud. HDZero's
centered 30 × 16 compatibility canvas is used, preserving the layout edited in
FlightCode Configurator. Battery voltage, per-cell voltage, flight timer,
FlightCode label, pilot name, and compact VTX band/channel/power are supported. Output
is queued and transmitted without blocking the flight-control loop.

Connect UART4 TX to the digital VTX RX input and share ground. Existing saved
settings for VTX on UART6 are migrated to UART4, preserving the active OSD protocol and tuning. Reconnect the VTX to UART4, then select **HDZero V3 · MSP +
DisplayPort** and **UART4** in the Configurator VTX tab, save, and reboot.

## Build

```powershell
.\tools\build.ps1 -Board FLYWOOF405NANO
```

Firmware image:
`build/flywoof405nano-release/FlightCode-FLYWOOF405NANO.hex`
