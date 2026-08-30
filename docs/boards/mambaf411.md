# DIAT Mamba F411

[Back to supported boards](../../README.md#supported-flight-controllers)

The `MAMBAF411` target follows the official Betaflight `DIAT/MAMBAF411`
hardware mapping.

## Hardware

| Component | FlightCode support |
| --- | --- |
| MCU | STM32F411 at 96 MHz |
| IMU | MPU6000 on SPI1, CS PA4 |
| Receiver | SBUS on USART1 RX PA10, controllable inverter PB10 |
| Motors | M1 PB3, M2 PB4, M3 PB6, M4 PB7 |
| Battery voltage | ADC PA0, 16.2:1 base divider |
| Analog OSD | MAX7456/AT7456E on SPI2, CS PB12 |
| Status LED | PC13 |
| Buzzer | Active-low PB2 |
| Persistent Blackbox | Not available; RAM flight log remains supported |
| Firmware update | USB STM32 DFU |

The status LED flashes rapidly during gyroscope calibration, once per second
as a heartbeat, and twice per second when valid receiver data is present.

## OSD tuning menu

With the quad disarmed and the ARM switch off, center roll and hold **throttle
at middle + yaw left + pitch up** for 0.8 seconds. Return every stick to center
after the menu opens. See the [Analog OSD guide](../../README.md#analog-osd) for
the menu controls.

## Build

```powershell
.\tools\build.ps1 -Board MAMBAF411
```

Firmware image:
`build/release/FlightCode-MAMBAF411.hex`
