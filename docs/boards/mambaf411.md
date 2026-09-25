# DIAT Mamba F411

[Back to supported boards](../../README.md#supported-flight-controllers)

The `MAMBAF411` target uses the DIAT Mamba F411 hardware mapping.

## Hardware

| Component | FlightCode support |
| --- | --- |
| MCU | STM32F411 at 96 MHz |
| IMU | MPU6000 on SPI1, CS PA4 |
| Receiver | SBUS or ELRS/CRSF on USART1 RX PA10, controllable SBUS inverter PB10 |
| Motors | M1 PB3, M2 PB4, M3 PB6, M4 PB7 |
| Battery voltage | ADC PA0, 16.2:1 base divider |
| Analog OSD | MAX7456/AT7456E on SPI2, CS PB12 |
| Digital OSD | MSP DisplayPort on a free TX UART (typically UART2 / PA2) |
| Status LED | PC13 |
| Buzzer | Active-low PB2 |
| Persistent Blackbox | Not available; RAM flight log remains supported |
| Firmware update | USB STM32 DFU |

The status LED flashes rapidly during gyroscope calibration, once per second
as a heartbeat, and twice per second when valid receiver data is present.

## OSD tuning menu

For digital video, select **HDZero V3 · MSP + DisplayPort** and UART2 in the
Configurator VTX tab, then save and reboot. UART1 is used by the receiver.
Its controllable inverter is enabled only for SBUS; the receiver/VTX UART
conflict is rejected by the Configurator and firmware.

With the quad disarmed and the ARM switch off, center roll and hold **throttle
at middle + yaw left + pitch up** for 0.8 seconds. Return every stick to center
after the menu opens. See the [Analog / Digital OSD guide](../../README.md#analog--digital-osd) for
the menu controls.

## Build

```powershell
.\tools\build.ps1 -Board MAMBAF411
```

Firmware image:
`build/release/FlightCode-MAMBAF411.hex`
