# CL Racing F4

[Back to supported boards](../../README.md#supported-flight-controllers)

The `CLRACINGF4` target follows the official Betaflight `CLRA/CLRACINGF4`
hardware mapping.

## Hardware

| Component | FlightCode support |
| --- | --- |
| MCU | STM32F405 at 168 MHz |
| IMU | MPU6000 on SPI1, CS PA4 |
| Receiver | SBUS on USART1 RX PA10, controllable inverter PC0 |
| Motors | M1 PB0, M2 PB1, M3 PA3, M4 PA2 |
| Battery voltage | ADC PC2, 11:1 base divider with calibration |
| Analog OSD | MAX7456/AT7456E on SPI3, CS PA15 |
| Status LED | PB5 |
| Buzzer | Passive 4 kHz output on PB4 |
| Persistent Blackbox | microSD on SPI2, CS PB12, detect PB7 |
| Firmware update | USB STM32 DFU |

The microSD is used as dedicated raw FlightCode storage rather than a FAT
volume. Do not use a card containing files that must be preserved. The onboard
SBUS inverter is isolated before entering the ROM bootloader so an active
receiver cannot prevent USB DFU detection.

## Build

```powershell
.\tools\build.ps1 -Board CLRACINGF4
```

Firmware image:
`build/clracingf4-release/FlightCode-CLRACINGF4.hex`
