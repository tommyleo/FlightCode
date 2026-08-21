# HDZero Halo

[Back to supported boards](../../README.md#supported-flight-controllers)

The `HDZERO_HALO` target follows the official Betaflight `HDZO/HDZERO_HALO`
hardware mapping and supports the Halo BLHeli_32 stack.

## Hardware

| Component | FlightCode support |
| --- | --- |
| MCU | STM32H743 at 480 MHz |
| IMU | MPU6000 or ICM-42688-P, detected automatically on SPI1 |
| Receiver | Integrated Gemini ExpressLRS using CRSF on USART1 RX PB7 |
| Motors | M1 PC6, M2 PC7, M3 PC8, M4 PC9 |
| Battery voltage | ADC PC0, 11:1 base divider with calibration |
| Analog OSD | Not fitted; HDZero uses digital video |
| Status LED | PE2 |
| Buzzer | Active-low PD12 |
| Persistent Blackbox | 16 MiB W25Q128 on SPI2, CS PB12 |
| Firmware update | USB STM32 DFU |

The 4-in-1 ESC is driven through the supplied eight-pin stack cable using
DSHOT300, DSHOT600 or DSHOT1200. ESC telemetry, MSP DisplayPort OSD, VTX
control, switchable 9 V BEC control and receiver telemetry are not implemented
yet.

## Build

```powershell
.\tools\build.ps1 -Board HDZERO_HALO
```

Firmware image:
`build/hdzero-halo-release/FlightCode-HDZERO_HALO.hex`
