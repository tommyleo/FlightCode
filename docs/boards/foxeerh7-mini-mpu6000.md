# Foxeer H7 Mini MPU6000 (`FOXEERH743`)

[Back to supported boards](../../README.md#supported-flight-controllers)

Foxeer specifies **FOXEERH743** as the Betaflight target for its H7 Mini
MPU6000 (MR1709). FlightCode uses the same shared target and firmware image as
the full-size Foxeer H743: `FlightCode-FOXEERH743.hex`.

Select **Foxeer H743 / H7 Mini MPU6000** in the Configurator Firmware tab.
The connected board identifies itself as `FOXEERH743`; the MPU6000 is detected
automatically on SPI2 and its gyro runs at 8 kHz.

| Hardware | Mini specification |
| --- | --- |
| MCU / IMU | STM32H743 / MPU6000 |
| Mounting / board size | 20 × 20 mm / 30 × 31 mm |
| Analog OSD | MAX7456-compatible |
| Blackbox | 16 MB SPI flash |
| Serial pads | Six UART sets plus RX8 for ESC telemetry |
| USB | Type-C |
| Barometer | DPS310, not used by FlightCode |

See the [shared H743 board guide](foxeerh743.md) for motor pins, sensor
alignment, OSD/flash buses, ADC inputs, build commands and settings storage.
Use the actual Mini board labels for wiring. RX8 is receive-only on the
published Mini specification; use an exposed TX/RX UART pair for HDZero
MSP + DisplayPort (a TX pad alone suffices for SmartAudio / Tramp). FlightCode does not implement ESC or CRSF telemetry.

The image has been compiled and software-tested, but the Mini has not been
validated on physical hardware. Check USB/DFU, saved settings, IMU identity,
axis signs, receiver failsafe, motor order/direction with propellers removed,
OSD, battery measurements and Blackbox before flight.

References: [Foxeer H7 Mini product specification](https://www.foxeer.com/foxeer-h7-mini-mpu6000-fc-8s-dual-bec-barometer-g-504)
and [Betaflight FOXEERH743 pin map](https://github.com/betaflight/config/blob/master/configs/FOXE/FOXEERH743/config.h).
