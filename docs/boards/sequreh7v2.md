# SEQURE H743 V2 (`SEQUREH7V2`)

FlightCode target for the STM32H743 board. Build with
`cmake --preset sequreh7v2-release` and
`cmake --build --preset sequreh7v2-release`. The result is
`build/sequreh7v2-release/FlightCode-SEQUREH7V2.hex`.

Pin assignments follow the [Betaflight SQRE target](https://github.com/betaflight/config/blob/master/configs/SQRE/SEQUREH7V2/config.h):

| Function | Connection |
| --- | --- |
| Motors 1–4 | PB4, PB5, PB0, PB1 (TIM3 CH1–4) |
| ICM-42688-P | SPI2, CS PB12; gyro yaw alignment 90 degrees |
| AT7456E analog OSD | SPI1, CS PA4 |
| W25Q128 flash | SPI3, CS PA15 |
| Receiver CRSF | USART1 RX PA10 |
| Receiver SBUS | R1 / USART1 RX PA10, internal RX inversion enabled |
| IRC Tramp VTX control | T2 / USART2 TX PA2 |
| Battery voltage ADC | ADC1 PC3, default 11:1 divider |
| Current ADC | ADC1 PC2, Betaflight default scale 1052, offset 0 |

The OSD driver detects the chip before enabling the overlay. Analog video must
pass through the board's camera and VTX video pads. The current ADC requires a
current-sensor signal from the ESC or power distribution board; an unconnected
current pad cannot provide a meaningful reading. FlightCode reports
`BATTERY_VOLTAGE` and `BATTERY_CURRENT` over USB at 5 Hz. Use
`GET_BATTERY_CURRENT` for a one-time reading. The current scale is an initial
board default and must be checked against a known load before relying on amps
or consumed capacity.

SBUS and CRSF share the R1 receiver pad; select the matching protocol in the
Configurator. T2 carries IRC Tramp commands (channel and power), not the analog
OSD video signal. UART6 is normally used for GPS and UART7 for HD VTX MSP.

The Configurator's OSD layout editor offers **Current** as a movable item.
It displays the measured value rounded to whole amperes, for example `45 A`.
The item is off by default and its enable state and position are saved with
the other flight settings. Firmware settings from version 24 are migrated
without changing the existing OSD layout.

This target has compiled successfully, but has not yet been checked on a
physical SEQURE H743 V2. Before flight, verify gyro direction, all four motor
outputs and numbering, receiver, analog OSD, voltage against a multimeter, and
current against a known load with propellers removed.
