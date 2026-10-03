# NewBeeDrone Hummingbird 200 RaceSpec

Target: `HUMMINGBIRD_200RS`. **Development port: builds successfully; physical
board and flight validation are pending.**

## Implemented support

The native AT32F435 backend uses an 8 MHz crystal and 288 MHz core clock,
Artery startup/CMSIS/peripheral libraries, a microsecond timer and USB CDC.
It shares FlightCode flight control, settings and Configurator protocols.
STM32 HAL and Betaflight flight-control code are not linked into this target.

- ICM-42688-P on SPI1, using its internal clock; optional CLKIN is left undriven.
- DSHOT300/600/1200 on four timer/DMA outputs, plus 4-way ESC passthrough.
- Integrated ELRS receiver through CRSF on UART1, reserved for the receiver.
  Receiver firmware remains separate (`NewBeeDrone Diversity 2.4Ghz RX V2`).
  CRSF telemetry transmission is not implemented in FlightCode.
- MSP DisplayPort, SmartAudio or Tramp on UART5 or UART7, according to settings.
- NBD7456 analog OSD through the existing MAX7456-compatible driver. Register
  and font compatibility still require testing on the fitted chip.
- Voltage/current ADC, status LED and inverted PC13 10V VTX power enable.
- SPI NOR Blackbox: standard 2–16 MiB NOR parts, including M25P16/W25Q128.
  W25N01G NAND is **not supported**; an unrecognized flash leaves persistent
  Blackbox unavailable rather than sending NOR erase/program commands to it.
- Persistent settings in the last 2 KiB internal flash page; native flash
  operations reject writes/erases outside this page.
- Configurator USB connection and AT32 ROM DFU flashing with read-back
  verification and settings-page protection.

UART2 ESC telemetry, I2C sensors and the LED strip have pin mappings but no
FlightCode drivers. No buzzer pin exists on this board.

## Build and memory

```powershell
.\tools\build.ps1 -Board HUMMINGBIRD_200RS -Configuration Release
```

Or use CMake presets `hummingbird200rs-debug` / `hummingbird200rs-release`.
Outputs are `build/hummingbird200rs-release/FlightCode-HUMMINGBIRD_200RS.hex`
and `.bin`. The Configurator accepts the matching HEX file.

The AT32F435G layout has 1 MiB flash, with application memory ending at
`0x080FF800`. ROM DFU uses USB `2E3C:DF11`; firmware CDC uses `2E3C:5740`.
The boot request uses reserved SRAM at `0x2001FFF0`. The linker uses only
127 KiB RAM, leaving the final KiB of the first 128 KiB untouched. It does not
change the MCU user-system-data RAM partition. Confirm the exact part and
installed RAM configuration on the physical board before increasing this limit.

## Pin mapping

| Function | Connection |
| --- | --- |
| Motor 1 / 2 / 3 / 4 | PA10 / PA9 / PA1 / PA0 |
| Motor timers | TMR1 CH3 / CH2, TMR2 CH2 / CH1 |
| Motor DMA | DMA1 CH1–4, TMR1/TMR2 overflow requests |
| Status LED / LED strip | PA8 / PB1 |
| ICM-42688-P | SPI1, CS PA4, interrupt PB11, optional CLKIN PB12 |
| SPI1 SCK / MISO / MOSI | PA5 / PA6 / PA7 |
| Blackbox flash | SPI2, CS PC15 |
| SPI2 SCK / MISO / MOSI | PB10 / PB14 / PB15 |
| Analog OSD | SPI4, CS PC14 |
| SPI4 SCK / MISO / MOSI | PB13 / PB8 / PB9 |
| Integrated ELRS | UART1 RX PB7, TX PA15 (TX left input) |
| ESC sensor input | UART2 RX PB0 |
| Digital VTX / free UART | UART5 RX PB5 TX PB6 / UART7 RX PB3 TX PB4 |
| I2C2 SCL / SDA | PH2 / PH3 |
| Voltage / current ADC | PA2 / PA3 |
| VTX 10V enable | PC13, active low |

Initial gyro alignment is zero. Voltage divider is 11; current scale is 307
in Betaflight units (0.1 mV/A). Alignment, LED polarity, power switching and ADC
calibration must be verified with the board, without propellers. Also verify
USB/DFU recovery, persistent settings, receiver/failsafe, motor order and timing,
ESC passthrough, both OSD modes, and the fitted Blackbox chip before flight.

## Software verification

Debug/Release Hummingbird builds and all six existing STM32 Release targets
compile. Host tests exercise the native DSHOT timer/DMA shadow-register sequence
for all three rates, fixed packet vectors, early-frame rejection, passthrough
and interrupt-mask restoration. Flash probe tests cover NOR capacities/bank
layout and rejection of NAND/unsupported addressing.

```powershell
.\tools\test-at32.ps1 -Compiler gcc
```

Configurator tests validate the generated Hummingbird HEX, target mismatch,
vector checks and protection of the settings page. Android `assembleDebug`
also succeeds. These checks do not establish hardware operation.

## Sources and licensing

- [Manufacturer target, pinned revision](https://github.com/newbeedrone/betaflight_config/blob/e0273b9564854e0ee1fbadd8200875b30bd35650/configs/HBRD/HUMMINGBIRD_200RS/config.h)
- [Manufacturer specifications](https://newbeedrone.com/products/hummingbird-200-flight-controller-at32f435-racespec-with-diversity-elrs-2-4-receiver-build-in)
- [Requested FC + 80A AM32 ESC stack](https://www.drone-fpv-racer.com/en/hummingbird-200-racespec-20x20-f435-80a-elrs-24g-stack-by-newbeedrone-14515.html)
- [Artery AT32F435/437 SDK](https://github.com/ArteryTek/AT32F435_437_Firmware_Library/tree/4aa5607a57e368541494dff247d06e84d94e3edc)
- [Betaflight AT32 platform reference](https://github.com/betaflight/betaflight/tree/master/src/platform/AT32)

Hardware mapping is taken from NewBeeDrone's pinned configuration. Betaflight's
AT32 implementations informed peripheral multiplexing and ROM DFU integration.
Artery SDK dependency is pinned to commit
`4aa5607a57e368541494dff247d06e84d94e3edc` (2.2.5). Its CDC descriptors are adapted
locally with the original Artery copyright/license headers retained. Consult
those headers and the SDK license before redistribution.

[Hardware manifest](../../boards/HUMMINGBIRD_200RS.json) · [Main README](../../README.md)
