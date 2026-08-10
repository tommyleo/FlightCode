# FlightCode - STM32 flight controllers

Experimental Quad X rate-mode firmware for the DIAT MAMBAF411 (STM32F411),
CL Racing CLRacingF4 (STM32F405), and Flywoo GOKU GN405 Nano HD V3
(`FLYWOOF405NANO`, STM32F405).

The gyroscope/PID control loop runs at 8 kHz, matching the highest useful data
rate of the supported MPU6000 and ICM-42688-P sensors. DSHOT300 is the default
ESC protocol; OneShot125 and MultiShot can also be selected from the
Configurator.

## Supported boards and enabled hardware

| Feature | MAMBAF411 | CLRACINGF4 | FLYWOOF405NANO |
| --- | --- | --- | --- |
| Flight controller | DIAT Mamba F411 | CL Racing F4 | Flywoo GOKU GN405 Nano HD V3 |
| MCU / clock | STM32F411 / 96 MHz | STM32F405 / 168 MHz | STM32F405 / 168 MHz |
| IMU | MPU6000, SPI1 | MPU6000, SPI1 | ICM-42688-P, SPI1 |
| Receiver | SBUS, USART1 with controllable inverter | SBUS, USART1 with controllable inverter | SBUS(5), UART5 with fixed inverter |
| Motor outputs | PB3, PB4, PB6, PB7 | PB0, PB1, PA3, PA2 | PB0, PB1, PA3, PA2 |
| ESC protocols | DSHOT300, OneShot125, MultiShot | DSHOT300, OneShot125, MultiShot | DSHOT300, OneShot125, MultiShot |
| Battery voltage | ADC PA0 | ADC PC2 | ADC PC3 |
| Analog OSD | MAX7456/AT7456E, SPI2 | MAX7456/AT7456E, SPI3 | Not fitted on HD V3 |
| Persistent Blackbox | No external storage | microSD on SPI2 | 16 MiB W25Q128 internal flash on SPI3 |
| RAM flight log | 2,944 records | 2,688 records | 2,688 records |
| Status LED / buzzer | PC13 / PB2 | PB5 / PB4 | PC14 / PC13 |
| USB configuration | USB CDC | USB CDC | USB CDC |
| Firmware update | STM32 DFU | STM32 DFU | STM32 DFU |

## Firmware features

All three targets enable rate mode, Quad X mixing, configurable PID/rates/expo,
progressive feedforward, TPA, gyro and D-term filters, board alignment, motor
direction and idle, receiver mapping, battery telemetry, protected motor test,
guided IMU and PID/mixer diagnostics, RAM flight logging, failsafe, buzzer and
USB DFU requests. Persistent settings are stored in a dedicated internal flash
sector. Configuration changes and diagnostic motor output are rejected while
armed.

The Balanced, Racing and Freestyle quick profiles share the proven base P/I
values and vary rates, expo, feedforward, D-term and filtering. Factory Racing
defaults are 420/420/350 deg/s, expo 0.30, progressive FF 0.025/0.025/0.015,
TPA 20% from 70% throttle, and gyro/D-term filters at 90/50 Hz.

- STM32F411 MCU, 96 MHz core, and USB CDC
- MPU6000 on SPI1: CS PA4, SCK PA5, MISO PA6, MOSI PA7; no fixed yaw alignment
- SBUS on USART1 RX PA10, inverter PB10, 100000 baud 8E2
- Motors: M1 PB3, M2 PB4, M3 PB6, M4 PB7
- LED on PC13: fast 100 ms flashes during gyroscope calibration, then one
  heartbeat flash per second, or two flashes with valid SBUS
- Active-low buzzer on PB2, controlled by CH5 > 2000 with two beeps every 500 ms

The mapping is based on the official Betaflight `DIAT/MAMBAF411` target.
Radio channels are CH1 throttle, CH2 roll, CH3 pitch, CH4 yaw, and
CH6 arm > 2000.

The CLRacingF4 uses the Betaflight `CLRA/CLRACINGF4` mapping: MPU6000 on SPI1
(CS PA4), SBUS on USART1 RX PA10 with inverter PC0, motors on PB0, PB1, PA3,
and PA2, LED on PB5, and a passive 4 kHz buzzer on PB4. The LED uses the same
gyroscope-calibration, heartbeat, and receiver-signal patterns as MAMBAF411;
the buzzer uses the same double-beep pattern.

### Flywoo GOKU GN405 Nano HD V3

The `FLYWOOF405NANO` target follows the official Betaflight mapping for the
current ELRS V3 hardware:

- STM32F405 at 168 MHz and ICM-42688-P on SPI1, CS PB12;
- external SBUS receiver on the pad marked **SBUS(5)**, which is UART5 RX on
  PD2 through the board's fixed hardware inverter;
- receiver power from an adjacent **+5V** and **GND** pad;
- motors M1 PB0, M2 PB1, M3 PA3, M4 PA2;
- status LED PC14, buzzer control PC13, and battery ADC PC3;
- built-in ELRS remains unused by FlightCode on UART6.

The HD V3 has no MAX7456/AT7456E and no analog `CAM`/`VTX` video path, so
analog OSD is intentionally reported as unavailable. Digital video hardware
can still provide its own OSD when driven by a separately implemented MSP
DisplayPort link; that link is not part of this target yet. Betaflight's
`FLYWOOF405NANO` definition also covers older revisions and still lists a
MAX7456, so the V3 hardware specification takes precedence for this target.

## OSD tuning menu

This menu is available only on targets with an onboard analog OSD chip.
With the quad disarmed and the ARM switch off, hold throttle high and roll
right for 0.8 seconds to open the OSD tuning menu.  Center all four sticks
before navigating.  Pitch selects a parameter, roll changes its value, yaw
left exits without applying changes, and yaw right applies and saves all
changes to flash.  Motor output remains inhibited while the menu is open.

The menu includes roll, pitch, and yaw P/I/D/feedforward values, maximum rates,
expo, TPA attenuation, and the TPA breakpoint.

## Building on Windows

From PowerShell:

```powershell
cd C:\SvilST\FlightCode
# One board (Release, the default)
.\tools\build.ps1 -Board MAMBAF411
.\tools\build.ps1 -Board CLRACINGF4
.\tools\build.ps1 -Board FLYWOOF405NANO

# All boards (Release)
.\tools\build.ps1 -Board All

# Debug must be requested explicitly
.\tools\build.ps1 -Board CLRACINGF4 -Configuration Debug
```

Default Release firmware images:

- `build/release/FlightCode-MAMBAF411.hex`
- `build/clracingf4-release/FlightCode-CLRACINGF4.hex`
- `build/flywoof405nano-release/FlightCode-FLYWOOF405NANO.hex`

Debug builds are stored in the matching `build/*-debug` directory.

The project can be opened directly in Visual Studio Code with the CMake Tools,
C/C++, and Cortex-Debug extensions.

## Project structure

```text
src/
├── app/                 Firmware entry point and main flight loop
├── control/             Rate controller, PID logic and motor mixer
├── drivers/
│   ├── imu/             MPU6000 and ICM-42688-P drivers
│   ├── motors/          DSHOT, OneShot125 and MultiShot outputs
│   ├── receiver/        SBUS receiver and frame decoder
│   └── usb/             USB device and CDC transport
├── platform/            STM32 board mapping, HAL configuration and interrupts
├── protocol/            Shared FlightCode Configurator protocol
└── storage/             Persistent settings and flight log
linker/                  Board-specific linker scripts
cmake/                   ARM toolchain configuration
tools/                   Build helpers
```

## Persistent Blackbox

The CLRacingF4 target supports the onboard microSD socket on SPI2
(PB13/PB14/PB15, CS PB12, detect PB7). Long-flight records are packed into
independent 512-byte sectors and transferred through DMA from a RAM queue, so
SD busy time never blocks the 8 kHz control loop. Recording is optional in the
Configurator. A new log is retained as soon as throttle rises above the 1%
idle-noise threshold; arming and disarming without touching throttle preserves
the previous log.

The Blackbox format treats the card as dedicated FlightCode storage; it is not
a FAT volume and must not contain files that need to be preserved. A persistent
on-card catalog retains the 20 most recent flight descriptors across power
cycles. The Configurator can list them, download complete JSON logs through the
FlightCode USB protocol, or clear the catalog without formatting the card.
Each sample includes the separated P, I, D and feedforward contributions for
all three axes. Older blocks remain readable and report a zero feedforward term.

Feedforward uses a smooth progressive curve: 70% of the configured gain around
stick center, increasing continuously to 100% at the configured maximum rate.
The Blackbox `ffTerm` field records the resulting contribution after this curve.

The Flywoo target uses its onboard 16 MiB W25Q128 NOR flash. It keeps the most
recent completed flight and prepares the alternate 8 MiB bank in the
background. Recording starts only after throttle rises above 1%, so an
arm/disarm cycle on the bench does not replace the previous flight. The
Configurator provides physical write/read verification, a synthetic session
test, detailed flash error diagnostics, catalog listing, JSON download and
erase controls.

MAMBAF411 has no persistent Blackbox storage in the current target, but its RAM
flight log remains available through the Configurator.

## USB Configurator

The firmware exposes a USB CDC serial port with telemetry, SBUS channels,
motor outputs, and PID configuration. After flashing the firmware:

1. restart the board normally without holding BOOT;
2. launch `C:\SvilST\FlightCodeConfigurator\start-configurator.cmd`;
3. in Chrome or Edge, select **Connect** and choose FlightCode.

PID settings can only be changed or saved while the quad is disarmed. The last
sector of internal flash is reserved for persistent settings.

The configurator also exposes persistent gyroscope and D-term low-pass
cutoffs. Factory defaults are 90 Hz for the gyroscope and 50 Hz for D-term;
accepted ranges are 50–250 Hz and 20–200 Hz respectively, with the D-term
cutoff constrained not to exceed the gyroscope cutoff.

## Safety

Always perform the first test without propellers. Verify motor order, motor
direction, gyroscope orientation, failsafe behavior, and the arming command.
