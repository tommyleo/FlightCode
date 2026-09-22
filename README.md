# FlightCode - STM32 flight controllers

*Born to race.*

Current release: **1.2.0**.

## FlightCode in action! 🚀

**[Watch the flight video on YouTube](https://youtu.be/JjHND97abkM)**

FlightCode is an experimental Quad X rate-mode firmware for STM32 flight
controllers. It provides an 8 or 16 kHz main loop, configurable PID and rates,
DSHOT motor output, receiver support, Blackbox logging, diagnostics and a shared
USB Configurator.

## Supported flight controllers

- **[DIAT Mamba F411](docs/boards/mambaf411.md)**<br>
  `MAMBAF411` · STM32F411 · MPU6000 · SBUS/CRSF · Analog/Digital OSD

- **[CL Racing F4](docs/boards/clracingf4.md)**<br>
  `CLRACINGF4` · STM32F405 · MPU6000 · SBUS/CRSF · Analog/Digital OSD · microSD Blackbox

- **[Flywoo GOKU GN405 Nano HD V3](docs/boards/flywoof405nano.md)**<br>
  `FLYWOOF405NANO` · STM32F405 · ICM-42688-P · SBUS/CRSF · Digital OSD · flash Blackbox

- **[Flywoo GOKU GN405 Nano Analog](docs/boards/flywoof405nano-analog.md)**<br>
  `FLYWOOF405NANO_ANALOG` · STM32F405 · automatic IMU detection · Analog/Digital OSD · flash Blackbox

- **[HDZero Halo](docs/boards/hdzero-halo.md)**<br>
  `HDZERO_HALO` · STM32H743 · automatic IMU detection · integrated Gemini ELRS · Digital OSD · flash Blackbox

- **[SEQURE H743 V2](docs/boards/sequreh7v2.md)**<br>
  `SEQUREH7V2` · STM32H743 · ICM-42688-P · Analog/Digital OSD (AT7456E + MSP DisplayPort) · ADC voltage/current · flash Blackbox

Each board page contains its pin mapping, supported hardware, receiver and OSD
details, build command and firmware output path.

The SEQURE target compiles but still needs validation on a physical board
before flight. Receiver and VTX UARTs are selectable in the Configurator;
SBUS RX inversion is enabled on the selected UART and disabled for ELRS/CRSF.
ELRS telemetry transmission is not yet implemented.

Want to use a different controller? Read the
**[new board support request guide](docs/board-request.md)** before opening an
issue. It lists the hardware documentation and bench testing needed to produce
a safe, maintainable port.

## Core features

- Quad X rate mode with configurable PID, rates, expo, feedforward and TPA;
- selectable 8 or 16 kHz main loop;
- gyro and D-term low-pass filters plus three-axis board alignment;
- DSHOT300, DSHOT600 and DSHOT1200;
- SBUS and CRSF receiver support where provided by the board;
- configurable motor idle and normal/reversed yaw direction;
- battery telemetry and persistent voltage calibration on supported targets;
- protected motor test, guided IMU diagnostics and PID/mixer simulation;
- RAM flight logging and persistent Blackbox on equipped boards;
- USB CDC configuration and restart into STM32 DFU;
- persistent settings stored in a reserved internal flash sector.

The selected main-loop rate controls the main scheduler, motor output and timed
system tasks. PID updates remain synchronized to fresh gyroscope samples. The
ICM-42688-P follows the selected 8 or 16 kHz scheduler rate, so its PID update
rate follows it as well. MPU6000 targets retain their hardware-limited 8 kHz
gyroscope and PID rate when the main scheduler runs at 16 kHz; the latest motor
command is held for the intermediate motor-output frame.

Version 2 Blackbox metadata and version 7 Configurator JSON logs record both
the measured main-scheduler period and the interval between fresh gyroscope/PID
updates. Each sample exposes the periods in microseconds and their derived
frequencies, alongside the separated P/I/D/FF terms.

## Analog / Digital OSD

FlightCode uses a shared OSD layout for analog and digital video systems. The
Configurator provides the same drag-and-drop editor for battery voltage,
per-cell voltage, flight timer, FlightCode label, pilot name, and the compact
VTX band/channel/power value (for example `F:3:200`). The firmware selects the correct rendering backend for each
supported board.

### Analog OSD

Boards fitted with a MAX7456/AT7456E provide a 30-column PAL/NTSC OSD layout,
an in-goggle tuning menu and the original FlightCode Sans font, including
dedicated battery and clock glyphs.

To open the in-goggle tuning menu, disarm the quad, keep the ARM switch off,
center roll and hold the following stick combination for 0.8 seconds:

- **Throttle:** middle;
- **Yaw:** left;
- **Pitch:** up.

After the menu opens, return all sticks to center before navigating. Pitch
selects an item, roll changes its value, yaw right saves and exits, and yaw left
exits without saving.

### Digital OSD

All listed STM32 targets can send MSP DisplayPort on a free TX UART at
115200 baud. FlightCode sends a non-blocking HDZero-compatible 30 × 16
canvas centered in the HD display, so the same Configurator layout editor and
saved element positions work on analog and digital video.

On the Flywoo HD and HDZero Halo targets, MSP DisplayPort is enabled by
default. On boards that also have an analog OSD chip, select **HDZero V3 ·
MSP + DisplayPort** and a free TX UART in the VTX tab, save, then reboot.
Do not assign that UART to the receiver or another peripheral. The analog
chip remains available for analog video when digital OSD is not selected.

## Building on Windows

From PowerShell:

```powershell
cd C:\SvilST\FlightCode

# Build one Release target
.\tools\build.ps1 -Board MAMBAF411
.\tools\build.ps1 -Board CLRACINGF4
.\tools\build.ps1 -Board FLYWOOF405NANO
.\tools\build.ps1 -Board FLYWOOF405NANO_ANALOG
.\tools\build.ps1 -Board HDZERO_HALO

# Build every target
.\tools\build.ps1 -Board All
```

Use `-Configuration Debug` to produce a debug build. The exact output path for
each target is listed on its dedicated board page.

## Project structure

```text
src/
├── app/                 Firmware entry point and main flight loop
├── control/             Rate controller, PID logic and motor mixer
├── drivers/             IMU, motors, receiver, OSD and USB drivers
├── platform/            STM32 board mapping, HAL and interrupts
├── protocol/            FlightCode Configurator protocol
└── storage/             Settings, flight log and Blackbox storage
docs/boards/              Board-specific guides
linker/                   Board-specific linker scripts
cmake/                    ARM toolchain configuration
tools/                    Build helpers
```

## Configurator

Use the shared **[FlightCode Configurator](https://github.com/tommyleo/FlightCodeConfigurator)**
to flash supported firmware, configure PID and rates, select filters and motor
protocol, calibrate the board, run protected diagnostics, and download flight
logs or persistent Blackbox recordings.

After flashing, restart the board normally, launch the Configurator in Chrome
or Edge, press **Connect**, and select the FlightCode USB serial device.
Configuration changes and diagnostic motor output are rejected while armed.

## Safety

Always perform the first test without propellers. Verify motor order, motor
direction, gyroscope orientation, receiver failsafe and the arming command
before installing propellers.

## Direct throttle and storage formats

Throttle is applied directly to the controller and mixer on each update.
Log metadata uses version 4; exports contain a single throttle channel.
Settings saved by the preceding firmware version are not migrated; configure
and save the aircraft settings again after updating. Existing onboard logs
from the preceding format are not loaded.
