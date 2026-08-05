# FlightCode - MAMBAF411 and CLRacingF4

Experimental Quad X rate-mode firmware for the DIAT MAMBAF411 (STM32F411) and
CL Racing CLRacingF4 (STM32F405).

The gyroscope/PID control loop runs at 8 kHz, matching the highest useful data
rate of the MPU6000. DSHOT300 is the default ESC protocol; OneShot125 and
MultiShot can also be selected from the Configurator.

- STM32F411 MCU, 96 MHz core, and USB CDC
- MPU6000 on SPI1: CS PA4, SCK PA5, MISO PA6, MOSI PA7, CW180 orientation
- SBUS on USART1 RX PA10, inverter PB10, 100000 baud 8E2
- Motors: M1 PB3, M2 PB4, M3 PB6, M4 PB7
- LED on PC13
- Active-low buzzer on PB2, controlled by CH5 > 2000

The mapping is based on the official Betaflight `DIAT/MAMBAF411` target.
Radio channels are CH1 throttle, CH2 roll, CH3 pitch, CH4 yaw, and
CH6 arm > 2000.

The CLRacingF4 uses the Betaflight `CLRA/CLRACINGF4` mapping: MPU6000 on SPI1
(CS PA4), SBUS on USART1 RX PA10 with inverter PC0, motors on PB0, PB1, PA3,
and PA2, LED on PB5, and an active-low buzzer on PB4.

## OSD tuning menu

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
# One board (Debug)
.\tools\build.ps1 -Board MAMBAF411
.\tools\build.ps1 -Board CLRACINGF4

# All boards (Debug)
.\tools\build.ps1 -Board All

# One board or all boards in Release mode
.\tools\build.ps1 -Board CLRACINGF4 -Configuration Release
.\tools\build.ps1 -Board All -Configuration Release
```

Firmware images:

- `build/debug/FlightCode-MAMBAF411.hex`
- `build/clracingf4-debug/FlightCode-CLRACINGF4.hex`

Release builds are stored in `build/release` and
`build/clracingf4-release`, respectively.

The project can be opened directly in Visual Studio Code with the CMake Tools,
C/C++, and Cortex-Debug extensions.

## Project structure

```text
src/
├── app/                 Firmware entry point and main flight loop
├── control/             Rate controller, PID logic and motor mixer
├── drivers/
│   ├── imu/             MPU6000 driver
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

## CLRacingF4 microSD Blackbox

The CLRacingF4 target supports the onboard microSD socket on SPI2
(PB13/PB14/PB15, CS PB12, detect PB7). Long-flight records are packed into
independent 512-byte sectors and transferred through DMA from a RAM queue, so
SD busy time never blocks the 8 kHz control loop. Recording is optional in the
Configurator and only flights that exceed 10% throttle are retained.

The current Blackbox format treats the card as dedicated FlightCode storage;
it is not a FAT volume and must not contain files that need to be preserved.
Log listing and download will use the FlightCode USB protocol rather than
direct PC filesystem access.

## USB Configurator

The firmware exposes a USB CDC serial port with telemetry, SBUS channels,
motor outputs, and PID configuration. After flashing the firmware:

1. restart the board normally without holding BOOT;
2. launch `C:\SvilST\FlightCodeConfigurator\start-configurator.cmd`;
3. in Chrome or Edge, select **Connect** and choose FlightCode.

PID settings can only be changed or saved while the quad is disarmed. The last
sector of internal flash is reserved for persistent settings.

The configurator also exposes persistent gyroscope and D-term low-pass
cutoffs. Factory defaults are 100 Hz for the gyroscope and 60 Hz for D-term;
accepted ranges are 50–250 Hz and 20–200 Hz respectively, with the D-term
cutoff constrained not to exceed the gyroscope cutoff.

## Safety

Always perform the first test without propellers. Verify motor order, motor
direction, gyroscope orientation, failsafe behavior, and the arming command.
