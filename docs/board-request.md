# Request support for a new board

FlightCode can be extended to additional STM32 flight controllers when enough
hardware documentation and test support are available. Before opening a
request, check the existing [supported boards](../README.md#supported-flight-controllers)
and search the repository issues to avoid duplicates.

## What to provide

Please include as much of the following information as possible:

- manufacturer, full product name, hardware revision, and a product link;
- clear photographs of both sides of the board;
- MCU model and flash/RAM capacity;
- IMU model and bus connection (SPI or I2C), including chip-select and
  interrupt pins;
- official schematic, pinout, or a reliable Betaflight target/configuration;
- USB, status LED, buzzer, battery ADC, current sensor, and boot-button pins;
- receiver protocols and the UARTs normally used for SBUS, CRSF, or integrated
  ELRS;
- motor outputs, timer channels, and DMA assignments when known;
- analog OSD chip and SPI pins, or digital OSD/DisplayPort connection;
- VTX-control UART and supported SmartAudio, IRC Tramp, or MSP connection;
- Blackbox storage type, part number, capacity, and pins (microSD or SPI flash);
- bootloader type and expected firmware format (`.hex`, `.bin`, or `.dfu`).

Links to manufacturer documentation are preferred. Do not upload copyrighted
or confidential files that you are not allowed to redistribute.

## Test hardware is required

A board port cannot be completed safely without access to real hardware. In
your request, state whether you can:

- build and flash development firmware;
- connect through USB and copy diagnostic output;
- test the IMU, receiver, OSD, motors, battery sensing, and Blackbox;
- provide short Blackbox logs and precise fault descriptions;
- perform the first tests with the propellers removed.

Motor tests must always be performed without propellers. Flight testing should
only begin after motor order and direction, gyro orientation, receiver
failsafe, arming behavior, and battery readings have been verified on the
bench.

## Request template

Open a [new FlightCode issue](https://github.com/tommyleo/FlightCode/issues/new)
with the title `[BOARD] Manufacturer Model revision` and copy the following
template into the description:

```markdown
## Board

- Manufacturer:
- Model:
- Hardware revision:
- Product page:
- MCU:
- IMU:
- Closest Betaflight target, if known:

## Hardware documentation

- Schematic or pinout link:
- Front and rear photographs:
- Other useful documentation:

## Required features

- Receiver and UART:
- Motor outputs / ESC protocol:
- OSD:
- VTX control:
- Blackbox storage:
- Battery/current sensing:
- Other peripherals:

## Testing

- I own the board: Yes / No
- I can build and flash test firmware: Yes / No
- I can provide USB diagnostics and Blackbox logs: Yes / No
- I agree to perform initial motor tests without propellers: Yes / No

## Additional notes

Describe any unusual wiring, integrated receiver/VTX behavior, known hardware
issues, or features that should be prioritized.
```

Submitting a request does not guarantee that the board will be supported. The
time required depends on documentation quality, hardware availability, driver
compatibility, and the contributor's ability to test development builds.

