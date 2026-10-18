# SEQURE H743 V2 (`SEQUREH7V2`)

FlightCode target for the STM32H743 board. Build with
`cmake --preset sequreh7v2-release` and
`cmake --build --preset sequreh7v2-release`. The result is
`build/sequreh7v2-release/FlightCode-SEQUREH7V2.hex`.

Pin assignments for this target:

| Function | Connection |
| --- | --- |
| Motors 1–4 | PB4, PB5, PB0, PB1 (TIM3 CH1–4) |
| ICM-42688-P | SPI2, CS PB12; gyro yaw alignment 90 degrees |
| AT7456E analog OSD | SPI1, CS PA4 |
| Digital OSD | MSP DisplayPort on a free UART |
| W25Q128 flash | SPI3, CS PA15 |
| Receiver SBUS or ELRS/CRSF | Selectable RX UART; default UART1 / PA10 |
| IRC Tramp VTX control | Selectable TX UART; default UART2 / PA2 |
| Battery voltage ADC | ADC3 channel 1, PC3_C, default 11:1 divider |
| Current ADC | ADC3 channel 0, PC2_C, default scale 1052, offset 0 |

The OSD driver detects the chip before enabling the overlay. Analog video must
pass through the board's camera and VTX video pads. The current ADC requires a
current-sensor signal from the ESC or power distribution board; an unconnected
current pad cannot provide a meaningful reading. FlightCode reports
`BATTERY_VOLTAGE` and `BATTERY_CURRENT` over USB at 5 Hz. Use
`GET_BATTERY_CURRENT` for a one-time reading. The current scale is an initial
board default and must be checked against a known load before relying on amps
or consumed capacity.

For digital video, select **HDZero V3 · MSP + DisplayPort** and a free UART
in the Configurator VTX tab, then save and reboot. UART8 is reserved for the
internal ESC telemetry connection and cannot carry MSP DisplayPort.

The board exposes five UARTs on user-accessible pads: UART1 (PA9/PA10),
UART2 (PA2/PA3), UART4 (PA0/PA1), UART6 (PC6/PC7), and UART7 (PE8/PE7).
Choose the receiver and VTX ports in the Configurator without assigning both
functions to the same UART. SBUS enables the STM32H743 RX inversion on the
selected port; ELRS/CRSF leaves it disabled. UART8 RX (PE0) is connected
internally for ESC telemetry and is therefore not offered as a configurable
receiver or VTX port. IRC Tramp carries channel/power commands, not the analog
OSD video signal. UART6 and UART7 are often used for GPS and HD VTX MSP
respectively, but those are wiring conventions rather than fixed requirements
in this target.

The Configurator's OSD layout editor offers **Current** as a movable item.
It displays one decimal in six cells, up to `999.9A`.
The item is off by default and its enable state and position are saved with
the other flight settings. Firmware settings from version 24 are migrated
without changing the existing OSD layout.

This target has compiled successfully, but has not yet been checked on a
physical SEQURE H743 V2. Before flight, verify gyro direction, all four motor
outputs and numbering, receiver, analog OSD, voltage against a multimeter, and
current against a known load with propellers removed.
Analog OSD video format can be selected in Camera OSD: Auto, PAL or NTSC.
Apply changes the format immediately; Save persists the choice for power-on,
independently of camera startup timing. Auto retains startup detection.
PAL uses 30 x 16 cells; NTSC uses 30 x 13 visible cells. Existing items outside
the NTSC grid are marked in the editor and must be moved into the visible area.
Settings version 27 is migrated with the existing layout and other settings
preserved; save once after updating to persist the new version.
Host regression: gcc -std=c11 -I tests/osd_stubs tests/osd_video_mode_test.c -o osd_video_mode_test

## Current display troubleshooting

The Configurator header next to battery voltage shows `BATTERY_CURRENT` with two decimals. Analog
and MSP DisplayPort OSD show one decimal (for example `1.2A`), avoiding the
previous whole-amp rounding which displayed any 0.5–1.49 A reading as `1A`.
The value is instantaneous current, not consumed capacity in mAh. The default
1052 scale matches Betaflight's SEQUREH7V2 target; it still needs calibration for
the connected ESC sensor. Arming without propellers alone does not establish
whether a whole-amp reading is stuck. Compare the precise reading before/after
motor operation. If it remains unchanged, verify the ESC current signal reaching
PC2 and compare against an independent measurement before changing the scale.

Diagnostic firmware additionally reports `BATTERY_CURRENT_ADC raw samples age_ms
errors` once per second and on `GET_BATTERY_CURRENT_ADC`. The Configurator's
current tooltip displays these readings. Advancing sample count confirms fresh
conversions, but does not establish that the configured ADC input reaches the
physical sensor pin; a stopped count/old age indicates a sampling failure. ADC conversions
now have a bounded 10 ms wait and recovery attempt, avoiding indefinite reuse
of the last value after a missed completion. Host verification:
`tools/test-h7-adc.ps1 -Compiler gcc`.

The Blueson A1 65A AM32 is documented as having a current sensor and telemetry,
but its manufacturer page/manual do not specify the Betaflight ADC calibration
scale. The FC's 1052 default is not a verified calibration for this particular
ESC. Current readings here use the analog current signal on PC2, not AM32 serial
telemetry on UART8. An 8-pin cable connects the boards without separate pad
soldering; diagnose conversion activity and sensor voltage before assigning a
hardware fault or changing the calibration.

[Blueson A1 manufacturer specifications](https://sequremall.com/products/sequre-blueson-a1-6s-8s-70a-4in1-esc)

H7 external ADC acquisition time is now 387.5 ADC clock cycles, matching the
Betaflight H7 external-channel configuration (previously 64.5). The longer
acquisition gives the analog input more settling time after switching between
voltage and current. This is not evidence that settling caused the reported
fixed reading; physical verification is still needed.

The SEQURE H743 V2 uses the H743 LQFP100 PC2_C/PC3_C analog pins. These connect
directly to ADC3 inputs 0/1. The previous ADC1 inputs 12/13 selected the internal
PC2/PC3 paths, not these direct analog inputs. FlightCode now uses ADC3, matching
Betaflight's H743 pin map. Successful conversions on the previous inputs could
therefore still produce misleading readings. The host regression imports the
actual board definitions and tests ADC initialization as well as sampling, so
it detects an ADC1 or channel 12/13 regression. The corrected build still needs
physical verification of both current response and battery voltage.

This fix leaves the settings structure, version 28 and flash address 0x081E0000
unchanged. Updating with the FlightCode Configurator Firmware tab erases only
sectors occupied by the application, leaving saved settings intact. A full-chip
erase in another flashing tool would erase them.

[Betaflight H743 ADC pin map](https://github.com/betaflight/betaflight/blob/master/src/platform/STM32/adc_stm32h7xx.c)
and [ST H743VITx pin definitions](https://github.com/STMicroelectronics/STM32_open_pin_data/blob/master/mcu/STM32H743VITx.xml).

### Current display response

Current is now filtered on each fresh ADC sample (nominally 20 Hz), using a
100 ms PT1 time constant. A sustained step reaches approximately 95% in 400 ms,
instead of the previous eight-sample averaging and smoothing that needed several
seconds. The 1052 calibration scale and voltage filtering are unchanged. Short
throttle pulses are still smoothed; displayed current is not a peak-hold meter.
The ADC host regression checks a 200 ms pulse, settling, decay, and steady scaling.
No settings layout/version or flash storage changes are required for this update.


## ADC3 DMA diagnostic — 2026-10-18

SEQUREH7V2 now identifies as 1.6.2-adc3dma and advertises CURRENT_ADC_DMA. It continuously scans rank 1 VBAT (ADC3 channel 1) and rank 2 current (channel 0), using DMA1 Stream1 independently of motor DMA1 Stream0 and SPI DMA1 Stream4. A 32-byte-aligned AXI SRAM buffer is processed in completed half-buffer callbacks with cache invalidation. Each published raw current value is the mean of 64 pairs; the sample counter counts physical pairs and advances much faster than the former 20 Hz sampler. Existing diagnostic packet fields remain compatible. ADC errors are reported, and a halted stream retains its last acquisition timestamp so the Configurator can mark stale readings.

The 1052 current scale, voltage divider, settings layout and reserved flash addresses are unchanged. Current uses a 100 ms PT1 filter; voltage uses an approximately 2.5 s time constant. The new tools/test-h7-adc-dma.ps1 regression tests the actual DMA processing code with simulated transfers. Hardware compilation and host tests do not validate the physical sensor. Compare USB-only, battery/motors stopped and battery/motors running without propellers before interpreting the measurement. The diagnostic remains unverified on hardware.


## USB recovery build — 2026-10-18

After a reported loss of CDC enumeration with the backup/DMA build, the SEQURE target was returned to its previous polled ADC implementation while retaining the complete settings backup protocol. The experimental DMA module is no longer included or advertised. The firmware identifies as 1.6.2-backup-recovery. This isolates the experiment; it does not establish the physical cause of the failed startup. Settings layout version 28 and address 0x081E0000 are unchanged. The build still requires a cold-start test on the physical board.

The recovery build also fixes an identified ROM-to-application startup defect: the bundled H7 SystemInit leaves VTOR unchanged unless USER_VECT_TAB_ADDRESS is enabled, which this project does not enable. board_init now masks interrupts, stops inherited SysTick, clears pending/enabled NVIC interrupts, points VTOR to application flash and restores interrupt delivery before HAL_Init. Without this, a DFU jump can keep ROM vectors or masked interrupts and prevent HAL/USB startup. `tools/test-h7-startup.ps1` compiles the actual entry sequence with both inherited ROM state and cold-reset state. Physical boot verification remains required.

During recovery the board settings sector at 0x081E0000 was read without writing the FC. Its stored version-28 FNV checksum was verified, and a complete Configurator-compatible CRC32 JSON backup was generated from the 252-byte settings payload. The application's flash contents matched the uploaded backup/DMA image byte for byte, excluding a partial flash as the explanation for that image.

Physical recovery result: the 1.6.2-backup-recovery image was programmed over ROM DFU on the connected SEQURE H7. STM32CubeProgrammer erased only application sectors 0 and 1 and verified the download. A second read of the complete 128 KiB settings sector matched the saved pre-flash sector byte for byte (SHA256 comparison). Starting the application from DFU then enumerated USB CDC VID 0483 / PID 5740 as COM14 with Windows status OK. An independent serial HELLO check could not open COM14 (access denied); application-protocol verification was therefore not completed by this test.
