# AstroNav Nano Self-Test Software Design

This document describes the firmware in enough detail that the current program can be recreated from it.

## Audience and scope

This design guide is intended for people who want to understand what the firmware is doing, how it boots, how it validates hardware, and how the build metadata and production tooling fit into the project.

This document is helpful for:

- new developers joining the project
- hardware testers validating the board
- production teams using the optional manufacturing metadata flow
- anyone debugging a boot, health, or flight-state issue

This document is not a substitute for the actual safety procedures and launch protocol. It explains the software and hardware behavior, not the legal or operational risk decisions.

## Chapter map

1. Purpose and system overview
2. Hardware assumptions and pin map
3. Power and input-voltage detection
4. Build-time metadata and production signing
5. Persistent state and startup flow
6. Hardware validation and test functions
7. Flight-state logic and sensor behavior
8. USB configuration and operational notes

## Purpose

The firmware is a board health and wiring verification program for the AstroNav RP2350A-based PCB. It boots, initializes the board peripherals, checks the IMU and barometer, measures the board input voltage through a resistor divider on GPIO27, reports the detected power source, and keeps a status LED updated with overall board state.

The project also includes a platform build step that generates build metadata and optional production signing. This is handled by [generate_version.py](generate_version.py), which writes [include/build_info.h](include/build_info.h) during compilation.

The program is intentionally simple and deterministic:

- It does not control mission logic.
- It does not stream sensor data externally.
- It performs self-test and continuous health reporting only.

## Hardware Assumptions

The program assumes:

- MCU platform: Raspberry Pi Pico 2 / RP2350 running the Arduino framework under PlatformIO.
- SPI bus is used for the IMU and BMP581.
- A single WS2812 status LED is attached.
- GPIO27 is connected to an input-voltage sense divider.
- GPIO1 is a pyro output that must be held low.
- GPIO2 and GPIO3 are servo outputs that must be held low.

## Pin Map

### SPI and peripheral pins

- GPIO4: MISO
- GPIO6: SCK
- GPIO7: MOSI
- GPIO9: IMU chip select
- GPIO12: BMP581 chip select
- GPIO0: WS2812 status LED

### Additional control pins

- GPIO1: pyro channel, output low at boot
- GPIO2: servo output, output low at boot
- GPIO3: servo output, output low at boot
- GPIO27: input-voltage sense

## Input Voltage Sensing

GPIO27 measures a divided version of the board input rail.

### Divider values

- Top resistor: 20k from input rail to the sense node
- Bottom resistor: 10k from sense node to ground
- GPIO27 is connected to the sense node

### Electrical meaning

The sense pin sees one third of the real input voltage:

- `V_sense = V_in / 3`
- `V_in = V_sense * 3`

The firmware treats this reading as the board input rail before the regulator. It is not a regulated 3.3 V rail reading.

### Source classification

The measured input rail is classified into one of three expected power sources:

- USB power: 4.5 V to 5.5 V
- 1S LiPo: 3.0 V to 4.35 V
- 2S LiPo: 6.0 V to 8.4 V

Anything outside those ranges is reported as `Unknown`.

## Libraries Used

The firmware includes:

- Arduino core
- SPI
- Adafruit NeoPixel
- SparkFun BMP581 Arduino Library
- math.h

## Build-time Metadata and Production Signing

Before the firmware is compiled, the Python build script validates the git tag, generates the version string, and writes the header file that exposes build metadata to the firmware.

Optional production metadata:

- A local secrets directory is checked for a manufacturing key file: warranty_key.txt
- A serial registry CSV is checked for serial tracking: serial_registry.csv
- If the key or registry are absent, the build still succeeds and prints a warning instead of failing
- If the files are present and valid, the script uses them to generate official warranty signatures and serial registration data

This allows open-source or local builds to compile without the manufacturing assets, while keeping the production path intact when those files are available.

## Persistent State

The program stores a small set of global runtime state:

- `boardHealth`: boolean summary of each test and overall warning/critical state
- `lastSummaryMs`: timestamp for periodic console summaries
- `bootWhiteUntilMs`: initial LED hold time after boot
- current IMU sample values
- current barometer values
- current measured input voltage
- current detected power source text

## Flight Log Storage

A flight is recorded in two stages: samples are collected in a RAM buffer during the flight and written to the flash filesystem as a CSV file after landing (or after the landing timeout).

### Current configuration

| Setting | Where | Value | Meaning |
|---|---|---|---|
| RAM log buffer | `FLIGHT_LOG_CAPACITY` in [src/main.cpp](src/main.cpp) | 6144 samples | 44 bytes per sample = 270 KB, 307 s at 20 Hz |
| Flash filesystem | `board_build.filesystem_size` in [platformio.ini](platformio.ini) | 2M | Holds `Settings.ini`, the info files and `/logs/Flight_*.csv` |

Rule of thumb for planning:

- RAM: 880 bytes per second of flight (20 samples/s x 44 bytes).
- Flash: about 135 bytes per CSV row, so about 2.7 KB per second of flight. A full 307 s log is roughly 830 KB; a typical 60 s flight is roughly 160 KB.
- The landing timeout is at least 300 s, so with default settings a flight is saved before the RAM buffer runs out.

### Long flights and full storage

- Nothing is written to flash between launch and landing for normal flights. Only when a flight outlasts 75% of the RAM buffer, the pyro decision is final and the state is `Descent`, the oldest samples are moved to the log file in batches of 20 rows per sample tick, so the descent keeps being logged. Such a log has the usual header (summary so far) and `FINAL_` summary lines at the end of the file.
- If the RAM buffer fills before apogee (very long climb or coast), further samples are dropped and counted in `RAM_DROPPED_SAMPLES`.
- Before writing, the firmware checks free space and keeps a 16 KB reserve for `Settings.ini`. Rows that do not fit are counted (`STORAGE_DROPPED_SAMPLES` / `STORAGE_FULL` line in the log, `LAST_LOG_STATUS` in `Settings.ini`), never silently cut off. Existing files are never deleted by the firmware.
- Altitude and vertical speed are stored as 32-bit values, so flights above 327 m or faster than 327 m/s are logged without saturation.

### Adjusting the storage

Both values can be changed, but keep these limits in mind:

- **RAM buffer:** the RP2350 has 512 KB RAM. With 6144 samples the firmware uses about 288 KB statically, leaving about 230 KB for heap and stack (FatFS, the flash translation layer, USB mass storage, file buffers). Keep at least 64 KB free. The practical maximum is about 9000 samples. Check the `RAM:` line of the build output after every change.
- **Flash filesystem:** the board has 4 MB flash. Firmware, filesystem and the 4 KB EEPROM area share it, so `filesystem_size` can go up to about 3.5 MB while the firmware stays below about 500 KB. A larger filesystem also uses a little more heap for the flash translation layer (about 8 KB per extra MB).
- **Changing `filesystem_size` reformats the filesystem on the first boot after the upload.** The filesystem start address moves, the old metadata no longer matches and is discarded, and a fresh empty filesystem is created. Copy all logs off the board first and re-enter your `Settings.ini` values afterwards.
- Saving a long log takes several seconds after landing (estimated 10-20 s for a full buffer). The status LED can stop updating during that time; do not remove power until it continues.

### Symptoms of a storage configuration that is too large

If the flight controller starts behaving strangely after the storage was changed, suspect memory first:

- unexpected reboots or freezes, especially while saving a log, while the USB drive is mounted, or right after boot
- the USB drive not appearing or appearing empty
- `FLASH_FS_OK` / `flashFs` reporting `FAIL`, or `Settings.ini` resetting itself

These usually mean the heap or stack has run out. Reduce `FLIGHT_LOG_CAPACITY` (for example back to 4096 or 2048) and rebuild; the RAM buffer does not affect the files on flash.

## Constants

### Sensor scaling

The IMU raw values are converted using fixed scale factors:

- Accelerometer: 1024 counts per g (`ACCEL_CONFIG0 = 0x06`: full scale ±32 g)
- Gyroscope: 8.2 counts per deg/s (`GYRO_CONFIG0 = 0x06`: full scale ±4000 deg/s)

These numbers are used after reading raw 16-bit sensor data. Both registers use full-scale select code 0, the largest range. Until v1.2.1 the gyro was divided by 16.4 (the ±2000 deg/s factor), so gyro values were half the real rate. The flight log stores the gyro in 0.1 deg/s steps in 16 bits, so logged rates saturate at ±3276.7 deg/s. The TestTools project still uses 16.4.

### ADC conversion

The voltage-sense code assumes a 12-bit ADC reading and a 3.3 V ADC reference.

- ADC counts range: 0 to 4095
- ADC reference: 3.3 V

The input voltage is computed from the measured sense voltage and divider ratio.

### Flash signature

The flash test uses the constant string `ASTRONAV_PCB_TEST` as a checksum source.

## Startup Flow

The setup sequence is important and should be preserved.

1. Start Serial at 115200.
2. Initialize the WS2812 LED.
3. Set LED brightness to 40.
4. Set the LED state to Busy.
5. Hold the boot-white state for about 2 seconds using `bootWhiteUntilMs`.
6. Delay 400 ms.
7. Set ADC resolution to 12 bits.
8. Wait up to 3 seconds for Serial connection.
9. Print a banner to the console.
10. Configure GPIO1, GPIO2, and GPIO3 as outputs and drive them low. (The flight firmware does this first, before step 1; see Boot-Time Pin Behavior.)
11. Configure the SPI chip-select pins as outputs.
12. Configure GPIO27 as input.
13. Put the chip-select pins high by default.
14. Pulse the BMP581 chip-select low then high to force SPI mode.
15. Configure SPI pins and start the SPI peripheral.
16. Run core tick, flash, and heap tests.
17. Read and classify input voltage from GPIO27.
18. Initialize the BMP581 and read one sample.
19. Detect and configure the IMU.
20. Verify the IMU data stream by reading several samples.
21. Read one final IMU frame into the live status variables.
22. Compute warning and critical status flags.
23. Print a boot summary.
24. Delay 500 ms.
25. Update the status LED.
26. Store the current time in `lastSummaryMs`.

## Boot-Time Pin Behavior

At boot, the firmware intentionally forces the following outputs low:

- GPIO1 low
- GPIO2 low
- GPIO3 low

In the flight firmware this is the very first thing `setup()` does, before the serial wait, OTP access and sensor setup, so those outputs do not float or accidentally activate. Before the firmware runs, the RP2350 pads default to inputs with pull-downs; the pyro MOSFET gate should still have its own external pull-down resistor.

## Flight Safety Rules

The flight logic in [src/main.cpp](src/main.cpp) (`updateFlightStateFromSamples`) follows these rules. Change them only deliberately and re-test (ground test plus toss test).

- **Pyro gates** (`tryApogeePyro`): never before apogee; only with `FIRE_PYRO_APOGEE=TRUE`; only with a barometer reading accepted in the same sample; only if the peak is at least `MIN_PYRO_FIRE_HEIGHT_M` above the launch altitude. A barometer glitch at apogee may retry for `PYRO_RETRY_WINDOW_MS` (1 s).
- **False launch** (`abortFalseLaunch`): a launch that comes to rest (landing confirmation or landing timeout) in `Boost`/`Coast`, before any apogee, with a reliable barometer and less than `FALSE_LAUNCH_MAX_CLIMB_M` (3 m) of climb, returns to `Idle`. The RAM log is discarded and the flight number reused. This is the only path from `Boost`/`Coast` back to `Idle`, and the pyro cannot have fired on it.
- **Apogee triggers:** `velocity` and `drop` (confirmed over 2-3 samples) and `peak_hold` all require free fall on the IMU (≤ 0.5 g) or, without the IMU, a barometric descent of at least 1 m/s. `baro_descent` is the fallback for a rocket whose IMU never reads free fall: peak climb rate ≥ 10 m/s, IMU below the launch threshold (not under thrust), descent ≥ 3 m/s and ≥ 2 m below the peak, for 3 samples.
- **Peak altitude** is updated only from barometer readings accepted in that sample, never from the extrapolated estimate during a dropout.
- **Landing timeout** runs even when both sensors have failed, so the log is always saved.
- **No flash writes while the USB drive is attached:** `persistDeviceProfile`, `ensureUsbInfoFiles` and the log writer return early while `usbDriveReady` is set. `Settings.ini` is written once per boot: before the drive is exported in USB mode, or at the end of `setup()` in flight mode.
- **USB exit commands** (`FLIGHT`, `EXITUSB`, `ARM`) and the eject callback act only in `UsbMode`. `powerMode` is re-measured every sample and can read "USB" on a sagging battery, so it is not used for this decision.
- **SPI:** `beginTransaction()` comes before chip select low, and chip select goes high before `endTransaction()`. The barometer uses SPI mode 0 and the IMU mode 3; switching modes with a chip already selected would give it a stray clock edge.
- **No watchdog** on purpose: after a reset in flight the board cannot calibrate (it is moving) and goes to `Fault`, so a watchdog would not restore deployment and would add a new failure mode during long blocking work (calibration, saving a long log).

## Test Functions

### Core tick test

A simple time test verifies that `millis()` advances across a short delay.

Expected result:

- `millis()` after a 2 ms delay must be greater than the starting value.

### Flash read test

The flash test XORs every byte in the flash signature string.

Expected result:

- The checksum must be non-zero.

This is a lightweight code-integrity / storage sanity check rather than a full flash readback test.

### Heap test

The heap test allocates 256 bytes with `malloc`, fills the buffer with a known pattern, verifies a few bytes, and frees it.

Expected result:

- Allocation succeeds
- First, second, and last test bytes match the pattern

## Voltage Measurement Algorithm

The code reads GPIO27 multiple times and averages the samples.

### Algorithm

1. Take 8 ADC samples.
2. Add the raw counts into a 32-bit total.
3. Compute the average counts.
4. Convert counts to sense voltage:
   - `senseVoltage = averageCounts * 3.3 / 4095`
5. Convert sense voltage to input voltage:
   - `inputVoltage = senseVoltage * 3`
6. Return the measured input voltage if it is greater than 0.1 V.

### Result handling

The measured voltage is stored in `currentVinVoltage`.
The corresponding source label is stored in `currentPowerSource`.

## Power Source Classification Rules

The measured input voltage is mapped to a source label:

- 4.5 V to 5.5 V -> `USB power`
- 3.0 V to 4.35 V -> `1S LiPo`
- 6.0 V to 8.4 V -> `2S LiPo`
- otherwise -> `Unknown`

If the source is `Unknown`, the firmware sets the warning flag.

## IMU Behavior

The IMU uses SPI and register-level access.

### WHO_AM_I / transport detection

The IMU transport is probed by reading register `0x72` using two possible SPI modes:

- SPI mode 3 first
- SPI mode 0 second

The first valid non-`0x00`, non-`0xFF` response is considered transport OK and also selects the working SPI mode.

The WHO_AM_I value must equal `0xE9`.

### IMU configuration sequence

After confirming WHO_AM_I, the firmware writes:

- `REG_MISC2 = 0x02`
- `PWR_MGMT0 = 0x0F`
- `ACCEL_CONFIG0 = 0x06`
- `GYRO_CONFIG0 = 0x06`

Then it reads those registers back and requires them to match.

### IMU sample decoding

The firmware reads 12 bytes starting at accelerometer data register `0x00`.

The bytes are interpreted as little-endian 16-bit signed values:

- accel X: bytes 0-1
- accel Y: bytes 2-3
- accel Z: bytes 4-5
- gyro X: bytes 6-7
- gyro Y: bytes 8-9
- gyro Z: bytes 10-11

Then the raw counts are converted to physical units:

- accel in g
- gyro in deg/s

### IMU stream verification

The firmware reads 8 samples and counts how many are finite and not clearly invalid.

Expected result:

- At least 6 valid samples out of 8

## Barometer Behavior

The BMP581 is initialized over SPI after the SPI bus is configured.

The firmware reads sensor data using the library call, converts pressure from Pa to hPa, and accepts the reading only if pressure is in a plausible atmospheric range.

Expected result:

- Temperature and pressure read successfully
- Pressure is between 300 hPa and 1200 hPa

## Health Model

The health summary uses two higher-level state flags.

### Warning

Warning is set when any non-critical monitored subsystem is degraded, including:

- barometer read failure
- IMU stream failure
- unknown input-source classification

### Critical

Critical is set when any fundamental board function fails:

- core tick
- flash check
- heap test
- SPI transport
- IMU WHO_AM_I
- IMU configuration
- IMU stream

Critical overrides warning behavior in LED selection.

## LED Behavior

The WS2812 LED is the user-visible state indicator. In the flight firmware it is updated every 25 ms by `updateStatusLed()`, and the colours are defined in `setLedProfile()` in [src/main.cpp](src/main.cpp).

### States

Each state has its own colour; boot, USB and fault colours are never reused. Breathing means waiting on the ground, solid means in flight, blinking means it needs attention.

| Flight state | `LedProfile` | Colour (R,G,B) | Pattern |
| --- | --- | --- | --- |
| Booting (and the first 2 s) | `Boot` | white (255,255,255) | breathing, 2 s |
| UsbMode | `Usb` | blue (0,128,255) | breathing |
| Calibrating | `Calibrating` | yellow (255,255,0) | blink 250/250 ms |
| Idle (and the unused Armed) | `Idle` | green (0,255,0) | breathing |
| Boost | `Boost` | orange (255,80,0) | solid |
| Coast | `Coast` | purple (160,0,255) | solid |
| Descent | `Descent` | cyan (0,255,255) | solid |
| Landed | `Landed` | magenta (255,0,255) | blink 200/200 ms |
| Fault | `Fault` | red | blink 150/150 ms |

Global brightness is 40/255.

### State selection

1. If `critical`, a low-battery fault or a sensor fault is latched, blink red (100 ms), whatever the state.
2. Else, during the first 2 s after boot, show `Boot`.
3. Else show the colour of the current flight state.

## Serial Commands

| Command | Allowed in | Effect |
| --- | --- | --- |
| `DUMP_HEALTH` | any state | Prints health flags (also read by the upload script). |
| `DUMP_OTP` | any state | Prints the OTP identity. |
| `FLIGHT` / `EXITUSB` / `ARM` | `UsbMode` | Detaches the drive, remounts, reloads `Settings.ini`, calibrates, goes to `Idle`. |
| `PYROTEST` (twice within 5 s) | `Idle`, before launch | 1 s pyro pulse; does not latch the flight pyro. |
| `GYROTEST` | `Idle`, before launch | For 10 s integrates the bias-corrected gyro (`updateGyroTest`) and prints the total angle per axis. A 90° turn must read about ±90. Read-only; cancelled when the board leaves `Idle`. |

## Serial Output

The firmware prints both boot-time and runtime summaries.

### Boot summary

The boot output lists:

- core tick
- flash read
- heap alloc
- IMU transport
- IMU WHO_AM_I
- IMU config
- IMU stream
- barometer
- VIN/input rail and classified power source
- PASS / WARN / FAIL result

### Runtime summary

Once per second, the firmware prints a condensed line containing:

- overall health
- individual health fields
- measured VIN and classified source
- current accel, gyro, temperature, and pressure values

## Main Loop Behavior

The loop runs continuously.

Per cycle it does the following:

1. Re-read input voltage from GPIO27 and refresh the source label.
2. If the IMU is initialized, read a new IMU frame.
3. If the IMU frame is valid, update the live accel/gyro values.
4. Read the barometer.
5. Recompute warning and critical flags.
6. Update the LED.
7. Print a runtime summary every 1 second.
8. Delay 10 ms.

## Pseudocode

```text
setup():
  start serial
  init LED
  set LED busy
  start boot timer
  set ADC to 12-bit
  wait for serial briefly
  print banner

  configure GPIO1, GPIO2, GPIO3 as outputs
  drive GPIO1 low
  drive GPIO2 low
  drive GPIO3 low

  configure chip selects and VIN sense pin
  force BMP581 into SPI mode
  configure SPI pins and start SPI

  run core tick test
  run flash test
  run heap test

  read VIN input voltage
  classify power source

  init barometer and read sample
  configure IMU and verify stream

  compute warning/critical
  print boot summary
  update LED

loop():
  read VIN input voltage
  classify power source

  if IMU initialized:
    read IMU frame
    if valid:
      update live IMU values
    else:
      mark IMU stream failed

  read barometer
  recompute warning/critical
  update LED

  every 1 s:
    print runtime summary

  delay 10 ms
```

## Rebuild Notes

To recreate this program from scratch, implement the following in the same order:

1. Define all pins and constants.
2. Create global health and sensor state.
3. Implement LED state helpers.
4. Implement the core tick, flash, and heap tests.
5. Implement GPIO27 voltage reading and source classification.
6. Implement SPI register helpers.
7. Implement IMU frame decoding and stream verification.
8. Implement IMU configuration.
9. Implement barometer reading.
10. Implement boot and runtime summary printing.
11. Configure outputs low in `setup()` before peripheral work.
12. Initialize SPI and sensors.
13. Update health flags and LED in the loop.

## Build Configuration

The project is configured in PlatformIO for the `rpipico2` environment using the Arduino framework.

Libraries declared in `platformio.ini`:

- Adafruit NeoPixel
- SparkFun BMP581 Arduino Library
- ICM-42688

## Practical Meaning of the VIN Reading

The voltage read from GPIO27 is the board input rail, not the regulated MCU rail.

This means it tells you which supply is present:

- USB power should read around 5 V
- 1S LiPo should read around 3.0 to 4.35 V
- 2S LiPo should read around 6.0 to 8.4 V

That is the value the firmware uses for user reporting and source detection.
