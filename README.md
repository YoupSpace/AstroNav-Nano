# AstroNav Nano

**The world's smallest rocket flight computer, as far as we know: 16 × 13.6 mm.**

![AstroNav Nano next to a 2 euro coin](https://image.easyeda.com/oshwhub/pullImage/d6e4ca3b0097472b9137191767315ac6.jpg)

The AstroNav Nano by [YoupSpace](https://youpspace.com/) logs your whole flight, detects
apogee and fires your parachute charge. It takes up 20% less board area than the previous
record holder (BPS.space, 17 × 16 mm) and still saves every flight as a CSV log you can open
over USB.

This repository holds the **firmware**. The hardware design files (schematic, PCB, BOM) and the
3D printable holder and case are on the
[OSHWLab project page](https://oshwlab.com/youphoutkamp/project_cafdwlpm).

> **Safety first.** Read the [Safety](https://wiki.youpspace.com/safety/) page before
> powering, flashing or flying. Keep pyro charges and igniters disconnected while
> developing and bench testing.

## Specs

| | |
|---|---|
| Size | 16 × 13.6 mm |
| Microcontroller | Raspberry Pi RP2350A |
| IMU | ICM-45686 (accelerometer + gyroscope, up to 32 G) |
| Barometer | BMP581 |
| Power | 1S or 2S LiPo, or 5 V USB, with reverse polarity protection |
| Power draw | about 0.1 W |
| Logging | 20 samples/s: acceleration, rotation, temperature, pressure, altitude, vertical speed |
| Storage | 2 MB flash: about 12 typical 60-second flights |
| Outputs | 1 pyro channel, 2 RGB status LEDs |
| Expansion | GPIO 2 and GPIO 3 (UART, I²C, PWM) |
| USB | Serial and mass storage, no drivers needed |

## The ecosystem

| Part | Where | License |
|---|---|---|
| Firmware | This repository | CC BY-NC 4.0 |
| Hardware design files and 3D printable files | [OSHWLab](https://oshwlab.com/youphoutkamp/project_cafdwlpm) | CC BY-NC 4.0 |
| User documentation | [wiki.youpspace.com](https://wiki.youpspace.com/) | |
| AstroNav Mission Control (desktop app for Windows and Linux) | [Releases](https://github.com/YoupSpace/astronav-mc-releases) | Closed source, free to use |

**AstroNav Mission Control** takes you through the whole loop: plan a flight (estimated apogee
and top speed from your rocket's weight and motor), configure the board safely, update the
firmware with one click, replay every flight with synchronized charts, compare what you planned
with what really happened, and export your launch video with a telemetry overlay.

## Documentation

The [AstroNav Nano wiki](https://wiki.youpspace.com/) is the main user documentation.

| I want to... | Page |
|---|---|
| Set up a new board | [Getting started](https://wiki.youpspace.com/getting-started/) |
| Change flight settings | [Configuration](https://wiki.youpspace.com/configuration/) |
| Prepare for a launch | [Pre-flight](https://wiki.youpspace.com/pre-flight/) and the [operating checklist](https://wiki.youpspace.com/checklist/) |
| Understand what happened in flight | [Flight states](https://wiki.youpspace.com/flight-states/) and [flight operation](https://wiki.youpspace.com/flight-operation/) |
| Read my flight data | [Flight logs](https://wiki.youpspace.com/flight-logs/) |
| Update the firmware | [Firmware update](https://wiki.youpspace.com/firmware-update/) |
| Use the desktop app | [AstroNav MC](https://wiki.youpspace.com/software/astronav-mc/) ([installation](https://wiki.youpspace.com/software/installation/)) |
| Fix a problem | [Troubleshooting](https://wiki.youpspace.com/troubleshooting/) |
| Go deeper | [Advanced](https://wiki.youpspace.com/advanced/) |

In this repository:

- [ASTRONAV_NANO_USER_MANUAL.md](ASTRONAV_NANO_USER_MANUAL.md): offline user manual.
- [SOFTWARE_DESIGN.md](SOFTWARE_DESIGN.md): developer reference, including
  [flight log storage](SOFTWARE_DESIGN.md#flight-log-storage) limits.

## How a flight works

`Idle` → `Boost` → `Coast` → `Descent` (apogee) → `Landed`

- **A bump is not a flight.** If a "launch" comes to rest without climbing 3 m, the board
  returns to `Idle` and stays ready for the real launch. The LED then goes orange → purple →
  green within a few seconds, and nothing is written to `/logs`.
- **Apogee** needs a trustworthy barometer and free fall on the IMU. If the IMU never reads
  free fall (tumbling or spinning rocket), a clear barometer descent after a real climb still
  triggers apogee, slightly later. Physically impossible pressure jumps, such as the ejection
  charge's pressure spike, are filtered out.
- **The pyro** fires at apogee only if the barometer is reliable and the rocket climbed at least
  `MIN_PYRO_FIRE_HEIGHT_M` above the launch point (default 5 m). That protects people
  handling an armed board.
- **Landing** is confirmed when one sensor is completely calm and the other barely moves. A
  timeout makes sure the log is always saved.
- **Nothing is written to flash during a normal flight.** The log is saved after landing to
  `/logs/Flight_NNNNNNNN.csv`, and its header shows what the board decided (detected apogee,
  trigger, pyro status, landing trigger).

The `ESTIMATED_*` settings are a baseline: good estimates make the board react faster and more
precisely, but they never block an event. See [Configuration](https://wiki.youpspace.com/configuration/).

## Status LED

Breathing = waiting on the ground, solid = in flight, blinking = needs your attention.

| State | LED |
| --- | --- |
| Booting | white, breathing (about 2 s) |
| USB mode | blue, breathing |
| Calibrating | yellow, blinking: keep the board still |
| Idle (ready for launch) | green, breathing |
| Boost | orange, solid |
| Coast | purple, solid |
| Descent (after apogee) | cyan, solid |
| Landed | magenta, blinking |
| Fault (pyro off), or low battery / sensor warning (still armed) | red, fast blinking |

Typical sequences:

- **Power-up on battery:** white → yellow → green.
- **Power-up on USB:** white → blue. After ejecting the drive: yellow → green.
- **Flight:** green → orange → purple → cyan → magenta.
- **Bump on the pad (false launch):** green → orange → purple → green.
- **Magenta on the pad without a flight:** the board was carried 3 m or more after a bump.
  Power-cycle it and let it calibrate again.

## Build and upload

Requires [VS Code](https://code.visualstudio.com/) with the
[PlatformIO IDE](https://platformio.org/install/ide?install=vscode) extension.

```powershell
pio run                                         # build
pio run --target upload --upload-port COM3      # upload (use your board's port)
pio device monitor                              # serial console, 115200 baud
```

A build produces `.pio/build/rpipico2/firmware-V<version>_<commits-ahead>_<commit>.uf2`.
You can also copy that file to the `RP2350` drive while the board is in BOOTSEL mode.
The version comes from the latest `firmware-vX.Y.Z` (or `vX.Y.Z`) git tag.

Platform, Arduino core and libraries are pinned in [platformio.ini](platformio.ini) for
reproducible builds.

> Changing `board_build.filesystem_size` erases all logs and `Settings.ini` on the next boot.
> Copy your logs off the board first.

## Repository layout

| Path | Contents |
|---|---|
| [src/main.cpp](src/main.cpp) | Flight state machine, altitude estimate, logging, settings |
| [src/hardware_control.cpp](src/hardware_control.cpp) | IMU, barometer, voltage sensing, pyro output |
| [src/otp_memory.cpp](src/otp_memory.cpp) | Board identity stored in the RP2350 one-time-programmable memory |
| [src/usb_info_files.cpp](src/usb_info_files.cpp) | Files shown on the USB drive (`Settings.ini`, `Howto.txt`) |
| [generate_version.py](generate_version.py) | Pre-build script: version, build metadata, production provisioning |
| [TestTools/](TestTools/) | Separate hardware self-test firmware |

`include/build_info.h` is generated on every build and is not tracked in git.

## License

The firmware, documentation, hardware design files and 3D printable files are licensed under
[CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/). You may study, modify and
share them and build boards for your own use, with credit to YoupSpace. Selling boards, kits
or derivatives needs written permission. Because of that non-commercial clause, this project is
source-available, not "open source" by the OSHWA or OSI definition. See [LICENSE](LICENSE).

## Disclaimer

This firmware does not replace vehicle-specific engineering, recovery-system testing, range
rules or local law. Use is subject to the YoupSpace
[Terms and Conditions](https://youpspace.com/terms-and-conditions/) and
[Privacy Policy](https://youpspace.com/privacy-policy/).
