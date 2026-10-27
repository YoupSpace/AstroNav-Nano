# AstroNav Nano Firmware

Firmware for the **AstroNav Nano**, a small RP2350 flight computer for model rockets by
[YoupSpace](https://youpspace.com/). It detects launch, apogee and landing from an IMU
and a barometer, fires the deployment charge at apogee, and saves every flight as a CSV log
you can open over USB.

> **Safety first.** Read the [Safety](https://wiki.youpspace.com/safety/) page before
> powering, flashing or flying. Keep pyro charges and igniters disconnected while
> developing and bench testing.

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

- **Apogee** needs a trustworthy barometer and free fall on the IMU. Physically impossible
  pressure jumps, such as the ejection charge's pressure spike, are filtered out.
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

## Disclaimer

This firmware does not replace vehicle-specific engineering, recovery-system testing, range
rules or local law. Use is subject to the YoupSpace
[Terms and Conditions](https://youpspace.com/terms-and-conditions/) and
[Privacy Policy](https://youpspace.com/privacy-policy/).
