# AstroNav Nano Firmware

Firmware for the AstroNav Nano RP2350 flight controller and flight-data
logger. The Nano measures motion and pressure, detects flight phases, records
flight data, and can control apogee deployment when configured and enabled.

## Documentation

The project wiki is the primary user documentation. Start here:

- [AstroNav Nano wiki](https://wiki.youpspace.com/)
- [Safety](https://wiki.youpspace.com/safety/)
- [Getting started](https://wiki.youpspace.com/getting-started/)
- [Firmware update](https://wiki.youpspace.com/firmware-update/)
- [Configuration](https://wiki.youpspace.com/configuration/)
- [Pre-flight checklist](https://wiki.youpspace.com/pre-flight/)
- [Flight states](https://wiki.youpspace.com/flight-states/)
- [Flight logs](https://wiki.youpspace.com/flight-logs/)
- [Troubleshooting](https://wiki.youpspace.com/troubleshooting/)
- [AstroNav MC desktop software](https://wiki.youpspace.com/software/astronav-mc/)

Read [Safety](https://wiki.youpspace.com/safety/) before powering, configuring,
flashing, testing, or flying the hardware. Keep all pyro and other hazardous
loads disconnected during development and bench testing.

## Build and upload

This repository uses VS Code with the PlatformIO IDE:

```powershell
pio run
pio run --target upload --upload-port COM3
pio device monitor
```

Replace `COM3` with the port assigned to the intended board. For the complete
setup and firmware-update procedure, see the
[firmware update guide](https://wiki.youpspace.com/firmware-update/).

### UF2 build artifact

A successful build creates a Git-versioned UF2 file under:

```text
.pio/build/rpipico2/firmware-V<version>_<commits-ahead>_<commit-sha>.uf2
```

Copy that file to the `RPI-RP2` drive while the RP2350 is in BOOTSEL mode.
Older firmware artifacts are removed automatically before a build.

Git metadata is included when available. Builds also work from a source copy
without Git or repository access by using the fallback version information
stored in [`include/build_info.h`](include/build_info.h).

To force a complete rebuild:

```powershell
pio run --target clean
pio run
```

## Repository notes

- [`platformio.ini`](platformio.ini) contains the board, framework, and library configuration.
- [`generate_version.py`](generate_version.py) generates build metadata and the firmware artifact name.
- [`src/`](src/) contains the firmware source.
- [`SOFTWARE_DESIGN.md`](SOFTWARE_DESIGN.md) contains the developer-oriented design reference.

This firmware is not a substitute for vehicle-specific engineering, recovery
system validation, range rules, or applicable laws. Follow the
[YoupSpace Terms and Conditions](https://youpspace.com/terms-and-conditions/)
and [Privacy Policy](https://youpspace.com/privacy-policy/).
