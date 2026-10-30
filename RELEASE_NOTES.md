# Unreleased

## Flight logs

- **Flight-estimate settings in the log header:** `ESTIMATED_HEIGHT_M`, `HEIGHT_MARGIN_M`, `ESTIMATED_SPEED_MPS` and `SPEED_MARGIN_MPS`, the values this flight used. Mission Control shows them with the flight and compares them with how it really flew.

# AstroNav Nano Firmware v1.2.2

Safety release from a full review of v1.2.1. Updating does not erase your logs or settings. Do a ground test and a toss test before flying.

## Safety

- **A bump on the pad no longer disarms the parachute.** Previously a knock while handling the rocket (sliding it onto the rail, setting it down hard) could be taken as a launch. The board then declared `Landed` a few seconds later, ignored the real launch and never fired the pyro. Now a "launch" that comes to rest without climbing 3 m returns to `Idle`, and the board stays ready for the real flight. The LED goes orange → purple → green, and the serial log shows `[FLIGHT] False launch`.
- **Backup apogee for tumbling or spinning rockets.** Apogee required the IMU to read free fall. A rocket whose IMU never reads free fall (tumbling, fast spin, board far from the centre of gravity) never deployed. A new fallback declares apogee on a clear barometer descent (≥ 3 m/s, ≥ 2 m below the peak), only after a real powered climb (≥ 10 m/s) and not under thrust. The log shows `APOGEE_TRIGGER=baro_descent`.
- **No early apogee after a barometer dropout.** The peak altitude was also taken from the estimate while barometer readings were missing. That estimate overshoots while the rocket slows down, which could trigger apogee while still climbing. The peak now only uses real barometer readings.
- **Pyro output driven low first thing at boot**, before the serial wait and OTP access (up to 1.5 s earlier than before).
- **USB exit commands only in USB mode.** `FLIGHT`, `EXITUSB` and `ARM` were accepted whenever the battery voltage happened to read in the USB range (a sagging 2S, or a 1S LiPo with a USB cable). That re-ran the remount and calibration, and could stall the loop or end in `Fault` on the pad or in flight. They now only work in USB storage mode.

## Fixes

- **`Settings.ini` was still written while the USB drive was attached.** At the end of startup, the board rewrote `Settings.ini` after the drive had already been shown to the computer. This is the simultaneous-access problem v1.2.1 set out to fix. Writes are now blocked while the drive is attached, and `Settings.ini` is written once per boot instead of up to three times.
- **Gyro values were half the real rotation rate.** The IMU is set to ±4000 °/s, but the firmware converted the values as if it were ±2000 °/s. Landing and calibration stillness checks are now as strict as intended, and logged gyro values are correct; they saturate at ±3276.7 °/s in the log.
- **SPI order.** The IMU is selected only after the SPI bus is switched to its mode. Before, it could see a stray clock edge after a barometer transfer.
- **Landing timeout with failed sensors.** If both sensors failed during a flight, the landing timeout never ran and the log was never saved. It now runs regardless.
- **TestTools** uses the corrected gyro scale too.

## Changes

- **New status LED colours.** Each state has its own colour, and `Idle` and `Landed` no longer look alike. Breathing means waiting on the ground, solid means in flight, blinking means it needs attention:
  - **Idle:** green, breathing (was green-teal)
  - **Boost:** orange, solid
  - **Coast:** purple, solid
  - **Descent:** cyan, solid (was pink)
  - **Landed:** magenta, blinking (was teal)

  Boot (white), USB (blue), calibrating (yellow, blinking) and fault (red) are unchanged.
- **New `GYROTEST` serial command** (`Idle` in flight mode only, read-only). Turn the board 90° flat on the table within 10 s; the board prints the total angle per axis, which should be about ±90.

## Before you fly

- Run `GYROTEST` once: a 90° turn should read about ±90 (not 45 or 180).
- Check that the pyro MOSFET gate has an external pull-down resistor.

---

# AstroNav Nano Firmware v1.2.1

Bug-fix release for leaving USB mode. Updating from v1.2.0 does not erase your logs or settings.

## Fixes

- **Ejecting the USB drive could leave the board stuck in USB mode.** After an eject, the firmware read and wrote `Settings.ini` while the drive was still attached to the computer. The computer and the board could then use the flash at the same time, and the board could crash and reboot back into USB mode. The board now detaches the drive first, tells the computer the drive is gone, remounts the filesystem, and only then reads your settings. The same applies to the `EXITUSB` and `FLIGHT` serial commands.
- **Edits made over USB are reliably picked up.** The filesystem is remounted after the USB session, so changes to `Settings.ini` are read fresh instead of from data cached before the session.
- **No file writes while the drive is attached.** At boot, `Howto.txt`, `Website.url` and `Settings.ini` are now written before the drive is shown to the computer.
- **Health readback after upload.** The upload script asked the board for its health only once, while the board was still booting and re-connecting USB, so it often reported `readback unavailable`. It now retries for up to 20 seconds.

## Changes

- **USB serial stays available after leaving USB mode.** Previously the whole USB connection was dropped. Now only the drive is removed, so `DUMP_HEALTH`, `DUMP_OTP` and the ground `PYROTEST` keep working on USB power.
- **Reset reason at boot:** the serial log now starts with `[BOOT] firmware=... reset_reason=...`, so an unexpected reboot (crash, watchdog, brownout) is visible right away.

---

# AstroNav Nano Firmware v1.2.0

A safety and reliability release. Apogee, pyro and landing decisions have been reworked, flight logs are more complete and more robust, and several bugs that could lose data or settings are fixed.

> [!WARNING]
> **Copy your flight logs off the board before updating.** The flash filesystem grows from 1 MB to 2 MB, and the first boot after the update reformats it. All logs and `Settings.ini` are erased. Re-check your settings afterwards.

## Safety

- **New setting `MIN_PYRO_FIRE_HEIGHT_M` (default 5 m).** The pyro never fires unless the rocket climbed at least this high above the launch point. If a bump while carrying an armed rocket is mistaken for a launch, the charge still cannot fire near people. Setting `0` disables this protection; treat it as a dangerous setting.
- **The pyro never fires on an unreliable barometer.** Parachute timing depends on the barometer, so a missing, stale or rejected reading blocks deployment.
- **`FIRE_PYRO_APOGEE=FALSE` is now respected.** Previously the pyro still fired at apogee with this setting.
- **No deployment on the ground or in a tree.** Apogee now requires free fall on the IMU (≤ 0.5 g). A rocket resting, carried or swinging reads about 1 g, so it can't trigger.
- **`PYROTEST` is ground-only.** It works only in `Idle` before launch, with double confirmation, and can no longer fire during a flight. A test no longer uses up the flight pyro.
- **Invalid settings are rejected.** Non-numeric, out-of-range, `nan` or `inf` values fall back to the safe default and the file is rewritten. Previously a typo could silently disable launch detection. An unrecognized `FIRE_PYRO_APOGEE` value is treated as `FALSE`.

## Flight logic

- **New `Descent` state** (replaces `PyroFired`): Idle → Boost → Coast → **Descent** → Landed. Apogee is always recorded, also when the pyro is disabled or blocked.
- **Apogee detection rebuilt.** The primary trigger is the end of the climb, with a drop-from-peak backup and a peak-hold backup. The `ESTIMATED_*` settings are now a real baseline: near the expected apogee, confirmation is faster. They never block apogee on flights far below or far above the estimate.
- **Altitude estimate with less lag.** Single light filter (about 50 ms behind instead of about 0.4 s) and a velocity slope over 0.4 s.
- **Barometer spike filter.** Physically impossible jumps, such as the ejection-charge pressure spike, are rejected before they reach the altitude estimate. The limit scales with your speed settings. Also fixes the estimate running away during a barometer dropout.
- **Landing detection uses both sensors.** Landing is declared when one sensor is fully calm and the other moves at most slightly, so wind rocking still lands. A rocket still swinging, for example in a tree, waits until it settles. Free fall is no longer mistaken for "lying still".
- **Landing timeout** that scales with the real peak height and keeps waiting while the barometer still shows a descent, so the log is always saved.
- **No flash writes during a normal flight.** The flight number is prepared on the ground. Launch no longer rewrites `Settings.ini` during boost, and the sample loop no longer bursts after a stall.

## Flight logs

- **New header fields:** `DETECTED_APOGEE_M`, `DETECTED_APOGEE_MS`, `APOGEE_TRIGGER`, `LAUNCH_ALTITUDE_M`, `MIN_PYRO_FIRE_HEIGHT_M`, `PYRO_BLOCKED_REASON`, `LANDING_TRIGGER`, `BARO_REJECTED_SAMPLES`, `SAMPLE_GAPS`, `PYRO_AT_APOGEE_ENABLED` and `RAM_DROPPED_SAMPLES`.
- **New CSV column `baro_rejected`**, added at the end. It marks samples where the spike filter intervened.
- **No more saturation:** altitude and vertical speed are stored as 32-bit values. Previously they capped at 327 m and 327 m/s.
- **Longer recording:** the RAM buffer grows from 2048 to 6144 samples (about 102 s to about 307 s at 20 Hz). Longer flights continue streaming to flash during the descent; those logs end with `FINAL_` summary lines.
- **Storage-full handling.** Free space is checked before writing. Samples that don't fit are counted in the log and in `Settings.ini` and never cut off silently. The firmware never deletes your files.
- **Summary numbers fixed.** Float values in log headers, `Settings.ini` and serial output were printed empty; `printf` float support is now linked in.

## Settings.ini and device profile

- **Flight records now persist.** `MAX_FLIGHT_ALTITUDE_M`, `MAX_FLIGHT_SPEED_MPS`, `LAST_FLIGHT_ALTITUDE_M` and `LAST_FLIGHT_SPEED_MPS` previously failed their checksum and were reset on every boot.
- **New `[storage]` section:** `STORAGE_TOTAL_KB`, `STORAGE_FREE_KB`, `ESTIMATED_LOG_SECONDS_FREE` and `LAST_LOG_STATUS`.
- **New `OTP_HARDWARE_MATCHES_BUILD` field.** A board's OTP identity stays valid when firmware is built for a newer hardware revision.
- **`LOG_COUNT`** now reflects the number of flight logs on the board.

## Fixes

- Log cleanup could delete the wrong file, for example the real `/Settings.ini`. The firmware no longer deletes files in `/logs`.
- The upload script only probes the upload port and Raspberry Pi USB devices. It no longer sends `DUMP_OTP` to unrelated serial devices.

## Build and repository

- Platform, Arduino core and libraries are pinned to exact versions for reproducible builds. The unused ICM-42688 library dependency is removed.
- `include/build_info.h` and `.pio/` are no longer tracked in git. The header is generated on every build.
- New documentation: [Flight Log Storage](https://github.com/YoupSpace/AstroNav-Nano/blob/main/SOFTWARE_DESIGN.md#flight-log-storage) (how to adjust RAM and flash sizes, and the reboot symptoms if they are too large), an updated [user manual](https://github.com/YoupSpace/AstroNav-Nano/blob/main/ASTRONAV_NANO_USER_MANUAL.md) and a new [README](https://github.com/YoupSpace/AstroNav-Nano/blob/main/README.md).

## Upgrade notes

1. **Back up `/logs` first.** The filesystem is reformatted on the first boot.
2. After the first boot, set your values in `[settings]` again and review `MIN_PYRO_FIRE_HEIGHT_M`.
3. Flight records start at `0.0`; the stored profile resets once because of the new checksum format.
4. **Tools that read the CSV:** the `state` column now shows `descent` instead of `pyro`. A `baro_rejected` column is appended at the end. Existing columns are unchanged.
5. Do a ground test (`PYROTEST` with an LED or meter, no charge) and a toss test before flying. Check `APOGEE_TRIGGER`, `PYRO_BLOCKED_REASON` and `LANDING_TRIGGER` in the log.

Documentation: [wiki.youpspace.com](https://wiki.youpspace.com/) · [Safety](https://wiki.youpspace.com/safety/) · [Firmware update](https://wiki.youpspace.com/firmware-update/) · [youpspace.com](https://youpspace.com/)
