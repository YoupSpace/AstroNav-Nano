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
