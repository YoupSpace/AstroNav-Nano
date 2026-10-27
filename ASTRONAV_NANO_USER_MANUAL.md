# AstroNav Nano User Manual

This manual describes the complete user workflow for the AstroNav Nano: first setup, USB configuration, pre-flight checks, flight operation, recovery, and post-flight review.

The firmware and hardware are safety-critical. Read the safety section before connecting a battery, payload, pyro circuit, servo, or launch hardware. This manual explains how the device works; it does not replace your launch provider's procedures, applicable law, an electrical inspection, or a flight-readiness review.

## 1. What the device does

AstroNav Nano is a flight controller and flight-data logger. At startup it:

1. Initializes the processor, status LED, storage, IMU, barometer, and voltage sensing.
2. Holds the pyro and servo outputs low so they do not float during startup.
3. Runs health checks and measures its input voltage.
4. Selects USB configuration mode when USB power is detected, or flight mode when a supported battery source is detected.
5. Samples acceleration, rotation, temperature, pressure, altitude, and vertical velocity during operation.
6. Detects launch, boost, coast, apogee, and landing using the available sensor data.
7. Fires the pyro output at apogee only when the relevant setting and safety logic allow it.
8. Saves a CSV mission log and updates the device profile after landing.

AstroNav Nano works with the separate AstroNav MC companion software, but the board can also be configured by editing the files exposed over USB.

## 2. Safety before anything else

Follow these rules every time:

- Treat the pyro output as capable of ignition whenever the board is powered.
- Never connect an igniter, deployment charge, motor initiator, or other hazardous load while configuring or testing the board.
- Do not use `PYROTEST` with a live pyro circuit. It can energize the output.
- Keep the device physically safe and disconnected from launch hardware while building, flashing, calibrating, and troubleshooting.
- Do not change a flight setting unless you understand its effect and have reviewed it in a controlled test plan.
- Inspect wiring, polarity, battery condition, connectors, mounting, and the output inhibit strategy before flight.
- A green-looking or non-fault state is not permission to launch. The operator remains responsible for the complete flight-readiness decision.

If anything is unexpected, disconnect hazardous hardware, remove power when safe, and investigate before continuing.

## 3. Skill levels and feature labels

This manual uses these labels:

- **Basic:** required for normal setup and ordinary use. A first-time user should stay here.
- **Advanced:** useful for experienced users, configuration tuning, and data review. Use it only after the basic workflow is understood.
- **Pro (optional):** useful but not required for normal operation. Pro features add control, diagnostics, manufacturing metadata, or test capability. They should be enabled only when the operator has a specific reason and a suitable test procedure.
- **Dangerous test:** not a normal user feature. It may energize hardware and requires a controlled bench procedure with all hazardous loads disconnected.

The core device remains useful without the pro features. Optional does not mean risk-free.

## 4. First setup

### Basic: install the development tools

For firmware builds or uploads, install:

1. VS Code.
2. The PlatformIO IDE extension.
3. Git, with `C:\Program Files\Git\cmd` available on `PATH` on Windows.

Open the project folder in VS Code. Build it with the PlatformIO Build action or:

```powershell
pio run
```

Upload only to the intended board and port:

```powershell
pio run --target upload --upload-port COM3
```

Replace `COM3` with the port shown for your board. Firmware development and upload are technical tasks; a successful build does not prove that the assembled hardware is safe to fly.

### Basic: connect the board over USB

1. Keep pyro, servo, and other hazardous outputs disconnected.
2. Connect the board to the computer with USB.
3. Allow startup health checks to finish.
4. Open the newly exposed `AstroNav Nano` drive.
5. Read `Howto.txt` and inspect `Settings.ini` before changing anything.

The board enters USB mass-storage mode only when it classifies its input as USB power, approximately 4.5 V to 5.5 V. A battery voltage outside the supported ranges is reported as `Unknown` and may create a warning or fault.

## 5. Configure the board

### Basic: edit `Settings.ini`

`Settings.ini` is an INI-style text file. Edit only the values in its `[settings]` section and preserve the key names. The available settings are:

| Setting | Meaning | Default |
| --- | --- | ---: |
| `ESTIMATED_HEIGHT_M` | Expected apogee height in metres. Near this height apogee is confirmed faster. | `120.0` |
| `HEIGHT_MARGIN_M` | Height tolerance around the estimate. | `20.0` |
| `ESTIMATED_SPEED_MPS` | Expected top speed in metres per second; used to reject impossible barometer jumps. | `35.0` |
| `SPEED_MARGIN_MPS` | Speed tolerance around the estimate. | `8.0` |
| `LAUNCH_THRESHOLD_G` | Acceleration threshold used to confirm launch. | `1.35` |
| `FIRE_PYRO_APOGEE` | Allows (`TRUE`) or blocks (`FALSE`) the automatic apogee pyro pulse. | `TRUE` |
| `MIN_PYRO_FIRE_HEIGHT_M` | **Dangerous setting.** The pyro never fires unless the rocket climbed at least this many metres above the launch point. `0` disables this protection. | `5.0` |

The estimate and margin settings are a **baseline**, not a limit: accurate values let the flight controller react faster and more precisely, but apogee and landing are still detected when the real flight is much lower or much higher than estimated.

`MIN_PYRO_FIRE_HEIGHT_M` protects people handling an armed board: if a bump while carrying the rocket is mistaken for a launch, the pyro still cannot fire close to the ground. Only lower it if you fully understand the consequence. Independently of this setting, the pyro never fires when the barometer is unreliable, because parachute timing depends on it.

Recommended beginner procedure:

1. Start with the generated defaults.
2. Change only values supported by your test plan.
3. Save the file.
4. Safely eject the drive in Windows.
5. Reconnect over USB and confirm that the values are still present.

`FIRE_PYRO_APOGEE=FALSE` is useful for sensor and detection tests: apogee can still be detected and logged, but the automatic pyro pulse is blocked. It is not a substitute for physically safe wiring.

The `[debug]` and `[checks]` sections are diagnostic snapshots. Do not edit them as a normal configuration step. The `[profile]` section contains device identity and stored flight history.

### Advanced: use AstroNav MC

AstroNav MC can provide a friendlier interface for editing settings, reading logs, inspecting flight summaries, and simulating or validating mission behavior. It complements the onboard firmware; it does not replace the onboard safety logic or the operator's checks.

### Pro (optional): production identity and build metadata

Production workflows may use `generate_version.py`, `include/build_info.h`, a warranty key, a serial registry, and the board's OTP identity data. These features can provide traceability, a serial number, firmware metadata, and an official warranty signature.

They are not needed for ordinary local builds or normal configuration. Keep production secrets outside the repository, and use this workflow only for authorized manufacturing or service work.

## 6. Understand the status and state flow

The firmware moves through controlled states rather than jumping directly to output activity:

1. **Booting:** hardware and storage initialization.
2. **USB mode:** the drive is available for configuration and log access.
3. **Calibrating:** the board must remain still while it establishes gravity, gyro bias, and baseline pressure.
4. **Idle:** the board is ready and waiting for launch conditions.
5. **Boost:** sustained acceleration indicates launch and powered ascent.
6. **Coast:** boost has ended and the vehicle is still in flight.
7. **Descent:** apogee was detected. This state is always entered at apogee, also when the pyro is disabled or blocked; whether the pyro fired is shown in the log's `pyro` column and `PYRO_BLOCKED_REASON`.
8. **Landed:** the vehicle has remained settled long enough to confirm landing.
9. **Fault:** a critical issue has latched the system safe and disables the pyro output.

The status LED communicates broad conditions. Startup is busy/white, calibration has its own status, USB and idle modes have distinct idle indications, and a fault is red and blinking rapidly. Always use the USB debug snapshot and serial output to investigate a warning; do not infer detailed health from the LED alone.

## 7. Pre-flight procedure

### Basic: safe readiness check

1. Disconnect all hazardous output loads.
2. Confirm the current settings and intended `FIRE_PYRO_APOGEE` value.
3. Confirm the battery type and voltage are within a supported range: 1S LiPo, 2S LiPo, or the intended USB configuration source.
4. Mount the board firmly in the intended orientation.
5. Confirm that the sensor area is not obstructed and that the board can remain still during calibration.
6. Power the board and watch startup health behavior.
7. Leave it still while calibration runs.
8. Confirm that it reaches `Idle` without a critical fault.
9. Review the wiring and launch authorization separately from the board status.

If calibration times out, the board was moved, the sensors are not healthy, or the power source is unknown, stop and correct the cause. Do not treat a forced or improvised workaround as readiness.

### Advanced: review health data

Over USB, inspect the `[debug]` and `[checks]` sections. Useful fields include `FAULT_REASON`, `POWER_MODE`, `STATE`, `VIN_VOLTAGE`, `HEALTHY`, `IMU_STREAM_OK`, `BARO_OK`, `FLASH_FS_OK`, and `CRITICAL`. Serial monitoring uses 115200 baud:

```powershell
pio device monitor
```

Resolve critical conditions before connecting any hazardous load.

## 8. What happens during flight

During flight the board samples sensors on a regular schedule and estimates altitude and vertical velocity from pressure, while using the IMU for acceleration, rotation, launch confirmation, and orientation-related checks.

Launch requires sustained acceleration above the configured threshold. Between launch and landing the board does not write to flash, so nothing can stall the sensor loop during the flight.

During boost and coast, apogee requires a trustworthy barometer, free fall on the IMU (or a real barometric descent when the IMU is unavailable), and a minimum flight time. It triggers when the climb has stopped (velocity) or, as a backup, when the drop from the peak is clearly larger than noise. Confirmation samples are required rather than acting on one noisy reading. Physically impossible barometer jumps (for example a pressure spike from the ejection charge) are rejected before they reach the altitude estimate.

When apogee is confirmed the board enters `Descent`, and:

- with `FIRE_PYRO_APOGEE=TRUE`, the pyro output receives a one-second pulse if the barometer is reliable and the peak is at least `MIN_PYRO_FIRE_HEIGHT_M` above the launch point;
- with `FIRE_PYRO_APOGEE=FALSE`, the event is logged but the output is not fired.

`Landed` is declared when one sensor is completely calm and the other shows at most minimal movement (for example wind rocking). A rocket still swinging, for example in a tree, keeps waiting until it settles. A timeout that scales with the real peak height guarantees the log is always saved. The flight log header records what the board decided: `DETECTED_APOGEE_M`, `APOGEE_TRIGGER`, `PYRO_BLOCKED_REASON`, `LANDING_TRIGGER`, `BARO_REJECTED_SAMPLES` and `SAMPLE_GAPS`.

Sensor fallback can allow healthy sensor paths to continue supporting state logic when another reading is invalid, but fallback behavior is not a reason to ignore warnings or continue an unsafe operation.

## 9. Finish the session and retrieve data

### Basic: leave USB mode correctly

When the board is powered by USB:

1. Finish editing and close files using the drive.
2. Use Windows **Safely Remove Hardware / Eject**.
3. Wait for the board to leave storage mode before changing power or wiring.

The alternative `EXITUSB` serial command can request the same transition. Never unplug during an active write when a safe eject is available.

### Basic: retrieve a flight log

After the flight has landed and the board has finished saving:

1. Remove power only according to your local recovery procedure.
2. Reconnect the board over USB with hazardous loads disconnected.
3. Open the `logs` folder.
4. Copy the relevant `Flight_XXXXXXXX.csv` file to your mission records.
5. Review state changes, health bits, altitude, velocity, acceleration, pressure, temperature, roll, pitch, and pyro status.

Do not modify the original log. Work on a copy for analysis.

### Advanced: interpret a CSV log

The log includes a sample counter and time in milliseconds, power mode, flight state, pyro flag, sensor values, derived altitude and velocity, orientation, and health bits. Compare the recorded state transitions with independent mission data and note any warning or fault condition before drawing conclusions.

## 10. Optional pro features and test controls

| Feature | Label | Why use it | Required caution |
| --- | --- | --- | --- |
| AstroNav MC log review and simulation | Pro (optional) | Easier configuration, summaries, and repeatable behavior checks. | It does not replace physical inspection or onboard safety logic. |
| Production serial, OTP identity, and warranty signature | Pro (optional) | Traceability and authorized manufacturing/service validation. | Requires controlled production assets and should not expose secrets. |
| Serial health monitoring | Advanced / Pro (optional) | Helps diagnose power, sensor, storage, and state behavior. | Use 115200 baud and keep hazardous outputs disconnected. |
| `FIRE_PYRO_APOGEE=FALSE` detection-only tests | Pro (optional) | Tests detection and logging without automatic apogee output. | Still treat the board as energized and verify the physical circuit. |
| `PYROTEST` | Dangerous test | Bench verification of the pyro output path. | Never connect live hardware. Send `PYROTEST` twice within five seconds; the output pulse is limited to one second. |

`PYROTEST` is a ground-only test: it is accepted only in `Idle` (flight mode, calibrated, before any launch), never in USB storage mode and never during or after a flight. It does not use up the flight pyro, so after replacing the igniter the board can still deploy on the next flight without a reboot. It is deliberately not part of the normal beginner workflow.

## 11. Troubleshooting

### The USB drive does not appear

- Confirm the board is actually powered by USB.
- Check that the measured input is in the USB range.
- Reconnect with hazardous outputs disconnected.
- Check serial output and the USB/storage health fields.
- Try a different cable or port if the computer does not detect the board.

### The board reports an unknown power source

Check battery voltage, polarity, wiring, and the voltage-sense path. Supported classifications are USB 4.5-5.5 V, 1S LiPo 3.0-4.35 V, and 2S LiPo 6.0-8.4 V. Do not continue with an unknown source until it is understood.

### Calibration fails

Place the board on a stable surface, stop vibration, check IMU and barometer connections, and retry. If it still fails, inspect `IMU_STREAM_OK`, `BARO_OK`, and the serial fault output.

### The board is red and blinking

This indicates a latched fault condition. The pyro output is disabled safely. Keep hazardous hardware disconnected, read `FAULT_REASON` and the health checks, and correct the underlying problem before rebooting or retesting.

### The board reboots or freezes after a firmware update

If this started after the flight-log storage was changed (RAM log buffer or flash filesystem size), the board may be running out of memory. See "Flight Log Storage" in [SOFTWARE_DESIGN.md](SOFTWARE_DESIGN.md) and reduce the RAM log buffer. Note that changing the filesystem size erases all logs and `Settings.ini` on the first boot, so copy your logs off the board before such an update.

### Settings do not seem to change

Confirm the key spelling and value format, save the file, safely eject the drive, and reconnect or reboot. Change only the `[settings]` values; diagnostic sections are generated by the firmware.

### A log is missing

Wait for the landing/save process to complete, then reconnect over USB and inspect `/logs`. Check flash filesystem health and serial output. Do not remove power during a write.

Check the `[storage]` section of `Settings.ini`: `LAST_LOG_STATUS=truncated_storage_full` means the flash was full and the end of the flight could not be saved (the log header then says how many samples were dropped). `STORAGE_FREE_KB` and `ESTIMATED_LOG_SECONDS_FREE` show how much room is left; copy old logs off the board and delete them to make room. The firmware never deletes your files itself.

### Settings were reset to a default value

If a value in `[settings]` is not a valid number, is out of range, or is `nan`/`inf`, the board ignores it, uses the safe default and rewrites the file so you can see what is active. An unrecognized `FIRE_PYRO_APOGEE` value is treated as `FALSE`. The serial output names the rejected key.

## 12. One-page operating checklist

### Before configuration

- [ ] Hazardous outputs disconnected
- [ ] Correct board and firmware identified
- [ ] USB power connected
- [ ] `Howto.txt` and `Settings.ini` reviewed

### Before flight

- [ ] Settings reviewed and tested
- [ ] Battery and wiring inspected
- [ ] Board mounted securely
- [ ] Calibration completed while stationary
- [ ] No critical fault
- [ ] Independent flight-readiness review completed

### After flight

- [ ] Landing confirmed
- [ ] Mission log saved
- [ ] Power removed safely
- [ ] CSV log copied and preserved
- [ ] Device warnings, faults, and pyro status reviewed

## 13. Source of truth

This guide describes the current intended user workflow. For exact implementation behavior, the firmware source in `src/`, the generated USB `Howto.txt`, and the project `README.md` remain the source of truth. Update this manual whenever user-visible settings, state transitions, output behavior, or file names change.