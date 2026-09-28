# Build and commission the base-drone firmware

## Rebuild on Linux x86-64

```bash
bash firmware/setup.sh
bash firmware/build.sh
```

Use ArduCopter **4.7.1**, revision `dbe792162d06cab66c3475fd5556bf7a120f119e`.
The setup script downloads the source/submodules, installs Python dependencies in a local
virtual environment and installs the ArduPilot GCC 10 toolchain locally. The host needs
Python 3 with venv support, Git, curl, tar, Make and a C/C++ compiler for SITL. No system
packages are changed. Generated dependencies and binaries are excluded from this repository.

The build embeds `scripts/base_drone.lua` in ROMFS. This uses the physical odd-lift/even-lateral mapping recorded in
[HARDWARE-MIXER.md](HARDWARE-MIXER.md), sets forward thrust routing off and
lateral routing on immediately, and reasserts those settings every 100 ms.
The upstream C++ patches add the Puya onboard-flash ID/capacity and enforce
nonnegative collective-lift allocation with rotational saturation feedback.
`puya-nor.patch` and `lift-motor-limits.patch` record these changes. Follow
[USB-UPDATE.md](USB-UPDATE.md) to install the latter while retaining calibration.
This firmware uses **ALT_HOLD (2)** for initial
commissioning; it does not include the lost custom ACCEL mode (30).

## Check the package

See [VALIDATION.md](VALIDATION.md) for the completed build and boot checks and their scope.

`build.sh` collects the board outputs in `firmware/dist/base-drone-StellarH7V2/` with a
SHA-256 manifest and checks the compressed APJ payload against the raw binary.

For the disarmed simulator boot check:

```bash
bash firmware/build.sh sitl
firmware/.venv/bin/python firmware/check_boot.py
```

The SITL build uses debug optimisation and disables global warnings-as-errors plus the `maybe-uninitialized` error promotion for host GCC 16; the board
build uses the pinned ARM GCC 10 with its normal warning policy. The boot check uses a
stock quad plant only to boot the firmware, read back parameters, check that the embedded
script remains running, and select ALT_HOLD/LAND while disarmed. It never arms and does
not validate the custom airframe's dynamics or airborne failsafes.

## Flash the Stellar H7 V2

1. Remove propellers. Save the existing Betaflight configuration before replacing it.
2. For the initial Betaflight-to-ArduPilot installation, use the generated
   `arducopter_with_bl.hex` with a compatible STM32 DFU flashing tool. Follow the
   [board instructions](https://ardupilot.org/copter/docs/common-stellarh7v2.html) and
   [initial installation procedure](https://ardupilot.org/copter/docs/common-loading-firmware-onto-chibios-only-boards.html).
3. For subsequent updates through the ArduPilot bootloader, load `arducopter.apj` using
   the ground station's custom firmware option.
4. Load `base-drone.parm`, reboot, and read the parameters back. This file is specific to
   4.7.1: several parameter names and units differ from the old SITL configuration.
5. Confirm the frame is `BaseDrone Hardware 6DoF`, all eight motor functions exist, scripting is
   enabled, and no script errors appear. `SCR_DIR_DISABLE=2` excludes SD-card scripts;
   the embedded mixer is the only script this package needs.
6. Confirm `SCR_LD_CHECKSUM` and `SCR_RUN_CHECKSUM` both equal `934390`, as supplied
   in the parameter file. Reboot and verify script pre-arm checks pass. Recompute these
   checksums through a new boot check whenever you change the embedded mixer.
7. After flashing, verify the attitude display against hand tilts before accepting orientation.
   Betaflight board alignment was zero; `AHRS_ORIENTATION=0` is the starting assumption.
   Calibrate accelerometers, configure the estimator for the installed
   sensors, calibrate battery sensing, and establish motor idle and hover settings on hardware.
   Resolve every pre-arm failure; retain `ARMING_SKIPCHK=0`.

## Logging storage on this physical board

The board build applies `stellar-flash.hwdef` through waf's `--extra-hwdef` option.
It selects onboard NOR flash with LittleFS. `puya-nor.patch` adds the Puya
PY25Q128HA ID `0x852018` to the existing NOR filesystem driver, using 256-byte
pages and 256 x 64-KiB erase blocks (16 MiB). The overlay uses the original SPI
pins and a 20-MHz maximum clock for initial validation.
The stock target selects W25NXX NAND and discards the first ID byte, explaining
why the NOR chip's repeating ID appears as `0x201885`. This identification is
inferred from the observed bytes and the manufacturer datasheet; physical boot
must confirm `0x852018`. Unsupported IDs still fail rather than guessing capacity.
See the [Puya datasheet](https://www.puyasemi.com/download_path/%E6%95%B0%E6%8D%AE%E6%89%8B%E5%86%8C/Flash/PY25Q128HA_Datasheet_V2.0.pdf),
sections 5.3, 10.1 and 10.40, for clocks, commands and ID.
LittleFS automatically formats flash if it cannot mount an existing filesystem;
this can replace old Betaflight blackbox logs on that chip.
Flight parameters and accelerometer calibration use separate MCU flash storage.
When updating this FC, back up its live parameters first and do not reload the
starting parameter file over the calibrated battery and sensor settings.

To install the onboard-flash update from the repository directory:

1. With props removed, battery disconnected and MAVProxy connected over USB, save
   the live configuration: `param save firmware/logs/before-flash-update.parm`.
2. Exit MAVProxy with Ctrl+C so it releases the USB connection. Run in the Fedora
   terminal:

   ```bash
   firmware/.venv/bin/python firmware/ardupilot/Tools/scripts/uploader.py --port '/dev/ttyACM*' --baud-flightstack 115200 firmware/dist/base-drone-StellarH7V2/arducopter.apj
   ```

   This asks the FC to enter its existing ArduPilot bootloader, checks its board ID,
   uploads and verifies the application, then reboots it. Do not use force/erase
   options. If it cannot identify the board, stop and inspect the output.
3. Reconnect MAVProxy using the port that exists after reboot. Do not reload
   `base-drone.parm`: the application update should retain live settings and calibration.
4. Run `param set LOG_DISARMED 1`, wait 20 seconds, then run
   `param set LOG_DISARMED 0`. Power-cycle with USB and reconnect.
5. Run `module load log`, then `log list`. Download the new nonempty log with
   `log download NUMBER firmware/logs/onboard-flash-check.bin`, replacing NUMBER with
   the ID in the list. Completion of this download is the storage acceptance check.

## Verify the hardware output protocol

The pinned six-axis Lua motor initialiser does not reapply the output rate/mode
once the script adds its motors. `MOT_PWM_TYPE=5` alone can leave the hardware
banner reporting PWM. The parameter file explicitly sets `SERVO_BLH_MASK=255`
(outputs 1-8) and `SERVO_BLH_OTYPE=5` (DShot300). No firmware reflash is needed
when adding these settings to an existing installation. Reboot and confirm the
hardware banner reports DShot300 on outputs 1-8 before powered motor tests.
Outputs 9-10 remain unused. This does not change saved ESC rotation directions.
A simulator parameter readback cannot verify the physical output protocol.

## Verify controls with props removed

For the later props-on stage, [LIFT-TEST.md](LIFT-TEST.md) describes a separate
supervised tethered STABILIZE test with Up/Down throttle controls and Space to stop.
It is not a free-flight or autonomous takeoff interface.

Use [BENCH-TEST.md](BENCH-TEST.md) for the timed hardware test tool. Its default
invocation only displays status; a named powered test requires typing `RUN`.
The `rc-loss` and `link-loss` tests check ground failsafe disarming and heartbeat
recovery. They do not establish airborne LAND behaviour or isolate GCS action
while armed when the RC timeout acts first.

Use MAVLink RC overrides from source system ID **255**, sent continuously at 20 Hz or more.
Send channels 1–4 together. Keep channels 1, 2 and 4 at 1500 when neutral. Channel 3 must
start at 1000 before arming; in ALT_HOLD, its centre commands zero climb rate. Select
ALT_HOLD explicitly and confirm the reported mode before arming. Increasing channel 3
above its centre requests takeoff/climb. Avoid a step to a high throttle command.

| Channel | Command | Neutral |
|---|---|---|
| 1 | Lateral thrust, while keeping roll level | 1500 |
| 2 | Fore/aft pitch | 1500 |
| 3 | Climb/descent in ALT_HOLD | 1500 in flight; 1000 before arming |
| 4 | Yaw rate | 1500 |

The parameter file limits the equivalent lean command to 10 degrees, climb to 0.5 m/s,
and descent to 0.3 m/s. These limits do not establish safe hardware gains or a current limit.
Neutral lateral input does not brake drift. No horizontal position sensor is listed in the
hardware plan. Landing without one cannot hold horizontal position.

1. Test each motor individually and record its physical position and rotation. Outputs
   1–8 map to motor functions 1–8; motors 1/3/5/7 provide lift, 2/4/6/8 provide lateral thrust.
2. Check positive (right) lateral demand favours M4/M6; negative demand favours M2/M8.
   Verify roll, pitch and yaw corrections against the physical motor diagram.
3. Test LAND (9) and confirm the pilot's lateral input cannot reposition the landing aircraft.
   Lateral motors may still produce attitude corrections; LAND does not imply all side motors stop.
4. Stop overrides while continuing GCS heartbeats. Confirm RC failsafe and LAND.
5. Stop all GCS traffic. Confirm GCS failsafe and LAND. Reconnection must not re-arm or
   resume manual flight automatically.
6. Test low-battery responses, disarm, reboot, log recording and log retrieval.
7. Complete the loaded tests in [the flight checklist](../Base-Drone-Flight-Checklist.md)
   before free flight.

Overrides expire after 0.5 s; RC loss timeout is 1 s. GCS timeout is 2 s. Actual transition
timing must be measured. The control application must stop overrides when operator input
stops being fresh; a background thread continuously resending stale inputs defeats this test.
Do not use the old SITL `drive.py`, parameter dumps, force-arm commands or rig scripts on hardware.

## Resolve before flight

- Verify the physical geometry against the inherited mixer factors, including lateral motor
  pitch/yaw contributions. The simulation factors are not measurements of the assembled frame.
- Measure motor idle, thrust response, vibration and hover demand; tune using hardware logs.
- Verify estimator behaviour with no GPS or compass installed. This package does not claim
  horizontal position hold or supply fictitious navigation data.
- Finish and test the laptop control interface; this package contains firmware and configuration.
- Measure command-loss and mode-change behaviour on the actual link and FC.
- Calibrate battery voltage before enabling battery-based flight decisions: the user measured
  22.9 V at the ESC while Betaflight displayed 12.05 V. The cause remains unresolved.
  This commissioning parameter file uses voltage-only monitoring. No Hall/ESC current scale
  is assumed; verify independent FC ADC inputs or a supported ESC telemetry route before
  enabling total-current monitoring. Verify battery thresholds under load afterwards.
- Establish electrical/output limits through restrained tests. In this upstream revision,
  `AP_MotorsMatrix_6DoF_Scripting::output_armed_stabilizing()` does not call
  `get_current_limit_max_throttle()`. `MOT_BAT_CURR_MAX` is not a verified current ceiling here.

Build success establishes that the firmware compiles for this board. It does not establish
that this aircraft is ready for free flight.
