# Base-drone firmware validation — Measured geometry update — November 2026

## Lift-motor lower limits — 11 November 2026

`lift-motor-limits.patch` applies a zero lower bound to non-reversible motors
with positive collective factors. Both rotational and horizontal allocation
respect that bound. Rotational scaling sets the existing roll/pitch/yaw limit
flags consumed by the rate integrators. Zero-collective lateral motors retain
signed intermediate requests so opposite-direction selection still works.
Reversible motors retain their existing signed bounds.

The host regression executes the production allocator, Euler rotation function
and PID integral update with stubbed hardware/state interfaces. It checks both
signs of low-throttle roll/pitch, preservation of low-demand allocation and
lateral selection, horizontal correction bounds, zero-throttle limits,
integral growth suppression and unwind, reversible compatibility, and 20,000
mixed commands with roll/pitch offsets up to 45 degrees. It passes; the original
upstream allocator fails the low-throttle regression assertion.

This is not a closed-loop aircraft simulation. The patch does not eliminate
integral growth below saturation when ground contact or a tether prevents
movement. It does not establish the cause of the previous liftoff motion.
No changes were made to calibration, rate gains, idle, Lua geometry, controls,
current monitoring or failsafe actions. The hardware startup identity is
`BaseDrone 4.7.1 liftlimits1`. See `USB-UPDATE.md` for installation and verification.

Board and SITL builds passed. The board uses 1,517,772 bytes of flash with
186,164 bytes free. APJ/raw/HEX image checks, exact-source-patch checks and
firmware-identity checks passed. Existing geometry and NOR identification checks
and all 26 bench/lift-tool unit tests passed. The rebuilt SITL passed the
disarmed boot check: 66 parameters, loaded/running Lua checksums `934390`,
and ALT_HOLD/LAND mode selection. Hardware-only DShot parameters still require
FC readback; the simulator never armed.

Learning backlog: lift-motor saturation feedback and the distinction between
signed lateral selection and impossible negative lift were applied at the user's
request; their detailed derivation has not been worked through together.

## Compiled attitude-response check — 11 November 2026

`tests/check_attitude_response.py` runs the compiled patched SITL against a
prescribed-attitude JSON fixture on localhost. It uses the embedded Lua mixer,
STABILIZE, saved ATC/MOT settings, neutral steering and throttle input 1300.
Simulator-truth AHRS supplies known angles; the fixture ignores motor forces.
Only the disposable simulator receives forced arming and disabled failsafe
actions. No FC connection or hardware parameter writes occur.

All six static-error tests passed. After establishing a level target, the fixture
steps each axis by 5 degrees. Outputs are averaged 0.4–0.8 seconds after the step.
Positive roll raises M3/M5 relative to M1/M7; negative roll reverses this.
Nose-down raises front lift relative to rear; nose-up reverses this.
Clockwise heading error favours M2/M6 for anticlockwise yaw, and the opposite
heading error favours M4/M8. These yaw cases test heading error, not imposed
angular velocity. Reports and console output are in `logs/attitude-response/`
and `logs/attitude-response-console.log`.

This executes the attitude-to-rate, rate-to-torque, allocation and output path.
It does not validate EKF estimation, physical wiring, propeller direction,
realised thrust, hover dynamics or the cause of the earlier liftoff movement.
The first harness run contained duplicate simulator defaults and triggered a
battery failsafe; the corrected harness emits unique parameter names and checks
armed state and running lift outputs before accepting each stimulus.

## Geometry and original bring-up record

| Check | Result |
|---|---|
| ArduCopter source | 4.7.1, `dbe792162d06cab66c3475fd5556bf7a120f119e`; Puya NOR patch plus `lift-motor-limits.patch` |
| Output mapping | Odd outputs lift, even outputs lateral; matches supplied Betaflight MCU output pins |
| Rotations | M1/M2/M5/M6 CCW; M3/M4/M7/M8 CW, viewed from outward shaft end |
| Lateral force | User confirmed left motors push aircraft left, right motors push right |
| Allocation sign checks | Pure-axis signs and cancellation, offset-CoM collective balance, and bounded mixed lateral/yaw coupling pass at level and low outputs |
| StellarH7V2 build | Passed with ARM GCC 10 and embedded hardware mixer |
| Flash usage | Onboard NOR build: 1,517,552 bytes reported by linker; 186,380 bytes free |
| Image integrity | APJ decompression matches raw binary and application bytes in bootloader-inclusive HEX |
| Board identity | APJ board ID 1503; HEX starts at `0x08000000`, application at `0x08020000` |
| Disarmed SITL boot | Passed; 66 supported parameters passed again; SERVO_BLH_MASK/OTYPE are absent from this SITL target and await hardware confirmation |
| Embedded script | Loaded and running checksums both `934390` |
| Mode selection | ALT_HOLD (2) and LAND (9) accepted while disarmed |

Mass 1.75 kg is recorded as user-reported. Geometry now uses the supplied
740 x 453 mm lift rectangle, 255 mm lateral fore/aft span, approximately 10 mm
forward CoM shift, 42 mm lift-hub height and negligible lateral-axis height.
Lateral width is unmeasured and not needed for purely Y-directed thrust. Propeller
reaction torque/thrust coefficient remains an estimate. No inertia was required
for allocation checks, and no dynamic gain tuning was performed. Native simulation
files were not changed. See [HARDWARE-MIXER.md](HARDWARE-MIXER.md).

The analytical checks parse the emitted Lua factors and compute physical force and
reaction torque under the assumed geometry. They are not execution of the compiled
ArduPilot allocator or proof of saturated/tilted behaviour. The disarmed SITL check
used a stock quad plant only to boot firmware and read back configuration; it never
armed. Simulated GPS/compass were present. There was no hardware flashing or flight
verification in this update.

Betaflight's OCTOX8 mixer from the supplied dump is not used in the ArduPilot image.
ArduPilot applies throttle-scaled lateral allocation, unlike the native simulation's
simplified output path. The custom ACCEL mode remains deferred.

The commissioning parameters now select voltage-only battery monitoring. The Hall
sensor is not assumed installed. Independent ESC analogue current inputs are not
verified. The previously reported physical voltage/display mismatch remains unresolved.
The nominal voltage scale must be calibrated before battery-based flight decisions.

Before flight: verify attitude axes after flashing, every motor's location and spin,
actual propeller thrust direction, physical CoM/lever arms, estimator operation,
command loss/failsafe behaviour, gains, vibration and electrical/output limits.

Raw results are under `firmware/logs/`. The package and SHA-256 manifest are under
`firmware/dist/base-drone-StellarH7V2/`. This supersedes the 3 October mixer/package.

Hardware bring-up found RCOut PWM despite MOT_PWM_TYPE=5. Code inspection shows
the six-axis scripting initialiser does not reapply motor update rate after adding
motors. Commissioning parameters now explicitly select outputs 1-8 and DShot300
using SERVO_BLH_MASK=255 / SERVO_BLH_OTYPE=5. Confirm the actual hardware banner.

## Onboard flash configuration correction

The user confirmed the hardware DS300 banner, individual operation of all eight
motors, accelerometer calibration, battery voltage agreement and neutral MAVLink
RC inputs. These do not validate flight dynamics.

The upstream board definition selects W25NXX NAND. The observed ID `0x201885`
is consistent with Puya NOR ID `0x852018` read with the NAND one-byte offset.
Puya's PY25Q128HA datasheet documents that ID, 256-byte pages, 64-KiB erase blocks,
and 128-Mbit capacity. Physical chip identification remains to be confirmed by
booting the NOR driver and checking the returned ID.

`stellar-flash.hwdef` selects `littlefs:jedec_nor` and limits SPI to 20 MHz.
`puya-nor.patch` adds only the named ID and the existing 16-MiB geometry case to
`AP_Filesystem_FlashMemory_LittleFS.cpp`. The NOR patch itself makes no motor/control changes; the later lift-limit patch is separate.
Packaging requires the exact combined recorded patches, NOR LittleFS and disabled SD FAT.
The production NOR identification function passed a host test with mocked SPI:
it accepts `85 20 18`, selects exactly 16 MiB and rejects shifted/unknown IDs.
This test does not exercise electrical SPI communication, erase or program.
The SD workaround is superseded and is not the current build configuration.
Hardware write/download verification remains pending.

The onboard NOR ARM build passed. Generated definitions select NOR LittleFS and
20-MHz flash SPI, with SD FAT disabled. Packaging passed board-ID and APJ/raw/HEX
integrity checks and verified the exact two-line source patch and unchanged Lua.
