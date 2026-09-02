# Base drone: boxed parts to first flight

Use this checklist to commission the eight-motor aircraft without the pendulum assembly.
Hardware selections and the provisional wire schedule are recorded in [Hardware-BoM.md](Hardware-BoM.md).
Check actual parts and board revisions against that record before wiring.

Keep propellers off until the loaded-test stage. Calibration, motor checks and tuning require
the assembled vehicle; software, wiring plans and test procedures can be prepared beforehand.

## Priority for the next few days

Prepare the flight software for the first flight with **no pendulum fitted**. Work through
section 1 first; resolve motor geometry and output assignments from section 2 alongside it.

The software work depends on three decisions: how pilot commands reach the controller, how
controller outputs map to the eight physical motors, and what the aircraft does when commands
stop or a failsafe activates. Record these decisions before configuring the hardware.

### Software completion criteria

- [x] **Replacement source available:** pin upstream ArduCopter 4.7.1; retain the mixer in this
  repository. The lost custom ACCEL mode is deferred for the initial ALT_HOLD flight.
- [x] **Hardware firmware built:** successful `StellarH7V2` build with ALT_HOLD and the embedded
  eight-motor mixer. No pendulum controller or simulated sensor script is included.
- [ ] **Control workflow ready:** prepare the first-flight control application and document
  connection, command axes/units/limits, arming, takeoff, landing and disarming.
- [ ] **Configuration ready:** save a dedicated hardware parameter file and motor/output map;
  document settings that still need calibration on the assembled aircraft.
- [ ] **Failure behaviour tested:** record tests for stopped commands, lost link, reconnection,
  mode changes and failsafes, including lateral outputs. Distinguish simulation results from
  checks still required on hardware.
- [ ] **Deployment package ready:** collect the firmware, source revision, parameters, required
  scripts, flashing instructions and ordered props-off test procedure in one reproducible package.

Software preparation is complete when these artifacts and test results exist. Flight readiness
also requires the assembled-aircraft checks and loaded tests below. Keep unchecked items open
until their results have been recorded.

## 1. Prepare the flight software — highest priority

**Status at 3 October 2026:** downloaded ArduCopter 4.7.1 at
`dbe792162d06cab66c3475fd5556bf7a120f119e` and built `StellarH7V2` with the existing
Lua eight-motor mixer embedded. The agreed first-flight path is upstream ALT_HOLD;
the custom ACCEL mode is deferred. See [firmware/README.md](firmware/README.md) for
rebuilding, flashing and commissioning. Hardware validation remains outstanding.

- [x] Download and pin replacement ArduPilot source; the old custom checkout was lost in transfer.
- [ ] Compare the upstream six-axis mixer behaviour with the native simulation (the output paths differ).
- [x] Build for `StellarH7V2`, including scripting and the embedded motor mixer for ALT_HOLD.
- [ ] Verify the mixer against the final physical geometry and numbered motor diagram.
- [x] Exclude pendulum control, pendulum compensation and simulated sensor scripts from the hardware package.
- [ ] Define the first-flight input device, command mapping, flight mode and landing procedure.
- [ ] Verify operation with the installed sensors. The planned hardware has no horizontal position
  sensor: holding level or commanding zero acceleration does not stop existing drift.
- [ ] Verify mode changes and failsafe landing with the custom mixer, including lateral motor outputs.
- [ ] Prepare a dedicated hardware parameter file covering outputs, DShot, FC orientation, battery
  monitoring, telemetry, logging and failsafes.
- [ ] Remove simulation-only settings and force-arming from the hardware workflow.
- [ ] Implement and test stale-command handling, including a frozen control application while
  another program continues sending heartbeats.
- [ ] Verify that proposed current/output limits affect the custom motor path. Do not treat a
  configured current limit as proof of a hard electrical ceiling.
- [ ] Prepare repeatable tests for arming, disarming, command directions, mode changes, lost
  connection and reconnection.
- [x] Save a reproducible firmware package: binary, source revision, parameters, embedded Lua
  mixer and flashing instructions (`firmware/dist/base-drone-StellarH7V2/`).

## 2. Finalise the build plan

- [ ] Check every box against the hardware list, including revisions, cables and capacitors.
- [ ] Finish motor mounts, electronics mounts, battery retention, landing supports and cable routes.
- [ ] Check clearance around all eight propeller discs, allowing for wiring and frame flex.
- [ ] Make a numbered motor diagram showing each motor's physical position, thrust direction,
  rotation, propeller, ESC board/channel, FC output and software motor number.
- [ ] Draw the final pad-to-pad wiring diagram. Verify connector pinouts individually.
- [ ] Check supplies: wire, XT60, heat-shrink, fasteners, spacers, FC soft mounts, battery straps,
  microSD, solder/flux and a smoke stopper rated for 6S voltage.
- [ ] Prepare the restraint/enclosure and designated flight-test area.

## 3. Flash and configure the electronics

| Device | Required work |
|---|---|
| Stellar H7 V2 | Back up the existing configuration; establish the correct bootloader/initial flashing procedure; install the verified custom ArduPilot build; load hardware parameters. |
| XIAO ESP32C6 | Flash the appropriate DroneBridge firmware; configure Wi-Fi, UART pins, baud rate and MAVLink transport. |
| Eight ESC channels | Read and record installed firmware; configure direction and required settings. Update only where needed for compatibility or a known issue. |
| Laptop | Install/configure the ground station and control software; prepare log download and review tools. |

- [ ] Complete the FC backup, flashing and configuration.
- [ ] Complete XIAO flashing and configuration over USB before connecting FC power.
- [ ] Verify the XIAO power path before allowing simultaneous USB and FC-provided power.
- [ ] Complete ESC firmware inspection and configuration for all eight channels.
- [ ] Complete laptop setup and save the connection settings.

ArduPilot supports the StellarH7V2 target and DShot outputs. Verify the chosen RPM telemetry
arrangement against the exact firmware and board; do not assume bidirectional DShot works on
all eight outputs. See the [ArduPilot board documentation](https://ardupilot.org/copter/docs/common-stellarh7v2.html).

The planned XIAO configuration is MAVLink2 at 115200 baud, GPIO21 TX to FC RX1, GPIO2 RX from
FC TX1, flow control disabled, and UDP port 14550 to the laptop. Verify board pad labels against
GPIO numbers. See [DroneBridge](https://github.com/DroneBridge/ESP32) and its
[wiring documentation](https://dronebridge.gitbook.io/docs/dronebridge-for-esp32/hardware-and-wiring).

## 4. Assemble the airframe

- [ ] Print/fabricate the remaining structural parts and inspect them.
- [ ] Cut and finish the central tube; fit insulating protection at cable exits.
- [ ] Assemble and secure the frame joints.
- [ ] Install all eight motors; check that screw lengths cannot reach the windings.
- [ ] Mount both ESCs with electrical insulation and airflow.
- [ ] Soft-mount the FC, record its orientation and preserve USB/microSD access.
- [ ] Mount the current sensor and XIAO; keep the antenna clear of carbon and power wiring.
- [ ] Install the battery cradle, non-slip liner and straps.
- [ ] Check centre of mass, stiffness and clearances with the battery installed.

## 5. Make the harness and solder the wiring

### High-current connections

- [ ] Make the airframe XT60 lead with the planned 10 AWG main feed.
- [ ] Pass the positive main conductor through the HCS sensor in the correct direction.
- [ ] Make insulated positive and negative splits feeding both ESCs through 12 AWG branches.
- [ ] Connect each ESC's battery input.
- [ ] Fit one supplied 1000 µF, 35 V capacitor per ESC, with correct polarity and short leads.
- [ ] Connect all eight motors: 24 phase-wire connections at the ESCs.
- [ ] Take the FC battery feed from after the current-sensor measurement point.

The Hall sensor measures the conductor passing through its aperture. Motor current does not
pass through its small signal pads. Distribute two upright and two lateral motors to each ESC
board, as specified in the hardware plan.

### Signal and peripheral connections

- [ ] Connect eight FC motor outputs to their assigned ESC inputs, with signal grounds.
- [ ] Wire the selected ESC telemetry arrangement.
- [ ] Connect HCS regulated 5 V, ground and current output to the FC.
- [ ] Leave ESC analogue current outputs disconnected from the HCS current-sensor input.
- [ ] Connect XIAO power, ground and crossed TX/RX.
- [ ] Fit the buzzer/status hardware if selected.
- [ ] Insulate, label and strain-relieve every connection.
- [ ] Inspect for solder bridges, loose strands, exposed joints and wires touching carbon or
  entering propeller paths.

## 6. First power-up — props off

- [ ] Check polarity and resistance/continuity with a meter.
- [ ] Power through the smoke stopper; investigate unexpected current draw or heating.
- [ ] Measure the FC and peripheral supply rails.
- [ ] Confirm reliable booting from the flight battery.
- [ ] Confirm FC identification, firmware version and sensor health.
- [ ] Confirm Wi-Fi telemetry and commands work in both directions.
- [ ] Confirm microSD logging and download a sample log.
- [ ] Check reported battery voltage against a multimeter.
- [ ] Calibrate current-sensor zero and scale using a suitable reference.
- [ ] Remove the smoke stopper before tests whose motor current would exceed its rating.

## 7. Calibrate and verify — props off

- [ ] Calibrate accelerometers and check gyro health.
- [ ] Tilt the vehicle by hand and verify the displayed attitude and axes.
- [ ] Configure the estimator for the installed sensors and chosen flight mode.
- [ ] Test each motor individually; verify number, physical location and rotation.
- [ ] Verify corrective motor responses oppose roll, pitch and yaw disturbances.
- [ ] Verify lateral commands select the intended motors.
- [ ] Set reliable motor starting/idle behaviour and conservative initial output limits.
- [ ] Verify RPM/ESC telemetry where configured.
- [ ] Test arm, disarm, restart and mode-change behaviour.
- [ ] Test lost Wi-Fi, stopped command stream, closed ground station and link reconnection.
- [ ] Check low-battery and estimator-failure responses.
- [ ] Run the powered system long enough to reveal resets, overheating and communications problems.

ArduPilot's GCS failsafe watches heartbeats. Test stale control commands separately, and verify
the landing response with this aircraft's custom mode and mixer. See the
[GCS failsafe documentation](https://ardupilot.org/copter/docs/gcs-failsafe.html).

## 8. Loaded tests and first flight

- [ ] Fit the correct propellers after the preceding checks pass.
- [ ] Use the planned restraint/test enclosure for initial loaded tests.
- [ ] Increase load progressively; inspect vibration, current, battery sag and temperatures.
- [ ] Check the XT60, power joints, motors and both ESC boards after each run.
- [ ] Measure actual all-up mass and establish a realistic hover-throttle estimate.
- [ ] Adjust filters and initial gains using hardware logs.
- [ ] Confirm adequate attitude authority and acceptable motor saturation.
- [ ] Perform a brief low hover in the designated test area, then land and inspect.
- [ ] Review the log before increasing flight duration or testing lateral motion.
- [ ] Repeat until takeoff, controlled hover and landing are consistent.
- [ ] Save the successful firmware, parameters and build configuration as the baseline.

## Preparation schedule for the next few days

### Day 1

- [x] Download and pin replacement ArduPilot firmware (4.7.1).
- [ ] Settle first-flight control and failsafe behaviour.
- [x] Build the H7 target.

### Day 2

- [ ] Prepare hardware parameters and test scripts.
- [ ] Finalise motor and wiring diagrams.
- [ ] Flash USB-accessible boards if available.
- [ ] Assemble commissioning instructions and check remaining supplies.

Bring a verified firmware build, labelled wiring diagram, prepared configuration and ordered
test procedure to the lab. Complete actual hover calibration and tuning on the assembled vehicle.
