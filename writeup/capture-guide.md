# Evidence capture guide

This guide turns the existing validations into an auditable evidence set. It deliberately
separates runs that work today from captures that require a small logging addition.

## Before every capture

1. Use a clean, named output directory. Do not overwrite an earlier run.
2. Copy [`data/run-metadata-template.yaml`](data/run-metadata-template.yaml) into it and fill it
   out before interpreting results.
3. Record `git rev-parse HEAD` and `git status --short`. A dirty tree is acceptable if its diff
   is archived or described; an unknown tree is not.
4. Record the Godot version, binary path, scene, environment overrides, sensor seed, and expected
   duration.
5. Save stdout/stderr even when a CSV is produced. It contains pass/fail summaries and mode
   transitions.
6. Run Godot batch cases serially. Existing notes report intermittent shared import-cache races
   between parallel instances.

The installed executable used during this audit was:

```text
/home/omicrontheta/Godot/Godot_v4.7.1-stable_linux.x86_64
```

Replace it with a local `godot` command where appropriate; always capture the reported version.

## Captures that work with the current harness

### A. Portable-core smoke and single-link plant tests

Purpose: prove the mixer mapping and dependency-free controller/idealized-plant path before showing
the integrated Godot result.

```bash
./native/tests/test_flightstack
./native/tests/test_plant
godot --headless --path sim-v-1 --script res://tests/test_simulation_components.gd
```

The current binaries passed during the write-up audit. Rebuild them from source before treating
the output as release evidence, and archive both build command and compiler version.

### B. Idealized double-balance time history

Purpose: produce a plot-ready CSV with command, realized acceleration, and both link angles.

```bash
BALANCE2_HEADLESS=1 \
BALANCE2_SEED=1 \
BALANCE2_CSV="$PWD/writeup/data/raw/simulation/balance2-ideal-seed-001.csv" \
godot --headless --path sim-v-1 res://balance2_test.tscn \
  > "$PWD/writeup/data/raw/simulation/balance2-ideal-seed-001.log" 2>&1

python3 native/tools/plot_accel2.py \
  writeup/data/raw/simulation/balance2-ideal-seed-001.csv \
  writeup/figures/source/balance2-ideal-seed-001.png
```

Current CSV columns are `t,a_cmd,a_real,theta1_deg,theta2_deg`. This is the idealized cart harness,
not the full flying rigid body; label it accordingly.

### C. Integrated noisy double swing-up/catch/hold

Purpose: validate the headline maneuver on the real native inner loop and current Godot/Jolt plant.

```bash
USE_NATIVE=1 \
NUM_ARMS=2 \
SWINGUP=1 \
USE_SENSOR_NOISE=1 \
SENSOR_SEED=1 \
RECORDING_AUTO_EXIT=1 \
godot --headless --path sim-v-1 res://main.tscn \
  > "$PWD/writeup/data/raw/simulation/swingup-full-seed-001.log" 2>&1
```

A current audit run caught at 3.50 s and passed the configured 8 s hold before releasing the
arms. At present this command produces useful summaries and coarse once-per-second state lines,
but not the full-state CSV required for V06. Use it as a check, not as the final plot source.

Repeat the release evidence for the documented seed set rather than promoting seed 1 alone.

### D. Integrated noisy double balance

Purpose: establish a representative balance run and the per-run summary on the real inner loop.

```bash
USE_NATIVE=1 \
NUM_ARMS=2 \
BALANCE_ONBOARD=1 \
USE_SENSOR_NOISE=1 \
SENSOR_SEED=1 \
RECORDING_AUTO_EXIT=1 \
godot --headless --path sim-v-1 res://main.tscn \
  > "$PWD/writeup/data/raw/simulation/balance-full-seed-001.log" 2>&1
```

Parse the `RESULT_BAL` line into the experiment manifest. The full state still needs the logging
addition described below.

### E. Reaction-feed-forward comparison video

Purpose: create a deterministic, synchronized visual ablation after the arms are released.

```bash
GODOT_BIN=/path/to/godot \
bash sim-v-1/tools/render_ff_comparison.sh \
  "$PWD/writeup/recordings/raw/reaction-ff-seed-001"
```

The script holds the two runs identical until release and then changes only `POSTREL_FF`. Preserve
the two source clips as raw recordings. Put the labeled composite in `recordings/final/` after
checking trim alignment. A numeric paired trace is still needed before quoting the 2.2× result.

### F. Existing acceleration step trace

`sim-v-1/simulation/acceleration_grader.gd` prints `[step]` and full-rate `[trace]` rows for
150 ms after a command transition. The old SITL autotest parser in `sitl/autotest/tracetime.py`
documents the intended 5%/1% timing metrics.

For Edition 1, make a deterministic native, zero-arm run with a moderate +6 m/s² command, save the
log, and convert its `[trace]` rows to CSV. Do not use the ±12 m/s² rail case for the headline
response because saturation changes the inverse-model behavior.

## Small logging addition needed for the best plots

The integrated `main.gd` harness should gain an optional `ANALYSIS_CSV=/absolute/path.csv` output.
Keep it disabled by default and write at 100–400 Hz. One row should contain:

```text
time_s,seed,phase,controller_mode,reaction_ff_on,
theta1_true_rad,theta1_sensed_rad,theta1_rate_true_rad_s,theta1_rate_sensed_rad_s,
theta2_true_rad,theta2_sensed_rad,theta2_rate_true_rad_s,theta2_rate_sensed_rad_s,
position_x_m,velocity_x_m_s,altitude_m,velocity_y_m_s,
body_roll_rad,body_pitch_rad,body_yaw_rad,
accel_cmd_x_m_s2,accel_real_x_m_s2,accel_cmd_y_m_s2,accel_real_y_m_s2,
motor_1_norm,motor_2_norm,motor_3_norm,motor_4_norm,
motor_5_norm,motor_6_norm,motor_7_norm,motor_8_norm,
control_saturated,arm_release_event,catch_event
```

Desirable extra fields are feed-forward terms separated by source—reference/motor inverse, drag,
reaction lateral, reaction vertical, and feedback trim—and the battery voltage factor. Those make
controller ablations far more intelligible.

Log both truth and sensed arm states. The difference is one of the most direct ways to show that
the controller is not operating on perfect simulator state. Keep grading on truth.

## Batch experiments for Edition 1

### Swing-up repeatability

- Configuration: current double, native stack, sensor model on, fixed nominal latencies.
- Hold protocol: explicitly set `SWINGUP_HOLD_TIME`; use 13 s if the public claim remains “more
  than 13 s,” or revise the claim to the current 8 s default.
- Seeds: at least 1–20 for a release-quality statistic; retain 1–6 for direct comparison with the
  existing claim.
- Record: catch success, catch time, worst link angle at catch, maximum during the final hold
  window, cart travel, altitude deviation, peak and RMS command, saturation duration.
- Plot: each run as a thin line or point; median and full range; no cherry-picked trajectory.

### Balance repeatability

- Start both links at the documented perturbation.
- Use a fixed mid-run base-velocity bump.
- Record maximum/final angle per link, RMS command, saturation, altitude error, and pass/fail.
- Include at least one no-noise control and 20 noisy seeds.

### Shared arm-sensing latency sweep

- Hold all noise and plant parameters fixed.
- Sweep the shared angle/rate latency densely from nominal through failure; use 2.5 or 5 ms steps
  around the boundary.
- Run several seeds per point.
- Plot injected sensor latency on the x-axis. Show the separately estimated actuator/filter/ZOH
  contribution when discussing total effective delay.
- Archive failures. Their growing oscillation is the clearest evidence of the boundary.

### Reaction-feed-forward ablation

- Use identical seed and state up to the release event.
- Turn only post-release reaction FF on/off.
- Measure lateral and vertical acceleration error, peak displacement, and RMS over a declared
  window after release.
- Keep altitude hold identical. Otherwise the experiment confounds lateral reaction cancellation
  with arm-weight support.

### Sensitivity sweep

Run only after plausible ranges are written down. Vary one family at a time:

- body and link mass/inertia;
- motor spin-up/down lag and thrust scale;
- hinge viscous/Coulomb friction;
- body/bob drag;
- sensing latency;
- noise/vibration amplitude;
- battery sag.

Report failure boundary or performance degradation, not merely a grid of nominal passes.

## Hardware evidence sequence

### H1. Configuration identification

Measure finished mass, center of mass, motor coordinates and axes, link lengths, distributed link
mass, pivot friction, and inertia estimate. This closes the dated configuration discrepancy before
control tests begin.

### H2. Motor/prop characterization

On a guarded thrust stand, record voltage, DShot/PWM command, RPM, thrust, current, and temperature.
Run static curves and small/large step responses at several battery voltages. Estimate asymmetric
spin-up/down time constants and saturation. Use the actual selected propeller.

### H3. Sensor and latency characterization

Measure raw IMU PSD with motors off and at restrained operating points; encoder/arm-IMU angle
error against a reference; end-to-end timestamp latency; optical-flow velocity error; and drift.
Retain raw full-rate logs so filtering can be changed offline.

### H4. Bare-airframe response

Use a restraint or suitable enclosure. Repeat the same lateral command profile as simulation and
plot command, lateral acceleration, body pitch, motor outputs, and voltage. This is the first honest
sim-to-real overlay.

### H5. Pendulum ladder

Proceed from a fixed-base joint test to restrained single-link balance, free single-link balance,
restrained double balance, then any double swing-up attempt. Define abort thresholds and success
criteria before each test. Preserve unsuccessful runs that trigger a model or controller change.

## Release checklist

- Every headline number links to raw data and metadata.
- The plot script regenerates every final graph from archived input.
- Seeds, sample rate, filters, and pass thresholds are stated.
- The same angle, sign, and frame conventions are used throughout.
- Simulation parameters and current physical-design parameters are not combined in one unlabeled
  column.
- Videos have a visible “simulation,” “bench,” or “flight” label.
- A reader can tell whether a run uses the idealized cart, full Godot plant, SITL bridge, or native
  in-process stack.
