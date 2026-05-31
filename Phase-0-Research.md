# Phase 0 — Flight Controller Architecture Decision

**Goal:** Decide before any other work whether the flight stack is custom-from-scratch or an extension of ArduPilot. This decision shapes the entire downstream plan.

**Checkpoint criteria:**
- A written decision with justification
- Confirmed understanding of how lateral force is represented in the chosen stack

---

## The core question

The drone must **translate laterally without tilting** — this is the *fully-actuated / omnidirectional* problem. A custom mixer alone is not enough; the controller *above* the mixer must command lateral force directly.

Two paths:

| | Custom from scratch | ArduPilot extension |
|---|---|---|
| Mixer matrix | ~1–2 days either way | ~1–2 days either way |
| ESC/DShot output timing | Must build | Inherited |
| IMU filtering (lowpass + RPM notch) | Must build | Inherited |
| Attitude/state estimation | Must build | Inherited |
| Failsafes | Must build | Inherited |
| Tuning infrastructure | Must build | Inherited |
| 6DOF / omnidirectional control | Must design & build | Upstreamed path exists |
| SITL (sim ↔ firmware bridge) | Must build custom bridge | Connects to external physics sim |
| Documentation / community | Sparse | Active Discourse threads |

---

## ArduPilot 6DOF path — research notes

### Key source files

| File | Purpose |
|------|---------|
| `libraries/AP_Motors/AP_Motors6DOF.cpp` | 6DOF motor backend — `add_motor_raw_6dof()`, output mixing |
| `libraries/AP_Motors/AP_Motors6DOF.h` | Class definition, per-motor factor arrays |
| `libraries/AP_Motors/AP_MotorsMatrix.cpp` | Standard matrix mixer — `add_motor_raw()` (4DOF only) |
| `libraries/AP_Scripting/examples/Motors_6DoF.lua` | Lua example loading a custom 6DOF mixer matrix |
| `libraries/SITL/examples/JSON/readme.md` | Full SITL JSON bridge protocol spec |

### Relevant threads / PRs

| Source | Key takeaway |
|--------|--------------|
| [PR #16105 — Copter: add 6DoF support](https://github.com/ArduPilot/ardupilot/pull/16105) | Original upstreamed 6DOF implementation by IamPete1 |
| [Newbie trying 6DOF Omnicopter (Discourse)](https://discuss.ardupilot.org/t/newbie-trying-6dof-omnicopter/90319) | Users point to `Motors_6DoF.lua` as the entry point for custom frames |
| [Six Degrees of Freedom Omnicopter With ArduPilot](https://cyberops.eu/news/11578/six-degrees-of-freedom-omnicopter-with-ardupilot) | End-to-end build writeup |
| [Hackaday coverage of the 6DOF PR](https://hackaday.com/2021/01/09/six-degrees-of-freedom-omnicopter-with-ardupilot/) | Good overview of control architecture |

---

### Does our motor layout map onto the existing 6DOF path?

**Our geometry:** 4 upright motors + 4 canted lateral motors (octacopter)

#### The `add_motor_raw_6dof()` function

The relevant backend is **`AP_Motors6DOF`**, not the standard `AP_MotorsMatrix`. It exposes a dedicated function:

```cpp
void AP_Motors6DOF::add_motor_raw_6dof(
    int8_t  motor_num,
    float   roll_fac,
    float   pitch_fac,
    float   yaw_fac,
    float   throttle_fac,
    float   forward_fac,
    float   lat_fac,
    uint8_t testing_order
)
```

Six factors per motor — roll, pitch, yaw (rotational) plus throttle, forward, lateral (translational). **This is exactly what our geometry needs.** For our frame:

| Motor type | throttle_fac | forward_fac | lat_fac | roll/pitch/yaw |
|---|---|---|---|---|
| Upright (×4) | ~1.0 | 0.0 | 0.0 | Position-dependent |
| Canted (×4) | ~cos(θ) | ~sin(θ) × direction | ~sin(θ) × direction | Small due to canting |

> **Caveat (important):** The PR author derived mixer matrix values using **PSO (particle swarm optimisation) in MATLAB**, not direct geometry calculation. The factors interact across all 6 axes and must be solved as a system to avoid saturation and coupling. Raw cosine/sine of cant angle is a starting point, but the final values need proper optimisation or at minimum verification that the matrix is full-rank across our 6 desired DOFs. This is a ~1-day task in Phase 1.

#### Predefined 8-motor frames in `AP_Motors6DOF`

ArduPilot already ships two 8-motor 6DOF configurations:

- **`VECTORED_6DOF`** — 4 lateral-vectored motors + 4 vertical mixed motors
- **`VECTORED_6DOF_90DEG`** — alternate layout with more decoupled control axes

Our 4-upright + 4-canted layout is a close relative. We can likely start from one of these configurations as a template and adjust factors for our specific cant angle and arm geometry.

#### Lateral force as an explicit command axis

**Confirmed.** The output mixer in `output_armed_stabilizing()` is:

```cpp
linear_out[i] = throttle_thrust * _throttle_factor[i]
              + forward_thrust  * _forward_factor[i]
              + lateral_thrust  * _lateral_factor[i];
```

Lateral thrust is a **first-class command axis** at the mixer level. The attitude controller above it transforms earth-frame acceleration setpoints into body-frame thrust commands (including lateral), so the pendulum outer loop can issue `desired_lateral_force` directly.

#### Yaw coupling

The dual-track normalisation in `VECTORED_6DOF` separates:
- Track 1: rotational + pitch + throttle (`rpt_out`)
- Track 2: yaw + forward + lateral (`yfl_out`)

Each track is normalised independently before summing. This prevents saturation in one axis (e.g., full lateral demand) from robbing authority in an orthogonal axis (e.g., yaw). This is the right approach for our mixed-orientation prop geometry where yaw coupling is non-trivial.

**Summary:** Our motor layout maps cleanly onto the existing 6DOF path. ✓

---

### SITL ↔ external physics bridge

ArduPilot exposes a well-documented **JSON SITL interface** for connecting any external physics engine (Godot, MATLAB, custom Python, etc.).

#### Communication protocol

**Transport:** UDP. The physics backend listens on **port 9002**. ArduPilot auto-discovers the backend's reply address — no manual IP/port config needed in the physics engine.

**ArduPilot → Physics backend (binary packet):**

```
uint16  magic       = 18458 (16-ch) or 29569 (32-ch)
uint16  frame_rate
uint32  frame_count
uint16  pwm[16]     (or pwm[32] if SERVO_32_ENABLE = 1)
```

PWM values are 1000–2000 µs. `frame_count` increments every output step; the physics backend can detect dropped or duplicate frames.

**Physics backend → ArduPilot (plain-text JSON, newline-terminated):**

| Field | Type | Required | Notes |
|-------|------|----------|-------|
| `timestamp` | float (s) | ✓ | Absolute physics time |
| `imu.gyro` | [rad/s, rad/s, rad/s] | ✓ | Body frame |
| `imu.accel_body` | [m/s², m/s², m/s²] | ✓ | Body frame |
| `position` | [N, E, D] m | ✓ | Earth frame |
| `velocity` | [N, E, D] m/s | ✓ | Earth frame |
| `attitude` OR `quaternion` | [rad,rad,rad] or [q1-q4] | ✓ | One required |
| `rng_1`…`rng_6` | float (m) | optional | Rangefinders |
| `battery` | {voltage, current} | optional | |

**Example minimal response:**
```json
{"timestamp":2.5,"imu":{"gyro":[0,0,0],"accel_body":[0,0,-9.81]},"position":[0,0,0],"attitude":[0,0,0],"velocity":[0,0,0]}
```

#### Timing

- Recommended simulation rate: **≥400 Hz** for copter (set via `SIM_RATE_HZ`)
- The physics backend may enforce its own maximum timestep and can run faster than the recommended rate
- ArduPilot sends a heartbeat every 10 s to help the backend detect connection state

**Summary:** The SITL JSON bridge is fully specified, requires no ArduPilot source modifications, and supports the per-motor PWM output + IMU/state feedback that Godot will need. ✓

---

## Custom from scratch — considerations

*(Only fill this section if ArduPilot shows a blocking problem.)*

- What would be the minimum viable custom stack?
- What is the realistic time cost vs ArduPilot?

---

## Decision

**Status:** DECIDED — 2026-05-31

**Chosen path:** ArduPilot

**Justification:**

Everything time-consuming in a flight stack — ESC/DShot output timing, IMU filtering (lowpass + RPM notch), attitude/state estimation, EKF, failsafes, tuning infrastructure — is inherited from ArduPilot rather than built from scratch. The `AP_Motors6DOF` class already supports our exact need: lateral force as a first-class command axis, with `add_motor_raw_6dof()` accepting per-motor throttle, forward, and lateral factors alongside the rotational axes. Two existing 8-motor `VECTORED_6DOF` configurations give us a template to start from. The SITL JSON bridge is fully documented and ready to connect to Godot. None of this would exist on a from-scratch path.

The only non-trivial ArduPilot-specific work is deriving the 8-motor factor matrix for our cant angle and arm geometry — but that's a Phase 1 task regardless of which path we chose.

**How lateral force is represented in the chosen stack:**

In `AP_Motors6DOF`, lateral force is a first-class axis alongside roll, pitch, yaw, throttle, and forward. The attitude controller transforms earth-frame acceleration setpoints into body-frame force/torque commands. The mixer then computes per-motor throttle via:

```
output[i] = roll * roll_fac[i] + pitch * pitch_fac[i] + yaw * yaw_fac[i]
           + throttle * throttle_fac[i] + forward * forward_fac[i] + lateral * lat_fac[i]
```

The pendulum outer loop sits above this: it commands a desired lateral acceleration, which flows into the position/attitude controller and appears as `lateral_thrust` at the mixer.

---

## Open questions

- [ ] **Mixer matrix derivation:** Confirm we can calculate or optimise the 8-motor factor matrix for our specific cant angle and arm radius (or start from the existing `VECTORED_6DOF` constants and adjust). Is a PSO pass needed, or is analytical derivation good enough for Phase 1?
- [ ] **6DOF frame selection in firmware:** Which `FRAME_TYPE` parameter value enables `AP_Motors6DOF` on ArduCopter for a custom frame? (The Lua script path is confirmed; verify this works in firmware without recompiling.)
- [ ] **Pendulum outer loop interface:** Where exactly does the pendulum controller inject its lateral force setpoint in the ArduPilot control chain? Confirm it's above the attitude controller (not competing with attitude hold).
- [ ] **H743 compatibility:** Verify the 6DOF control path compiles and runs on the H743 (STM32H743). Check if any SITL-only code paths exist that would break on hardware.

---

## Phase 0 checkpoint: PASS ✓

- [x] Written decision with justification recorded above
- [x] Lateral force representation confirmed and documented
