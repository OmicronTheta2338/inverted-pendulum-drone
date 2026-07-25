# Native Flight Stack — Migration & Validation Plan

The plan here is to retire the ArduPilot SITL bridge and validate the native
(in-process) flight stack end to end. I've split it into self-contained work packets so
each one fits in its own context window. Read the Context and Load-bearing concepts
sections once; after that, a single packet section is enough to work that packet on its
own.

---

## Context — why this exists

The old path ran real ArduPilot firmware in SITL and bridged it to the Godot plant over
UDP (`sim-v-1/main.gd:_service_sitl`). That bridge cost ~20–40 ms of dead-time (the async
UDP round-trip plus AP's `AP_RCProtocol_FDM` RC pipeline), which is fatal for an unstable
balance loop. Most of the fix already exists: mode 30 (`ModeAccel`) is ported to a
Godot-free C++ core (`native/src/mode_accel.*`) behind an `ApFlightStack` GDExtension, and
`main.gd` runs it in-process with no bridge latency (`main.gd:_service_native`, env
`USE_NATIVE=1`).

This isn't a from-scratch build. Here's where the four original goals stand:

| Goal | State | Remaining |
|---|---|---|
| Portable C++ control layer | **Done** — `ApFlightStack` wired in | Git-track it; hand-sync with firmware |
| Fast/precise accel mode | **Implemented** — overdrive FF, delayed-fb I, drag-cancel, 0/1/2-arm reaction FF in `mode_accel.cpp` | Validated vs the *bridge*, not the native path → re-validate (Packets A, C) |
| Sensor-noise layer | **Validated (Packet B)** — `main.gd:_sensed_state()`, `USE_SENSOR_NOISE=1`; accel loop tracks under noise, delay budget ~25 ms ≪ 139 ms | Empirical RHP break-test deferred to D (needs the pole in the loop) |
| Balancing scripts | **Pass Phase 8 — but vs an *idealized* accel model** (first-order cart, τ≈8 ms, `balance_test.gd`). Never stacked on real `ModeAccel` | Close the full loop (Packet D, capstone) |

The outcome I'm after: a fully native, bridge-free sim where the real `ModeAccel` realizes
commanded accelerations fast and precisely, under a realistic noise+latency model, with the
balance controller closing the loop on that real inner layer for 15 s.

### Does ArduPilot add noise to copy?
It does, but we deliberately don't copy it. AP's SITL INS (`AP_InertialSensor_SITL.cpp`)
only ever adds a tiny uniform baseline (accel σ≈0.01 m/s², gyro σ≈0.04°/s); the realistic
knobs (`ACC*_RND`, `GYR*_RND`, `VIB_*`) default to off. The thing AP introduced that
actually matters is EKF3 sampling latency and fusion lag, since `ModeAccel` reads EKF
estimates rather than raw sensors. Our harness reproduces that intent — Gaussian
per-channel noise plus per-group latency ring buffers — following the Phase-4 decision. It
stays in the harness; we don't replicate `AP_InertialSensor_SITL`.

---

## Load-bearing concepts (own these before the packets that lean on them)

1. **Acceleration-input cart abstraction.** Treat the drone as a cart whose lateral accel
   is the control input; the linearised pendulum then comes out mass-independent
   (`θ̈ = (g/L)θ − (1/L)u`). That's the reason the balance layer and the accel layer can be
   designed separately. (Balance-Controller-Design.md §2.)
2. **RHP-pole delay budget.** The unstable pole sits at √(g/L) ≈ 4.04 rad/s (single) /
   +7.2 rad/s (double). You can stabilise it only if the total effective loop delay is far
   below 1/p (≈248 ms / ≈139 ms). What kills you is latency, not noise amplitude, and that
   is what Packet B is really testing. (§5.)
3. **Inverse-motor-model overdrive FF.** This is how `ModeAccel` gets its ~40 ms accel
   response: it commands the exact inverse of the first-order motor lag through a
   saturation-aware reference governor (`mode_accel.cpp:292-342`). Packets A and D lean on
   it.
4. **Reaction-force FF.** The drone cancels each arm's hinge tension analytically,
   Smith-predicting θ,θ̇ forward ~42 ms: `a_pend=(m_b/m_d)[(a_d−g)·u − L·θ̇²]·u`
   (`mode_accel.cpp:464-491`). This one is Packet C.

## Decisions locked
- Noise lives in the GDScript harness (`_sensed_state`), not in portable C++. It's "the
  hardware," and it must never ship to the H743.
- Target the sim's MASS = 1.0 kg for now. The 850 g airframe (`numbers`) recal is deferred
  to hardware bring-up (see Out of scope).
- End-to-end integration (Packet D) is the capstone.

---

## Sequencing / dependencies

```
Packet 0 done (baseline+git) ─┬─► Packet A done (accel, no arms) ─┐
                              └─► Packet B (noise+latency) ───────┼─► Packet D (integration) ─► Packet E (retune, contingent)
                                  Packet A ──► Packet C (arms) ───┘
```
A and B are independent (parallelizable). C needs A. D needs A, B, C.

---

## Packet 0 — Baseline & git hygiene  *(prerequisite)*

**Why:** the whole `native/` subsystem plus `sim-v-1/armbalance.gdextension` is untracked
(`git status` shows `??`). Lock a known-good baseline so every later packet starts from a
clean diff.

**Do:**
- Branch off `main` (don't work on `main`).
- Rebuild and confirm it loads: `cd native && scons -j$(nproc)` (on Fedora, add
  `use_static_cpp=no`). Output lands at `sim-v-1/bin/libarmbalance.*.so`.
- Get the smoke tests passing: `native/tests/test_flightstack`, `native/tests/test_plant`
  (build commands live in each `*_main.cpp` header).
- Native flight is clean: `USE_NATIVE=1 godot --path sim-v-1 res://main.tscn` hovers and
  translates.
- Commit the native tree, the `.gdextension`, and the `main.gd` native path as the
  baseline. `.gitignore` already excludes `*.os`/`*.so`/`godot-cpp/`.

**Acceptance:** clean branch; extension builds; both smoke tests pass; native sim hovers;
baseline committed.

---

## Packet A — Native accel loop, no arms — Complete *(Task 3a + finish Task 1)*

> Status: complete (2026-07-10). Native path, 0 arms: lat +6 enters the 5% band at ~15 ms
> with 0.7% overshoot and settles <1%; +12 saturates the overdrive and gives 3.7% overshoot
> (the bang-bang regime). Drag-cancel holds accel out to 36 m/s; park hold pins lateral
> velocity at −2.89 m/s flat for 30 s, no runaway. Constants are byte-identical to the
> ArduPilot source (only the I/O binding differs). SPEC reconstructed in the appendix below.
> All acceptance criteria met. *(memory: `packetA-native-accel-validated`)*

**Goal:** show that native `ModeAccel` realizes commanded vertical and lateral accel fast
and precisely on the native path (clean truth first): ~40 ms to 5%, drag-cancel giving
constant velocity at zero command, no runaway.

**Key files:** `native/src/mode_accel.{h,cpp}`, `ap_attitude_control.cpp`,
`ap_motors6dof.cpp`; grader `main.gd:_grade()` (`:745-789`); `sitl/autotest/analyze.py`.

**Do:**
- Run: `USE_NATIVE=1 AUTOTEST=1 NUM_ARMS=0 godot --headless --path sim-v-1
  res://main.tscn`. The grader compares the body's own dv/dt (truth) against commanded
  accel, independent of the signal the controller closes on. Parse `[grade]`/`[step]`/`[trace]`.
- Check rise time, overshoot, and steady residual against the spec, and verify that
  drag-cancel gives constant lateral velocity at zero command (no runaway — memory
  `accel-drag-ff-runaway`).
- Clear the SPEC debt: `mode_accel.cpp` cites "SPEC 3 / 4.2 / 6 / §7" but no SPEC.md
  exists. Reconstruct a short spec (response target, FF/overdrive rationale, drag-cancel,
  calibrated constants and their 1.0 kg provenance) and add it to this doc's appendix, so
  the constants stay traceable and rebuildable.
- Sync check: diff native `mode_accel.cpp` against
  `~/03_Projects/ardupilot/ArduCopter/mode_accel.cpp` and record the intended divergences
  (typed structs vs RC/AHRS singletons).

**Acceptance:** native path, 0 arms, hits the vertical and lateral accel steps within
spec; constant velocity at zero command; no runaway; spec written.

---

## Packet B — Sensor-noise + latency layer (harness) — Complete *(Task 2)*

> Result (native path): the noise layer is enabled, tuned to the real sensor plan, and
> validated. The 0-arm accel loop tracks identically with noise on versus off (mean|e| lat
> 0.085 vs 0.083, up 0.272 vs 0.268): the grader stays on truth (`main.gd:456`), and the
> FF-dominant loop attenuates the injected accel σ (0.35 m/s²) about 60× down to a
> ~0.006 m/s² realised residual. There's no drift over the 50 s hold (flat ~0.006, not
> growing) because this loop closes no position feedback. Arm sensing is split by hardware:
> arm 1 uses 2×IMU fusion, arm 2 a magnetic encoder (see appendix). The 2-arm smoke test
> stayed bounded (no runaway; full arm validation is Packet C). Delay budget: effective
> θ-loop delay ≈ 25 ms, far under 247 ms (single) / 139 ms (double), so ~10× / ~5.6× margin;
> the ~15 ms native actuator lag dominates and the sensor latencies are noise inside it. Full
> model and budget are in the "Sensor + latency layer" appendix.

**Goal:** enable, tune, and validate the existing `_sensed_state` model on the native path.
The point is the latency budget (concept #2), not amplitude cosmetics.

**Key files:** `main.gd:_sensor_setup()` (`:487-510`), `_sensed_state()` (`:516-574`),
config block (`:146-177`), `_read_ch` (`:577-579`). Env: `USE_SENSOR_NOISE`,
`SENSOR_SEED`, `SENSOR_LAT_EKF_{ATT,VEL,POS}_MS`, `SENSOR_POS_DRIFT`.

**Do:**
- Turn it on: `USE_NATIVE=1 USE_SENSOR_NOISE=1 …` and confirm the Packet-A accel loop
  still tracks (the grader stays on truth by design — `main.gd:456`).
- Tune the σ's toward the real sensor plan (the current values are placeholders). The
  arm-angle sensing hardware isn't decided yet, so pick a defensible encoder σ and LSB and
  flag it as a hardware-dependent assumption to revisit.
- Validate the latency ring depths (imu 2.5 / ekf_att 8 / ekf_vel,pos 20 / arm 2.5 ms)
  against the RHP-pole budget: effective delay well under 1/p (248 ms single / 139 ms
  double). Sweep `SENSOR_LAT_EKF_*_MS` to see the margin.
- Don't wire noise into the (to-be-retired) UDP bridge path.

**Acceptance:** native path runs stably with noise on; accel loop still tracks;
noise/latency parameters and the delay-budget justification documented.

---

## Packet C — Arm reaction-force FF, with arms *(Task 3b)*

> Status: complete (2026-07-11), series double inverted pendulum. The first pass validated the FF
> against `main.tscn`'s two parallel arms, which was the wrong topology (`scene-topology-native-vs-balance`).
> `main.tscn` is now the real series chain (`Arm2` hinges to `Arm`'s tip), and the reaction FF was
> generalised from a per-arm sum to the coupled double-pendulum reaction `R = Σ mᵢ(g − a_bobᵢ)`, with
> each link's `θ̈` coming from the coupled EOM under effective gravity `g_eff = g − a_d`. It reduces exactly to
> the single-pendulum form at `m₂=0`. The derivation is validated against truth: the harness computes the true
> reaction from the bobs' own accelerations (`REACT_VALIDATE=1`), and the formula matches it to 4.7 % lat /
> 3.1 % up. The FF cuts the double-pendulum disturbance ~2.2× (lat 1.18→0.53, up 1.00→0.44 RMS), and the
> single link returns to the 0-arm baseline. Smith prediction is left off (`PEND_PREDICT_S=0`): a value >0 helps
> lateral but destabilises the vertical near aligned links, so it's a Packet-D tuning knob. Packets A and B are
> unaffected (A is 0-arm; B is a topology-independent sensor layer). See the appendix and memory.

**Goal:** validate the pendulum reaction FF so that a swinging arm barely perturbs the
drone's commanded accel. Open-loop arm, no balance controller yet.

**Key files:** reaction FF `mode_accel.cpp:464-491` (constants `PEND_MASS_RATIO=0.10`,
`PEND_L=0.60`, `PEND_PREDICT_S≈0.042`); arm state feed `main.gd:_update_pendulum()`
(`:871-887`) → `set_command(...theta,theta_dot)`.

**Do:**
- Run the accel profile with 1 then 2 free-swinging arms:
  `USE_NATIVE=1 AUTOTEST=1 NUM_ARMS=1 …` then `NUM_ARMS=2`.
- Compare drone accel tracking (grader) FF-on against FF-off to show that the reaction FF
  measurably reduces the arm-induced perturbation.
- Sanity-check `PEND_MASS_RATIO`/`PEND_L` against the sim arm (0.1 kg bob, 0.6 m), and the
  ~42 ms prediction horizon against the loop delay from Packet B.

**Acceptance:** with arms swinging, accel tracking degrades only marginally versus 0-arms,
and FF-on clearly beats FF-off.

---

## Packet D — Capstone: balance controller on the real ModeAccel *(Task 4)*

> Status: complete (2026-07-11). The balance controller is wired in-process (`BALANCE_ONBOARD=1`, `main.gd`) onto the
> real native ModeAccel plus coupled reaction FF plus the Packet-B sensor layer, geometry 0.7/0.6. Both the single and double
> inverted pendulum balance ≥15 s under noise, matching the idealized-cart (`balance_test`/`balance2_test`)
> numbers, with Monte-Carlo 8/8 seeds. The real ~25–30 ms effective-delay loop holds the arm as well as the assumed
> 8 ms τ_cl does. The empirical RHP break-test: the double falls at ~70 ms total effective delay (p·τ ≈ 0.5), about 2.5× the
> nominal margin, and the rate-channel latency is what kills it, not the angle. Two design-changing findings came out of it. First, the
> reaction FF's lateral job turns out near-redundant with the balance loop (a bounded disturbance the loop rejects anyway),
> while its vertical arm-weight carry was the stability-critical part, now handled by an explicit altitude-hold PD.
> Second, `up_stick=0` holds zero accel, which permits a constant-velocity sink, so the vertical axis needs a position
> loop. See the appendix. *(memory: `packetD-capstone-validated`)*

**Goal:** replace the idealized first-order cart with the real inner loop and close the
full loop: balance → real `ModeAccel` → Jolt plant (real hinged arm) → noisy/delayed
sensors (Packet B) → balance. Confirm 15 s upright.

**Key files:** new scene/mode (`flight_balance_test.{tscn,gd}` or an env-gated mode
in `main.gd`, e.g. `BALANCE_ONBOARD=1`) that instantiates `ArmBalance`/`ArmBalance2`
and feeds its output into `flightstack.set_command`. Reuse: `gdext_arm_balance.cpp`
(`configure`/`update`), `balance_test.gd` for angle convention and gains, `_service_native`
for state plumbing.

**Wiring gotchas:**
- **Command mapping.** Balance outputs `a_cmd` in m/s² (clamped ±12); `ModeAccel`'s lat
  stick normalizes to `ACCEL_LAT_CMD_MSS = 12`. So `lat_stick = clamp(a_cmd/12, -1, 1)`,
  a clean 1:1. (Watch the historical set_lateral scaling bug: memory
  `lateral-throttle-scaling-bug`.)
- **Angle convention.** `_update_pendulum` gives θ from straight-down; the balance
  controller wants θ from straight-up (+→+X). Reuse the exact `balance_test.gd`
  conversion rather than re-deriving it.
- **Cart state.** Feed `s,v` = the drone's lateral position/velocity (the lateral component
  of `pos_ned`/`vel_ned`), not a kinematic cart.
- Feed the same sensed arm state to both the balance controller and `ModeAccel`'s
  reaction FF.

**Do:** get the single arm balancing ≥15 s under noise, then the double arm; run a
Monte-Carlo seed sweep like Phase 8; compare real-loop against idealized `balance_test.gd`
numbers (does the true effective latency, versus the assumed 8 ms τ, still hold the arm?).

**Acceptance:** single then double inverted pendulum balances ≥15 s on the native path
with real `ModeAccel` and noise on, across seeds; margin characterized against Phase 8.

---

## Packet E — Balance retune / margin *(contingent on D)*

Do this only if D shows the real loop breaks what the idealized model held. Retune the LQR
`Q/R` (`native/tools/compute_lqr.py` / `compute_lqr2.py`), adjust the estimator cutoffs
(`arm_balance_controller.h:56-68`), or tighten the accel loop, then re-characterize the
delay margin. This may fold into D.

---

## Global verification

- **Build:** `cd native && scons -j$(nproc)` (Fedora `use_static_cpp=no`); the `.so` lands
  in `sim-v-1/bin/`.
- **Pure-C++ (fast):** `native/tests/`: `test_plant`, `test_flightstack`.
- **Accel grader:** `USE_NATIVE=1 AUTOTEST=1 [NUM_ARMS=n] godot --headless --path
  sim-v-1 res://main.tscn`, which emits `[grade]/[step]/[trace]`; post-process with `analyze.py`.
- **Balance:** `godot --path sim-v-1 res://balance_test.tscn` (interactive) or
  `BALANCE_HEADLESS=1 … res://balance_test.tscn`; Monte-Carlo
  `bash native/tests/balance_mc.sh seeds 24 7.5`.
- Run `/verify` on each packet end-to-end before committing.

## Out of scope (named so they aren't lost)
- 850 g recalibration of `MASS` and the `ModeAccel` constants (hover throttle, lateral
  rail LUT, vertical slope), deferred to hardware bring-up.
- Retiring the UDP bridge code (`_service_sitl`, `sitl/`): leave it until the native path
  is fully validated, and keep it as a fallback reference.
- Swing-up: balance only, as in Phase 8.
- Portable C++ estimator / EKF-lite: not now (noise stays in the harness).

---

## Appendix — SPEC (reconstructed by Packet A)

`mode_accel.cpp` cites "SPEC 3 / 4.2 / 6 / §7" but no such file existed. This is that spec,
reconstructed from the code and the Packet-A validation run. Everything here targets the
sim's MASS = 1.0 kg body (850 g recal deferred, see Decisions locked).

### §3 Response target
Commanded lateral/vertical acceleration is realised fast, monotone, and without overshoot:
to within 5% of a step in ~40 ms (the design ceiling), with settled error <1%. Measured
(native, 0 arms): lat +6 m/s² enters the 5% band at ~15 ms, peaks at 6.04 (0.7% overshoot),
and settles <1%. This is the loop the balance layer will close on, so any overshoot in
realised accel is a disturbance kick to the pendulum, and a clean monotone rise is worth
more than raw speed.

### §4.1 Feed-forward = plant inversion onto a chosen reference (the load-bearing idea)
The lateral/vertical actuator is a first-order lag: `ȧ = (u − a)/τ` (accel `a` chases
command `u` at a rate proportional to the gap; sim `MOTOR_TAU ≈ 30 ms`). For a first-order
plant, any smooth output path `a_des(t)` is produced by exactly one command,

```
    u(t) = a_des(t) + τ · ȧ_des(t)          (the "+1" delivers the target; the "τ·s" is the overdrive)
```

so the command isn't unique: the design gets to choose the output path and invert onto
it. We pick a first-order reference `a_des = A·(1 − e^(−t/FF_TAU))`, `FF_TAU = 8 ms`, and
that single choice generates everything below.
- **Overdrive.** At the step the command leads the target by `τ/FF_TAU ≈ 4.6×`
  (`lat_ff = ref + tau_lat·(cmd − ref)/FF_TAU`, `mode_accel.cpp:88-90`). A +6 command, for
  instance, feeds ~27.75 m/s² on the first tick, then decays to 6 with an 8 ms constant.
  That's why the response beats the motor's own ~3τ ≈ 84 ms: the `(τs+1)` premultiply
  cancels the motor lag and leaves the 8 ms reference we designed.
- **Why the first-order member, not a faster one.** The fast end of the family is
  minimum-time bang-bang (also pure feed-forward, also no "look and see"), but its switch
  time is only right when `τ` is exactly right. The first-order FF is the forgiving member:
  tolerant of `τ` error (battery sag, motor temp, prop wear on the H743) and monotone under
  model mismatch. FF does about 95% of the work and a small delayed I trims the last ~5%. In
  the sim `τ` matches exactly, so the conservatism buys nothing here beyond noise/latency
  margin for Packet B; its real payoff is hardware transfer.
- **Saturation-aware reference governor** (`mode_accel.cpp:91-114`). When the demanded
  overdrive exceeds the rail (`ACCEL_LAT_MAX_MSS = 38`), clamp the command to the rail and
  advance the reference as a plant driven by the rail; an overshoot-guard snaps `ref → cmd`
  on crossing. This is an involuntary bang-bang segment: a +12 command wants
  `0.040·12/0.008 = 60 > 38`, so it saturates and overshoots 3.7% against +6's 0.7%. The
  extra overshoot at +12 is the measured price of losing the exact inverse to the rail.

### §4.2 Drag-cancel: constant velocity at zero command
At zero accel command the body has to coast at constant lateral velocity, not decay.
`a_lat_out += ACCEL_DRAG_K_LAT · v_lat` (`:150`) cancels velocity-dependent aero drag.
`K = 0.05 ≤ true drag coeff` on purpose: under-cancel, never over-cancel, since over-cancel
is positive feedback and runs away (see memory `accel-drag-ff-runaway`). It's lateral only,
with no `+K·v` on the vertical/throttle axis (that would fight the throttle floor / rocket).
Measured: a sustained +6 for 6 s ramps velocity linearly to 35.76 m/s (avg accel 5.96, so
drag is fully cancelled up to 36 m/s); park hold pins lat velocity at −2.89 m/s flat for
30 s, with fwd drift ±0.03 m and yaw ±0.02°, so nothing runs away on any axis.

### §6 Feedback: delayed-reference I, P disabled
`KP_LAT = KP_UP = 0`. With the overdrive FF driving the plant, a plain P on `a_meas` fights
the FF and rings. A gentle I (`KI = 2 /s`) instead trims the small steady residual (FF alone
lands ~5–6% low; I pulls it to <1% over ~0.5 s). The error is formed against a 15 ms-delayed
copy of the FF reference (`ACCEL_FB_DELAY`, ring buffer `:116-125`), because `a_meas` lags
the command by about that much; comparing against the undelayed reference would read a false
error during every rise. Anti-windup freezes I at the rails.

### §7 Independent grader
Truth is the rigid body's own `dv/dt`, computed in `main.gd:_grade()` and never put in the
state packet, so it's independent of the IMU/EKF signal the controller closes on. In sim
both ultimately derive from Godot truth, so this validates the FF/trim math (including
zero-command drift) rather than hardware bias rejection — that needs the Packet-B noise
model.

### Calibrated constants (1.0 kg provenance)
All calibrated for the sim's 1.0 kg body (the 7-03 MASS-1.0 recal; memory
`accel-overdrive-40ms`). The values below are authoritative, taken from `mode_accel.cpp`:

| constant | value | meaning / how to re-derive |
|---|---|---|
| `ACCEL_LAT_MAX_MSS` | 38.0 | realised m/s² at `set_lateral(1.0)`; measure peak lat accel at full canted stick, 1.0 kg |
| `ACCEL_LAT_MAX_MSS_NEG_A/B` | 40.6 / 0.25 | empirical negative-rail LUT (canted mixer's −gain is nonlinear) |
| `ACCEL_LAT_CMD_MSS` | 12.0 | accel at full lateral stick (also the balance→stick 1:1 map, Packet D) |
| `ACCEL_HOVER_THR` | 0.096 | throttle for zero net vertical accel; = MASS·g / full-thrust |
| `ACCEL_VERT_THR_PER_MSS` (up/down) | 0.0132 / 0.0124 | Δthrottle per m/s² about hover, asymmetric (concave thrust curve) |
| `ACCEL_MOTOR_TAU_LAT / _UP / _LAT_K` | 0.028 / 0.037 / 0.0015 | effective plant lag the FF inverts (≈ sim `MOTOR_TAU` + mixer); lat τ grows with |cmd| |
| `ACCEL_FF_TAU` | 0.008 | chosen reference-rise constant → sets overdrive `τ/FF_TAU` and noise gain |
| `ACCEL_FB_DELAY` | 0.015 | reference delay ≈ measured lag of `a_meas` behind the command |
| `ACCEL_DRAG_K_LAT` | 0.05 | lateral drag-cancel gain, ≤ true drag coeff (never over-cancel) |
| `ACCEL_KI_LAT / _UP` | 2.0 / 2.0 | steady-residual trim; `KP = 0` |

**Sync:** all constants are byte-identical to
`~/03_Projects/ardupilot/ArduCopter/mode_accel.cpp`. The only intended divergence is the I/O
binding: native reads typed `AccelInput`/`AhrsState`, while firmware reads `rc().channel()`
plus `AP::ahrs()` and carries two RC-encoding constants (`PEND_RATE_MAX`, `PEND_ANG_CH`) the
typed path doesn't need.

---

## Appendix — Sensor + latency layer (reconstructed by Packet B)

This is harness-only (`main.gd:_sensed_state`, `:_sensor_setup`) and never ships to the H743;
it is "the hardware." It degrades the state `ModeAccel` reads with per-channel Gaussian noise
plus measurement/EKF latency (ring buffers), so the controller is validated on representative
sensing. The grader (`_grade`, `:456` / `:746`) stays on truth: realised-vs-commanded accel is
scored from the Jolt body's own dv/dt, independent of what the controller saw.

### The load-bearing idea: latency, not amplitude, sets the limit
The pendulum has a right-half-plane pole at √(g/L), and its error e-folds on a timescale `1/p`.
A feedback loop can stabilise it only if the total effective loop delay is far below `1/p` — a
fraction of it, not merely `<`, because a delay equal to `1/p` lets the error grow by `e` before
the correction lands, and there's uncounted phase lag (controller dynamics, finite accel-loop
bandwidth) on top. Noise amplitude doesn't enter this; it only sets control chatter. That's why
this packet is about the latency budget and why σ-tuning is cosmetic.

Why σ is cosmetic in this harness: on real hardware σ and latency are coupled — a noisier sensor
forces you to low-pass harder, and the filter's group delay eats the margin (worst on the arm-rate
channel, which sits on the RHP loop). This harness deliberately treats σ and latency as orthogonal
knobs, with no filter converting one into the other. So here, sweeping σ moves the margin by 0 and
only `SENSOR_LAT_*` touches it. The real σ↔latency coupling is a hardware-transfer concern,
recorded below rather than simulated.

### Per-channel model
1σ Gaussian per channel (SI), read back through a per-group latency ring (depth = ms/2.5 at 400 Hz).

| channel | σ (1σ) | latency | source / rationale |
|---|---|---|---|
| gyro (body) | 0.05 °/s + bias walk 0.5 °/s/√hr | 2.5 ms | MEMS gyro, raw INS at loop rate |
| accel_ef | 0.35 m/s² | 2.5 ms | ~5 mg sensor + **~30 mg vibration RMS** folded into one σ (the real unknown) |
| attitude (quat) | 0.1° | 8 ms | EKF3 attitude output-observer lag |
| velocity NED | 0.03 m/s | 20 ms | EKF velocity (IMU dead-reckon) |
| position NED | 0.02 m + random-walk 0.05 m/√s | 20 ms | **IMU-only** dead reckoning — no GPS/vision (short flights) |
| arm-1 angle θ₀ | 0.15° | **8 ms** | 2×IMU fused attitude: drift-bounded, maneuver-coupled |
| arm-1 rate θ̇₀ | 0.4 °/s | 2.5 ms | differential gyro — clean; **the RHP-critical channel** |
| arm-2 **relative** φ=θ₂−θ₁ | 0.10° + LSB 0.05° | 2.5 ms | magnetic encoder (~13-bit) *over the joint*; link2 absolute = θ₁(IMU)+φ, so it **inherits link1's 0.15° / 8 ms** |
| arm-2 rate θ̇₁ | 1.0 °/s | 2.5 ms | differentiated encoder, noisier |

**Env knobs:** `USE_SENSOR_NOISE`, `SENSOR_SEED`, `SENSOR_POS_DRIFT`,
`SENSOR_LAT_EKF_{ATT,VEL,POS}_MS`, `SENSOR_LAT_ARM_IMU_MS`.

**Arm-sensing rationale (hardware):** the central tube is hollow (structural), so no magnetic
encoder fits the arm-1 root; instead we sense arm-1 angle with two IMUs relative to the body IMU.
A gyro integrates to unbounded angle drift, so the accel gravity vector has to anchor it, and that
anchor sits under acceleration (specific force = gravity + linear accel), worst during the catch.
Two accels (common-mode/differential) recover a maneuver-tolerant reference at the cost of fusion
latency (hence the 8 ms on θ₀). The IMU is strong on rate (clean differential gyro, 2.5 ms), and
that rate is exactly the RHP-critical term. Arm 2 uses a magnetic encoder (clean, quantized).

### Delay budget vs the RHP pole
- Single pendulum: `p = √(g/L) = √(9.81/0.6) = 4.04 rad/s → 1/p = 247 ms`.
- Double pendulum: fastest unstable pole `≈ 7.2 rad/s → 1/p = 139 ms`.
- Effective θ-loop delay (series around the stabilising loop):

  | term | delay |
  |---|---|
  | arm-1 angle sense (IMU fusion) | 8 ms |
  | accel realisation (**native**, Packet A) | ~15 ms |
  | compute + one-tick ZOH | ~2.5 ms |
  | **effective** | **~25 ms** |

  That gives a margin of about 10× (single) / 5.6× (double), comfortably `≪ 1/p`.

**The actuator dominates.** The ~15 ms accel-realisation lag is ~60% of the budget; every sensor
latency (2.5–20 ms) is secondary, and the 20 ms EKF vel/pos sits on the slower cart-centering
loop rather than the RHP-critical θ path. So if the double-arm margin ever proves tight, the lever
is the accel loop / reaction FF, not the sensors. (Note that concept #3's "~40 ms" is the retired
bridge figure: the RC-pipeline latency was a harness artifact, and native Packet A measured ~15 ms,
which supersedes it here.) The empirical margin break-test — adding latency until balance fails —
needs an RHP pole in the loop, which only exists once the balance controller closes it, so it lives
in Packet D, not here.

### Assumptions to revisit at hardware bring-up
1. **Arm-sensing hardware is undecided.** The σ / LSB / latency above are defensible mid-grade
   guesses (2×IMU arm 1, magnetic encoder arm 2). Re-derive them once the parts are chosen.
2. **Velocity drift** is folded into the position random-walk; there's no explicit velocity-drift
   channel. If Packet D turns out velocity-sensitive, add one, or add optical flow (a velocity
   sensor, hitting the IMU's weakest channel) rather than a position sensor.
3. **Arm-2 absolute angle inherits arm-1's error** — now modelled (2026-07-11, for the series
   chain). The encoder is a relative joint measurement (φ = θ₂−θ₁); link2's absolute angle/rate are
   reconstructed as θ₁(IMU, 8 ms) + φ(encoder, 2.5 ms), sharing the same θ₁ sample, so link2 carries
   link1's ~0.15° noise and 8 ms latency (`main.gd:_sensed_state`). The effect: honest sensing about
   doubles the double-pendulum reaction-FF error (lat 0.53→0.85 RMS, seed 1), so sensor quality bites
   here.
4. **Maneuver-coupled arm-1 angle error** (the accel gravity anchor tilting under catch accel) isn't
   modelled; only its extra fusion latency (8 ms) is. This is the first thing to add if Packet D
   balance turns out fragile, since the error is a transient angle bias correlated with the very
   maneuver that's stabilising the pole.

---

## Appendix — Packet C (reaction-force FF validation, parallel arms, superseded)

> Superseded by "Appendix — Packet C (series double inverted pendulum)" below. This section
> validated the FF against two parallel arms in an early `main.tscn`; the project's real target is a
> series chain, so the reaction FF was generalised and re-validated. Kept for the record.

Validates the arm reaction-force feed-forward (`mode_accel.cpp:199-218`): a swinging arm should barely
perturb the drone's commanded acceleration. Open-loop arm, no balance controller. Native path, clean
(truth arm state, `USE_SENSOR_NOISE` off, to isolate the FF from sensing; Packet B covered noise).

### The FF, one line
Each arm's massless rod (a two-force member, so the force acts along the rod) applies a hinge reaction
`T = m_b·[(g − a_d)·û + L·θ̇²]`: a weight/effective-gravity term (`(g − a_d)·û`, which tilts with the
drone's own commanded accel) plus a centripetal term (`L·θ̇²`, a pure-swing pull that exists even at
zero drone accel). The FF Smith-predicts θ,θ̇ forward by `PEND_PREDICT_S`, projects `T` onto lat/up, and
adds the negative to the thrust command (`w = a_d − g` in code). The pivot is at the CoM, so it's pure
translation.

### Method
Env `REACTION_FF=0` (`main.gd`) passes `0` as the arm-count to `set_command`, so the controller's
reaction-FF loop is skipped while the physical arms (built from `NUM_ARMS`) keep swinging. Same plant,
same swing, only the compensation toggles, so the FF's effect is isolated in the truth-based grader
(`_grade`, body dv/dt, independent of the IMU signal the loop closes on). The AUTOTEST profile
self-excites the swing (sustained lat ±6 for 6 s each, giving ~31° swing). Five conditions:
`NUM_ARMS={0,1,2}` × `REACTION_FF={1,0}`. Run: `USE_NATIVE=1 AUTOTEST=1 NUM_ARMS=n REACTION_FF=b godot
--headless --path sim-v-1 res://main.tscn`; grade with `sitl/autotest/analyze.py`.

### Result — RMS / max tracking error (m/s², over the profile)
| condition | lat RMS/max | up RMS/max | zero-cmd resid lat/up | analyzer flags |
|---|---|---|---|---|
| 0 arms (baseline) | 0.26 / 0.90 | 0.20 / 0.47 | 0.22 / 0.08 | 0 |
| **1 arm, FF ON** | **0.26 / 0.89** | **0.20 / 0.69** | 0.20 / 0.17 | **0** |
| 1 arm, FF OFF | 0.50 / 1.79 | 0.46 / 1.24 | 0.43 / 0.48 | 2 |
| **2 arm, FF ON** | 0.61 / 2.25 | 0.25 / 1.06 | 0.30 / 0.20 | **0** |
| 2 arm, FF OFF | 0.76 / 3.42 | 0.64 / 1.60 | 0.65 / 0.86 | 2 |

- 1 arm: the FF makes the arm invisible — FF-on RMS equals the 0-arm baseline exactly (0.26/0.20).
- FF-on ≫ FF-off on every axis, at both arm counts. FF-off roughly doubles the lat and up error and
  about doubles the zero-command residual (how hard a free swing shoves the uncommanded drone), and it
  trips 2 analyzer error-flags versus 0 for FF-on. The vertical benefit is extra-large because the FF
  also carries the arm's static weight `m_b·g` that the bare-calibrated hover throttle misses.
- Acceptance (degrades minimally vs 0 arms; FF-on ≫ FF-off): PASS.

### The prediction horizon is stale (bridge-era)
The 2-arm FF-on lateral (0.61) didn't fully return to baseline and overshot (settled +6 → 6.20),
which is over-correction, not gain droop. `PEND_PREDICT_S = 0.042 s` is the retired UDP bridge's
~40 ms round-trip; the native loop is ~17 ms (Packet A actuator ~15 ms + compute/ZOH ~2.5 ms; truth
arm state adds no sense delay here). Re-running FF-on at 0.017 s:

| case | 42 ms | **17 ms** | baseline |
|---|---|---|---|
| 2 arm lat RMS/max | 0.61 / 2.25 | **0.29 / 0.82** | 0.26 / 0.90 |
| 1 arm lat RMS | 0.26 | 0.27 | 0.26 |

At the native horizon the 2-arm lateral halves and returns to baseline; 1 arm is unchanged (its
reaction is small enough that over-prediction barely bit). I reverted to 0.042, since adopting a new
value is a capstone-affecting design decision. The recommendation is to set `PEND_PREDICT_S` to the
native regime for Packet D. It's regime-dependent — ~17 ms matches the clean loop, ~25 ms matches the
noisy capstone (Packet B adds ~8 ms arm-angle IMU-fusion latency) — so pick it with the Packet-D noise
model in the loop. Either one beats 42 ms.

### Constants sanity check
| constant | value | check |
|---|---|---|
| `PEND_MASS_RATIO` | 0.10 | = bob 0.1 kg / drone 1.0 kg (`main.tscn`), checks out |
| `PEND_L` | 0.60 m | = Bob offset `(0,−0.6,0)`; the rod is a mesh with no collision, hence massless, so the point-mass model is exact, checks out |
| `PEND_PREDICT_S` | 0.042 s | stale (bridge); native ~17–25 ms, see finding |
| pivot vs CoM | ≈0 offset | Drone CoM at origin (`center_of_mass_mode=1`, mass 1.0); `Pivot` hinge at world origin, so it sits at the CoM, the reaction is pure translation (no attitude torque), and the FF's lat/up-only scope is justified, checks out |

---

## Appendix — Packet C (series double inverted pendulum)

Supersedes the parallel-arm appendix above. Validates the reaction FF for the project's real target: a
series double inverted pendulum, with link2 hinged to link1's tip and the cart (drone) acceleration the
only input. Open-loop chain, no balance controller. Native path, clean truth (noise off).

### Scene
`main.tscn` is converted to the series chain: `Pivot2` now hinges `Arm2` to `Arm`'s tip (previously both
arms went to the drone). Hanging chain, L1 = L2 = 0.60 m, bobs 0.1 kg each, drone 1.0 kg. (For the
capstone, adopt the balance LQR's L1=0.7 / L2=0.6, a one-line `PEND_L1/L2` change.)

### The reaction (the load-bearing derivation)
The chain loads the drone through link-1's root hinge only. Isolate the chain (both bobs) and apply
Newton's 3rd law: the reaction on the cart is the chain's total (weight − inertial reaction):

```
    R = Σ_i m_i (g_vec − a_bob_i)
```

`a_bob2` rides on link-1's motion, the coupling the parallel sum missed. The two links' angular
accelerations come from the coupled double-pendulum EOM under effective gravity `g_eff = g_vec − a_d`
(û⊥ = the per-link tangent, Δ = θ1−θ2):

```
    (m1+m2)L1 θ̈1 + m2 L2 cosΔ θ̈2 = (m1+m2)(g_eff·û1⊥) − m2 L2 sinΔ θ̇2²
    L1 cosΔ θ̈1 +        L2 θ̈2 =        (g_eff·û2⊥)     + L1 sinΔ θ̇1²
```

The FF adds `−R/m_d` to the thrust command. At `m₂=0` this collapses exactly to the old
single-pendulum along-rod form (verified in algebra and in-sim). Code: `mode_accel.cpp` reaction block.

### Validated against truth
`REACT_VALIDATE=1` (`main.gd:_react_validate`) computes the true reaction from each bob's own
finite-differenced acceleration (each arm's CoM is its bob) and compares it to the formula re-derived
from (θ, θ̇, a_d), which is identical math to the C++. Over a full flight the formula matches truth to
4.7 % lateral / 3.1 % vertical (true reactions ~1.5 / ~2.4 m/s²); the residual is
finite-difference/discretisation, not structure. The coupled derivation is correct.

### Result — RMS / max tracking error (m/s², over the profile)
| condition | lat RMS/max | up RMS/max |
|---|---|---|
| 0 arms (baseline) | 0.26 / 0.90 | 0.20 / 0.47 |
| 1 link, FF ON | 0.29 / 0.83 | 0.22 / 0.69 |
| 1 link, FF OFF | 0.50 / 1.79 | 0.46 / 1.24 |
| **2 link (double), FF ON** | **0.53 / 2.02** | **0.44 / 1.63** |
| 2 link (double), FF OFF | 1.18 / 4.62 | 1.00 / 2.99 |

- Single link returns to the 0-arm baseline (the coupled FF reduces to it), verified.
- Double pendulum: the FF cuts the disturbance ~2.2× (lat 1.18→0.53, up 1.00→0.44; peaks roughly
  halved). The residual is ~2× the no-arm baseline, since the double pendulum is genuinely harder
  (faster unstable pole, larger reactions) and this is open-loop with no delay compensation. Acceptance
  (FF ≫ off; bounded; derivation matches truth): PASS.

### Smith prediction — deferred to Packet D
Predicting the chain forward (`PEND_PREDICT_S > 0`) helps lateral (0.53→0.40) but destabilises the
vertical (up RMS 0.44→0.82 at 18 ms; spikes ~9 m/s² at 10 ms): near aligned links (Δ≈0, common while
hanging) the coupled EOM stiffens and prediction amplifies `θ̈`. Left at `PEND_PREDICT_S = 0`, which is
stable on both axes. Tune it with the real loop and balance controller closed (Packet D), not open-loop
here.

### Packets A and B are unaffected
- A validated the 0-arm accel response: no pendulum, so topology is irrelevant.
- B is a sensor-degradation layer — per-channel noise and latency on whatever θ/θ̇ channels are fed —
  and it's topology-independent; it already anticipated the series case (the arm-2 encoder is a relative
  joint measurement). No rework.

---

## Appendix — Packet D (capstone: balance on the real ModeAccel)

Closes the full loop: balance controller → real native `ModeAccel` (overdrive FF + drag-cancel + coupled
reaction FF) → Jolt plant (real hinged series arm) → Packet-B noisy/delayed sensors → balance. The idealized
first-order cart of `balance_test` (τ_cl = 8 ms, no overshoot, a kinematic cart the arm can't push) is gone; the
balance loop now closes on the real inner layer, with the RHP pole in the loop for the first time.

### Wiring (`BALANCE_ONBOARD=1`, `main.gd`)
- Env-gated mode reusing `_service_native`. Builds `ArmBalance` (1 link) / `ArmBalance2` (2 links, 0.7/0.6 gains).
- **Bring-up.** Teleport the drone to `BALANCE_ALT` (10 m) at rest and level, re-anchor `Pivot`/`Pivot2`, and erect
  the arm(s) inverted with a small lean (`BALANCE_INIT_DEG`, default 4°): the `balance2_test` IC on the real body.
- **Command map.** `lat_stick = clamp(a_cmd / 12, ±1)` (`ACCEL_LAT_CMD_MSS` = 12, a clean 1:1).
- **Convention.** This is the usual bug-magnet: `_update_pendulum` gives θ from straight-down, but the controller
  wants straight-up: `θ_up = wrap(π − θ_down)`, and the rate flips sign, `θ̇_up = −θ̇_down`. The same sensed
  measurement feeds both the balance loop and the reaction FF, re-expressed per convention, not sensed twice.
- **Vertical.** Altitude-hold PD `up_stick = clamp((2·(alt_ref − alt) − 2·v_y) / g, ±1)`; see finding 2.
- **Grader on truth.** Metrics use the arm's own basis angle, independent of the sensed signal the loop closed on.

### Results (15 s, geometry 0.7/0.6, native path, altitude held)
| case | link1 max / final | link2 max / final | rms_u | sat |
|---|---|---|---|---|
| single, noise off | 4.00 / 0.013° | — | 0.21 | 0 |
| single, noise on | 4.00 / 0.038° | — | 0.22 | 0 |
| double, noise off | 5.02 / 0.005° | 4.00 / 0.002° | 0.37 | 0 |
| double, noise on | 5.07 / 0.15° | 4.00 / 0.09° | 0.62 | 0 |
| double + noise, **MC 8 seeds** | **8/8 PASS**, worst final 0.19° | ≤ 0.14° | ~0.6 | 0 |

The arm never diverges past its start lean and settles to < 0.2°, never saturating, which matches the
idealized-cart `balance2_test` (§9: 5.1 / 4.0 max, < 0.4° final) essentially exactly. The real inner loop holds
the double inverted pendulum as well as the assumed 8 ms τ_cl model does. Capstone acceptance: PASS.

### Finding 1 — the reaction FF's two jobs have different criticality (resolves FF-on/off)
The reaction FF bundles two things: (a) lateral dynamic-reaction cancellation and (b) vertical arm-weight carry.
Turning the FF fully off (`REACTION_FF=0`) with `up_stick=0` crashed the drone, but the `[bal]` trace showed the
arms staying upright (th1, th2 → 0) while altitude sank 10 → 0: the failure was the uncarried 0.2 kg arm weight, a
vertical fall, not a loss of lateral balance. With the arm weight carried by the altitude-hold PD instead, FF-off
balances identically to FF-on (double + noise: max 5.09 vs 5.08, final 0.148 vs 0.163), holding even at a 0.6 m/s
bump into saturation. So:
- Lateral reaction cancellation is performance, and near-redundant with the balance loop: the loop closes on the
  arm angle, which the reaction moves, so it rejects the uncancelled reaction as one more bounded disturbance.
- Vertical weight carry is stability-critical (staying airborne), and is better done by an explicit altitude hold.
- The reaction FF still earns its keep on the accel-tracking spec (Packets A/C), which is a different objective
  than balance. For balance specifically, altitude-hold plus the balance loop suffice.

### Finding 2 — `up_stick = 0` is a zero-accel hold, not altitude hold
ModeAccel's `up_stick = 0` commands zero vertical acceleration, which permits a constant-velocity sink (any small
initial vertical velocity persists, since the I-trim nulls accel, not velocity). The drone has to actively hover
while balancing (unlike the fixed-height kinematic cart), so the balance mode adds a gentle altitude-hold PD
(kp = kd = 2, truth alt / v_y), and altitude then holds ~10 m over the run.

### Finding 3 — empirical RHP delay margin (the break-test Packet B deferred here)
Sweeping arm-sensing latency to failure (double + noise, altitude held so the break is purely lateral):
- **Angle only:** survives > 160 ms. The angle isn't the RHP-critical channel; the clean rate channel
  (θ̇, 2.5 ms) carries the pole stabilization, so delaying only the angle just makes centering sluggish
  (final 0.16 → 0.23°).
- **Angle + rate together** (`SENSOR_LAT_ARM_IMU_MS` = `SENSOR_LAT_ARM_RATE_MS`, a shared fusion latency): PASS to
  40 ms (straining, final 0.39°, rms_u 2.63), and FAIL at 55 ms.

The break lands at ≈ 47.5 ms arm-sense + ~22.5 ms (actuator ~15 + estimator filter ~5 + ZOH ~2.5) ≈ 70 ms total
effective delay = p·τ ≈ 0.5 (p = 7.2 rad/s, 1/p = 139 ms). Textbook, the RHP-pole limit is p·τ < 1 (a hard wall)
with a practical margin of ~0.3–0.5, so the loop breaks right at the practical edge. Nominal effective delay is
~25–30 ms (p·τ ≈ 0.2), giving ~2.5× delay margin and confirming the arithmetic budget empirically. This sharpens
Packet B: what kills the loop is the arm rate-channel latency (the θ̇ phase lead), not the angle. (`PEND_PREDICT_S`
isn't the lever here; it's a reaction-FF Smith horizon, orthogonal to the balance loop's own feedback.)

### vs Phase 8
The idealized model (`balance_test`/`balance2_test`, pure-C++ plant) held the single arm to ~50 ms and the double
to ~25 ms measurement latency. The real capstone loop — with the actual ~15 ms actuator, its overshoot, the
coupled reaction, and Packet-B sensing — breaks the double at ~70 ms total effective delay and matches the
idealized balance quality at nominal. The separation was legitimate: the real inner loop lives inside the outer
loop's delay budget with ~2.5× margin, and the overshoot the idealized model lacked never mattered (it rings
out-of-band, ~10× above the pendulum's pole).

### Deferred / debt
- `s, v` fed to the balance loop are truth world-X (the slow cart-centering loop, off the RHP path); sensed-`s,v`
  is a one-line upgrade.
- Maneuver-coupled arm-angle bias (Packet-B assumption #4) is still unmodelled, and it's the first thing to add if
  hardware balance proves fragile (it's a transient angle bias correlated with the very catch that stabilizes the
  pole).
- `PEND_PREDICT_S` closed-loop tuning wasn't pursued (the loop passes with margin, and it's the wrong lever for
  balance).
- Packet E (retune) wasn't triggered: the real loop held what the idealized model held.
- 850 g / hardware recalibration stays out of scope (all of the above is at the sim's 1.0 kg body).

---

## Appendix — Stretch goal: series triple inverted pendulum (isolated sandbox)

An exploratory extension beyond Packets 0–E: a series triple (L1=0.7, L2=L3=0.6 m, bobs 0.1 kg) in an
isolated sandbox, so the validated 2-arm path (`mode_accel.*`, `main.gd`, `main.tscn`) stays byte-untouched.
Separate first, merge if it survives (user's call). New files: `mode_accel3.{h,cpp}` (`ModeAccel3`, 0..3-link
reaction; n≤2 byte-identical to `mode_accel`), `three_arm_balance_controller.*` + `ArmBalance3` (8-state LQR),
`gdext_flightstack3.*` (`ApFlightStack3`), `native/tools/compute_lqr3.py`, `sim-v-1/main3.{tscn,gd}`.

### Feasibility (`compute_lqr3.py`)
- **Controllable, rank 8/8.** Series equal-length links don't degenerate (parallel ones would: different chain
  positions give distinguishable mode shapes, so one input sees all three modes).
- **Fastest unstable pole p = 9.90 rad/s → 1/p = 101 ms** (double: 7.2 / 139 ms). τ/3 ≈ 34 ms.
- Cross-checked two ways: the 8-state plant self-checks against `compute_lqr2` (n=2 → 7.2 rad/s + exact
  gains), and against the user's independent fixed-base 3×3 `A` (trace 140, det 22409, ω²={98.0,35.6,6.4}),
  which lands on the same λ_max. Fixed-base gives τ; the 8-state adds controllability and gains (cart input).
- **LQR gains ~3–8× the double** (k2≈+1640, k3≈−1127): ~0.4° on link 2 alone saturates the ±12 rail.

### Reaction FF (3×3 coupled EOM)
Generalised `R = Σ mᵢ(g − a_bobᵢ)` with the coupled triple EOM (symmetric mass matrix
`Aᵢⱼ = Lᵢ Lⱼ μ_max(i,j) cos(θᵢ−θⱼ)`, `μⱼ = Σ_{k≥j} mₖ`, explicit 3×3 solve). Reduces exactly to n≤2.
`REACT_VALIDATE` against truth (bobs' own dv/dt): 3.3 % lat / 2.8 % up (Packet C double was 4.7 / 3.1). Correct.

### Balance (`ArmBalance3` on `ModeAccel3`, 15 s)
| case | link max (deg) | final (deg) | rms_u | sat |
|---|---|---|---|---|
| clean, 0.5° lean | 0.70 / 0.56 / 0.50 | ~0.001 | 0.07 | 0 |
| **noise on, 0.5°** | **1.22 / 0.88 / 0.67** | **~0.2–0.3** | **2.17** | 0 |

Balances both clean and under the 3-encoder sensor chain at nominal delay. Chatter `rms_u` runs 0.07→2.17
(30×, the big-gain × 0.15° sensor amplification), but stays bounded and non-fatal.

### "Very tight," measured
- **Delay break** (shared arm angle+rate latency, noise on, 0.5°): PASS ≤ 6 ticks (15 ms), FAIL ≥ 7 ticks
  (17.5 ms), so the break is ≈ 16 ms injected, ~38 ms total effective delay, p·τ ≈ 0.38. Nominal is ~25 ms
  (p·τ≈0.25), giving ~1.5× margin (the double had 2.5×). The user's τ/3 = 34 ms was the right call; the
  double's measured p·τ=0.5 is too optimistic for the triple, since three modes stack phase lag. The rate
  channel is what breaks it (as in Packet D).
- **Catch basin.** A uniform lean survives ≥ 8°, but that's the gentle mode (the gains cancel: −601+1640−1127
  = −89). The zigzag mode (θ1+, θ2−, θ3+) commands ~38×/deg, so it's sub-degree. A swing-up catch lands near
  zigzag, so the practical catch basin is sub-degree.

### Run it
- Balance (F5 default): `godot --path sim-v-1 res://main3.tscn`
- Headless balance: `BALANCE_ONBOARD=1 NUM_ARMS=3 [USE_SENSOR_NOISE=1] godot --headless --path sim-v-1 res://main3.tscn`
- Reaction FF check: `AUTOTEST=1 REACT_VALIDATE=1 NUM_ARMS=3 godot --headless --path sim-v-1 res://main3.tscn`
- Delay sweep: add `SENSOR_LAT_ARM_IMU_MS=n SENSOR_LAT_ARM_RATE_MS=n`

### Verdict
It technically works: it balances the triple on the real inner loop, clean and noisy, at nominal delay. But it's
genuinely tight, with ~1.5× delay margin (against 2.5× for the double) and a sub-degree catch basin. Merging it
into the main path (generalising `mode_accel` to n-arm) is deferred pending a decision; a swing-up to the triple
would be brutal (sub-degree catch, near the zigzag mode).
