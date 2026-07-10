# Native Flight Stack — Migration & Validation Plan

Retire the ArduPilot SITL bridge; validate the native (in-process) flight stack
end-to-end. Segmented into self-contained **work packets** so each can be executed in
its own context window. Read the *Context* + *Load-bearing concepts* sections once;
then a packet section is enough to work that packet alone.

---

## Context — why this exists

The old path ran real ArduPilot firmware in SITL and bridged it to the Godot plant
over UDP (`sim-v-1/main.gd:_service_sitl`). That bridge added **~20–40 ms of
dead-time** (async UDP round-trip + AP's `AP_RCProtocol_FDM` RC pipeline) — fatal for
an unstable balance loop. The fix already largely exists: mode 30 (`ModeAccel`) is
ported to a Godot-free C++ core (`native/src/mode_accel.*`) behind an `ApFlightStack`
GDExtension that `main.gd` runs **in-process** with zero bridge latency
(`main.gd:_service_native`, env `USE_NATIVE=1`).

This is **not** a from-scratch build. State of the four original goals:

| Goal | State | Remaining |
|---|---|---|
| Portable C++ control layer | **Done** — `ApFlightStack` wired in | Git-track it; hand-sync with firmware |
| Fast/precise accel mode | **Implemented** — overdrive FF, delayed-fb I, drag-cancel, 0/1/2-arm reaction FF in `mode_accel.cpp` | Validated vs the *bridge*, not the native path → re-validate (Packets A, C) |
| Sensor-noise layer | **Validated (Packet B)** — `main.gd:_sensed_state()`, `USE_SENSOR_NOISE=1`; accel loop tracks under noise, delay budget ~25 ms ≪ 139 ms | Empirical RHP break-test deferred to D (needs the pole in the loop) |
| Balancing scripts | **Pass Phase 8 — but vs an *idealized* accel model** (first-order cart, τ≈8 ms, `balance_test.gd`). Never stacked on real `ModeAccel` | Close the full loop (Packet D, capstone) |

**Intended outcome:** a fully native (bridge-free) sim where the real `ModeAccel`
realizes commanded accelerations fast and precisely, under a realistic noise+latency
model, and the balance controller closes the loop on that real inner layer for 15 s.

### Does ArduPilot add noise to copy?
Yes, but we deliberately don't copy it. AP's SITL INS (`AP_InertialSensor_SITL.cpp`)
always adds only a tiny baseline (accel σ≈0.01 m/s², gyro σ≈0.04°/s, *uniform*); the
realistic knobs (`ACC*_RND`, `GYR*_RND`, `VIB_*`) default to **off**. What AP actually
introduced that matters is **EKF3 sampling latency / fusion lag** — `ModeAccel` reads
EKF *estimates*, not raw sensors. Our harness model reproduces the *intent* (Gaussian
per-channel noise + per-group latency ring buffers) per the Phase-4 decision. Keep it
in the harness; do **not** replicate `AP_InertialSensor_SITL`.

---

## Load-bearing concepts (own these before the packets that lean on them)

1. **Acceleration-input cart abstraction.** The drone is a cart whose lateral accel is
   the control input; the linearised pendulum is then **mass-independent**
   (`θ̈ = (g/L)θ − (1/L)u`). This is *why* the balance layer and accel layer separate.
   (Balance-Controller-Design.md §2.)
2. **RHP-pole delay budget.** Unstable pole at √(g/L) ≈ 4.04 rad/s (single) / +7.2 rad/s
   (double). Stabilisable iff total *effective* loop delay ≪ 1/p (≈248 ms / ≈139 ms).
   **Latency, not noise amplitude, is the killer** — this is what Packet B is really
   about. (§5.)
3. **Inverse-motor-model overdrive FF.** How `ModeAccel` gets ~40 ms accel response:
   commands the exact inverse of the 1st-order motor lag with a saturation-aware
   reference governor (`mode_accel.cpp:292-342`). Packets A/D lean on it.
4. **Reaction-force FF.** The drone cancels each arm's hinge tension analytically,
   Smith-predicting θ,θ̇ forward ~42 ms: `a_pend=(m_b/m_d)[(a_d−g)·u − L·θ̇²]·u`
   (`mode_accel.cpp:464-491`). This is Packet C.

## Decisions locked
- **Noise lives in the GDScript harness** (`_sensed_state`), not portable C++ — it is
  "the hardware," and must never ship to the H743.
- **Target the sim's MASS = 1.0 kg for now.** The 850 g airframe (`numbers`) recal is
  **deferred** to hardware bring-up (see *Out of scope*).
- **End-to-end integration (Packet D) is the capstone.**

---

## Sequencing / dependencies

```
Packet 0 ✅ (baseline+git) ─┬─►  Packet A ✅ (accel, no arms) ─┐
                            └─►  Packet B (noise+latency) ──┼─► Packet D (integration) ─► Packet E (retune, contingent)
                                 Packet A ──► Packet C (arms) ─┘
```
A and B are independent (parallelizable). C needs A. D needs A, B, C.

---

## Packet 0 — Baseline & git hygiene  *(prerequisite)*

**Why:** the entire `native/` subsystem + `sim-v-1/armbalance.gdextension` is untracked
(`git status` → `??`). Lock a known-good baseline so every later packet has a clean diff.

**Do:**
- Branch off `main` (don't work on `main`).
- Rebuild + confirm it loads: `cd native && scons -j$(nproc)` (Fedora: add
  `use_static_cpp=no`). Output → `sim-v-1/bin/libarmbalance.*.so`.
- Smoke tests green: `native/tests/test_flightstack`, `native/tests/test_plant`
  (build cmds in each `*_main.cpp` header).
- Clean native flight: `USE_NATIVE=1 godot --path sim-v-1 res://main.tscn` hovers +
  translates.
- Commit the native tree, `.gdextension`, and the `main.gd` native path as the
  baseline. `.gitignore` already excludes `*.os`/`*.so`/`godot-cpp/`.

**Acceptance:** clean branch; extension builds; both smoke tests pass; native sim
hovers; baseline committed.

---

## Packet A — Native accel loop, **no arms**  ✅ **COMPLETE** *(Task 3a + finish Task 1)*

> **Status: COMPLETE (2026-07-10).** Native path, 0 arms: lat +6 enters the 5% band at
> ~15 ms, 0.7% overshoot, settles <1%; +12 saturates the overdrive → 3.7% overshoot
> (bang-bang regime). Drag-cancel holds accel to 36 m/s; park hold pins lateral velocity
> at −2.89 m/s flat for 30 s — no runaway. Constants byte-identical to the ArduPilot
> source (only I/O binding diverges). SPEC reconstructed → appendix below. All acceptance
> criteria met. *(memory: `packetA-native-accel-validated`)*

**Goal:** prove native `ModeAccel` realizes commanded **vertical & lateral** accel fast
and precisely on the native path (clean truth first): ~40 ms to 5%, drag-cancel →
constant velocity at zero command, no runaway.

**Key files:** `native/src/mode_accel.{h,cpp}`, `ap_attitude_control.cpp`,
`ap_motors6dof.cpp`; grader `main.gd:_grade()` (`:745-789`); `sitl/autotest/analyze.py`.

**Do:**
- Run: `USE_NATIVE=1 AUTOTEST=1 NUM_ARMS=0 godot --headless --path sim-v-1
  res://main.tscn`. Grader compares the body's own dv/dt (truth) vs commanded accel —
  independent of the signal the controller closes on. Parse `[grade]`/`[step]`/`[trace]`.
- Check rise time, overshoot, steady residual vs the spec; verify drag-cancel gives
  constant lateral velocity at zero command (no runaway — memory `accel-drag-ff-runaway`).
- **Clear SPEC debt:** `mode_accel.cpp` cites "SPEC 3 / 4.2 / 6 / §7" but **no SPEC.md
  exists**. Reconstruct a short spec (response target, FF/overdrive rationale,
  drag-cancel, calibrated constants + their 1.0 kg provenance) and add it to this doc's
  appendix, so the constants are traceable/rebuildable.
- **Sync check:** diff native `mode_accel.cpp` vs
  `~/03_Projects/ardupilot/ArduCopter/mode_accel.cpp`; record intended divergences
  (typed structs vs RC/AHRS singletons).

**Acceptance:** native path, 0 arms, hits vertical & lateral accel steps within spec;
constant-velocity at zero command; no runaway; spec written.

---

## Packet B — Sensor-noise + latency layer (harness)  ✅ **COMPLETE** *(Task 2)*

> **Result (native path):** noise layer enabled, tuned to the real sensor plan, validated.
> The 0-arm accel loop tracks *identically* with noise on vs off (mean|e| lat 0.085 vs 0.083,
> up 0.272 vs 0.268) — the grader stays on TRUTH (`main.gd:456`) and the FF-dominant loop
> attenuates the injected accel σ (0.35 m/s²) ~60× to a ~0.006 m/s² realised residual. **No
> drift** over the 50 s hold (flat ~0.006, not growing) because this loop closes no position
> feedback. Arm sensing was **split by hardware** — arm 1 = 2×IMU fusion, arm 2 = magnetic
> encoder (see appendix). 2-arm smoke stayed bounded (no runaway; full arm validation is C).
> **Delay budget:** effective θ-loop delay ≈ 25 ms ≪ 247 ms (single) / 139 ms (double) → ~10× /
> ~5.6× margin; the ~15 ms native actuator lag dominates, sensor latencies are noise in it.
> Full model + budget → appendix "Sensor + latency layer".

**Goal:** enable, tune, validate the existing `_sensed_state` model on the native path.
The point is the **latency budget** (concept #2), not amplitude cosmetics.

**Key files:** `main.gd:_sensor_setup()` (`:487-510`), `_sensed_state()` (`:516-574`),
config block (`:146-177`), `_read_ch` (`:577-579`). Env: `USE_SENSOR_NOISE`,
`SENSOR_SEED`, `SENSOR_LAT_EKF_{ATT,VEL,POS}_MS`, `SENSOR_POS_DRIFT`.

**Do:**
- Turn it on: `USE_NATIVE=1 USE_SENSOR_NOISE=1 …` and confirm the Packet-A accel loop
  still tracks (grader stays on truth by design — `main.gd:456`).
- Tune the σ's toward the real sensor plan (current values are placeholders). Arm-angle
  sensing hardware is **undecided** — pick a defensible encoder σ + LSB, mark it as a
  hardware-dependent assumption to revisit.
- Validate latency ring depths (imu 2.5 / ekf_att 8 / ekf_vel,pos 20 / arm 2.5 ms)
  against the RHP-pole budget: effective delay ≪ 1/p (248 ms single / 139 ms double).
  Sweep `SENSOR_LAT_EKF_*_MS` for the margin.
- Do **not** wire noise into the (to-be-retired) UDP bridge path.

**Acceptance:** native path runs stably with noise on; accel loop still tracks;
noise/latency parameters + delay-budget justification documented.

---

## Packet C — Arm reaction-force FF, **with arms**  *(Task 3b)*

> **Status: COMPLETE (2026-07-10).** Native path, clean (truth arm state, noise off — isolate the FF
> from sensing, which Packet B covered). Added a `REACTION_FF=0` harness toggle (`main.gd`) that tells
> the controller 0 arms while the physical arms stay attached, isolating the FF in the truth-based
> grader. **1 arm, FF-on tracks *identically* to 0 arms** (lat/up RMS 0.26/0.20 = baseline); FF-off
> doubles every error and trips the analyzer (FF-on: 0 flags). 2 arms FF-on stays bounded and beats
> FF-off, but lateral didn't fully recover to baseline **at the stale 42 ms horizon** — root-caused:
> `PEND_PREDICT_S=0.042` is the retired *bridge* round-trip; at the native ~17 ms it halves the 2-arm
> lateral error (0.61→0.29 RMS) and restores baseline. **Left at 0.042 pending a user decision**
> (capstone-affecting constant). Constants checked: ratio 0.10 = 0.1/1.0 kg, L 0.60 m, and the **pivot
> sits at the drone CoM** (`main.tscn`) so the FF's translational-only scope is geometrically justified.
> All acceptance met. See appendix + memory `packetC-reaction-ff-validated`.

**Goal:** validate the pendulum reaction FF so a *swinging* arm barely perturbs the
drone's commanded accel — open-loop arm, **no balance controller yet**.

**Key files:** reaction FF `mode_accel.cpp:464-491` (constants `PEND_MASS_RATIO=0.10`,
`PEND_L=0.60`, `PEND_PREDICT_S≈0.042`); arm state feed `main.gd:_update_pendulum()`
(`:871-887`) → `set_command(...theta,theta_dot)`.

**Do:**
- Run the accel profile with 1 then 2 free-swinging arms:
  `USE_NATIVE=1 AUTOTEST=1 NUM_ARMS=1 …` then `NUM_ARMS=2`.
- Compare drone accel tracking (grader) **FF-on vs FF-off** to show the reaction FF
  measurably reduces the arm-induced perturbation.
- Sanity-check `PEND_MASS_RATIO`/`PEND_L` vs the sim arm (0.1 kg bob, 0.6 m) and the
  ~42 ms prediction horizon vs the loop delay from Packet B.

**Acceptance:** with arms swinging, accel tracking degrades minimally vs 0-arms, and
FF-on clearly beats FF-off.

---

## Packet D — **Capstone:** balance controller on the real ModeAccel  *(Task 4)*

**Goal:** replace the idealized first-order cart with the real inner loop and close the
full loop: **balance → real `ModeAccel` → Jolt plant (real hinged arm) → noisy/delayed
sensors (Packet B) → balance.** Confirm 15 s upright.

**Key files:** new scene/mode (`flight_balance_test.{tscn,gd}` **or** an env-gated mode
in `main.gd`, e.g. `BALANCE_ONBOARD=1`) that instantiates `ArmBalance`/`ArmBalance2`
and feeds its output into `flightstack.set_command`. Reuse: `gdext_arm_balance.cpp`
(`configure`/`update`), `balance_test.gd` for angle convention + gains, `_service_native`
for state plumbing.

**Wiring gotchas:**
- **Command mapping:** balance outputs `a_cmd` in m/s² (clamped ±12); `ModeAccel` lat
  stick normalizes to `ACCEL_LAT_CMD_MSS = 12`. So `lat_stick = clamp(a_cmd/12, -1, 1)`
  — a clean 1:1. (Watch the historical set_lateral scaling bug: memory
  `lateral-throttle-scaling-bug`.)
- **Angle convention:** `_update_pendulum` gives θ from **straight-down**; the balance
  controller wants θ from **straight-up** (+→+X). Reuse the exact `balance_test.gd`
  conversion — don't re-derive.
- **Cart state:** feed `s,v` = the drone's lateral position/velocity (lateral component
  of `pos_ned`/`vel_ned`), not a kinematic cart.
- Feed the **same sensed** arm state to both the balance controller and `ModeAccel`'s
  reaction FF.

**Do:** single arm balances ≥15 s under noise, then double arm; Monte-Carlo seed sweep
like Phase 8; compare real-loop vs idealized `balance_test.gd` numbers (does the true
effective latency, vs the assumed 8 ms τ, still hold the arm?).

**Acceptance:** single then double inverted pendulum balances ≥15 s on the native path
with real `ModeAccel` + noise on, across seeds; margin characterized vs Phase 8.

---

## Packet E — Balance retune / robustness  *(contingent on D)*

**Only if D shows the real loop breaks what the idealized model held.** Retune LQR
`Q/R` (`native/tools/compute_lqr.py` / `compute_lqr2.py`), adjust estimator cutoffs
(`arm_balance_controller.h:56-68`), or tighten the accel loop; re-characterize the
delay margin. May fold into D.

---

## Global verification

- **Build:** `cd native && scons -j$(nproc)` (Fedora `use_static_cpp=no`) → `.so` in
  `sim-v-1/bin/`.
- **Pure-C++ (fast):** `native/tests/` — `test_plant`, `test_flightstack`.
- **Accel grader:** `USE_NATIVE=1 AUTOTEST=1 [NUM_ARMS=n] godot --headless --path
  sim-v-1 res://main.tscn` → `[grade]/[step]/[trace]`; post-process `analyze.py`.
- **Balance:** `godot --path sim-v-1 res://balance_test.tscn` (interactive) /
  `BALANCE_HEADLESS=1 … res://balance_test.tscn`; Monte-Carlo
  `bash native/tests/balance_mc.sh seeds 24 7.5`.
- Run `/verify` on each packet end-to-end before committing.

## Out of scope (named so they aren't lost)
- **850 g recalibration** of `MASS` + `ModeAccel` constants (hover throttle, lateral
  rail LUT, vertical slope) — deferred to hardware bring-up.
- **Retiring the UDP bridge code** (`_service_sitl`, `sitl/`) — leave until the native
  path is fully validated; keep as a fallback reference.
- **Swing-up** — balance only, as in Phase 8.
- **Portable C++ estimator / EKF-lite** — not now (noise stays in harness).

---

## Appendix — SPEC (reconstructed by Packet A)

`mode_accel.cpp` cites "SPEC 3 / 4.2 / 6 / §7" but no file existed. This is that spec,
reconstructed from the code + the Packet-A validation run. Everything here targets the
sim's **MASS = 1.0 kg** body (850 g recal deferred — see *Decisions locked*).

### §3 Response target
Commanded lateral/vertical acceleration is realised **fast, monotone, and without
overshoot**: to within 5% of a step in **~40 ms** (design ceiling), with settled error
<1%. *Measured (native, 0 arms):* lat +6 m/s² enters the 5% band at **~15 ms**, peaks
6.04 (**0.7% overshoot**), settles <1%. This is the loop the balance layer will close on,
so **overshoot in realised accel = a disturbance kick to the pendulum** — a clean monotone
rise is worth more than raw speed.

### §4.1 Feed-forward = plant inversion onto a *chosen* reference (the load-bearing idea)
The lateral/vertical actuator is a first-order lag: `ȧ = (u − a)/τ` (accel `a` chases
command `u` at rate ∝ the gap; sim `MOTOR_TAU ≈ 30 ms`). For a first-order plant, **any**
smooth output path `a_des(t)` is produced by exactly one command,

```
    u(t) = a_des(t) + τ · ȧ_des(t)          (the "+1" delivers the target; the "τ·s" is the overdrive)
```

so the command is *not unique* — the design **chooses** the output path and inverts onto
it. We choose a **first-order reference** `a_des = A·(1 − e^(−t/FF_TAU))`, `FF_TAU = 8 ms`.
That single choice generates everything:
- **Overdrive.** At the step the command leads the target by `τ/FF_TAU ≈ 4.6×`
  (`lat_ff = ref + tau_lat·(cmd − ref)/FF_TAU`, `mode_accel.cpp:88-90`). e.g. a **+6**
  command feeds **~27.75 m/s²** on the first tick, then decays to 6 with an 8 ms constant.
  This is why the response beats the motor's own ~3τ ≈ 84 ms: the `(τs+1)` premultiply
  *cancels* the motor lag, leaving the 8 ms reference we designed.
- **Why the first-order member (not a faster one).** The fast end of the family is
  minimum-time bang-bang — also pure feed-forward, also no "look and see", but its switch
  time is only right if `τ` is exactly right. The first-order FF is the **robust** member:
  forgiving of `τ` error (battery sag, motor temp, prop wear on the H743), monotone under
  model mismatch. **FF does ~95%; a small delayed I trims the last ~5%.** In the sim `τ`
  matches exactly, so the conservatism buys nothing *here* except noise/latency margin for
  Packet B — its real payoff is hardware transfer.
- **Saturation-aware reference governor** (`mode_accel.cpp:91-114`). When the demanded
  overdrive exceeds the rail (`ACCEL_LAT_MAX_MSS = 38`), clamp the command to the rail and
  advance the reference as a plant driven by the rail; an overshoot-guard snaps `ref → cmd`
  on crossing. This *is* an (involuntary) bang-bang segment: a **+12** command wants
  `0.040·12/0.008 = 60 > 38`, so it saturates → **3.7% overshoot** vs +6's 0.7%. The extra
  overshoot at +12 is the measured price of losing the exact inverse to the rail.

### §4.2 Drag-cancel → constant velocity at zero command
At zero accel command the body must **coast at constant lateral velocity**, not decay.
`a_lat_out += ACCEL_DRAG_K_LAT · v_lat` (`:150`) cancels velocity-dependent aero drag.
`K = 0.05 ≤ true drag coeff` **deliberately** (under-cancel, never over-cancel — over-cancel
is positive feedback → runaway; see memory `accel-drag-ff-runaway`). **Lateral only** — no
`+K·v` on the vertical/throttle axis (would fight the throttle floor / rocket). *Measured:*
sustained +6 for 6 s ramps velocity **linearly to 35.76 m/s** (avg accel 5.96 — drag fully
cancelled up to 36 m/s); park hold pins lat velocity at **−2.89 m/s flat for 30 s**, fwd
drift ±0.03 m, yaw ±0.02° → **no runaway on any axis**.

### §6 Feedback: delayed-reference I, P disabled
`KP_LAT = KP_UP = 0` — with the overdrive FF driving the plant, a plain P on `a_meas`
fights the FF and rings. Instead a **gentle I** (`KI = 2 /s`) trims the small steady
residual (FF alone lands ~5–6% low; I pulls it to <1% over ~0.5 s). The error is formed
against a **15 ms-delayed** copy of the FF reference (`ACCEL_FB_DELAY`, ring buffer
`:116-125`) because `a_meas` lags the command by about that — comparing to the *undelayed*
reference would read a false error during every rise. Anti-windup freezes I at the rails.

### §7 Independent grader
Truth = the rigid body's own `dv/dt`, computed in `main.gd:_grade()` and **never** put in
the state packet, so it's independent of the IMU/EKF signal the controller closes on. In
sim both ultimately derive from Godot truth, so this validates the FF/trim math (incl.
zero-command drift), **not** hardware bias rejection (that needs the Packet-B noise model).

### Calibrated constants (1.0 kg provenance)
All calibrated for the sim's 1.0 kg body (the 7-03 MASS-1.0 recal; memory
`accel-overdrive-40ms`). Values below are authoritative from `mode_accel.cpp`:

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
`~/03_Projects/ardupilot/ArduCopter/mode_accel.cpp`. Only intended divergence = I/O
binding: native reads typed `AccelInput`/`AhrsState`; firmware reads `rc().channel()` +
`AP::ahrs()` and carries two RC-encoding constants (`PEND_RATE_MAX`, `PEND_ANG_CH`) the
typed path doesn't need.

---

## Appendix — Sensor + latency layer (reconstructed by Packet B)

Harness-only (`main.gd:_sensed_state`, `:_sensor_setup`); **never ships to the H743** — it *is*
"the hardware." It degrades the state `ModeAccel` reads with per-channel Gaussian noise +
measurement/EKF latency (ring buffers), so the controller is validated on representative sensing.
The grader (`_grade`, `:456` / `:746`) stays on **truth** — realised-vs-commanded accel is scored
from the Jolt body's own dv/dt, independent of what the controller saw.

### The one load-bearing idea: latency, not amplitude, is the killer
The pendulum has a right-half-plane pole at √(g/L); its error e-folds on a timescale `1/p`. A
feedback loop can only stabilise it if the **total effective loop delay ≪ 1/p** — a *fraction*,
not merely `<`, because a delay equal to `1/p` lets the error grow by `e` before the correction
lands, and there is uncounted phase lag (controller dynamics, finite accel-loop bandwidth) on top.
Noise **amplitude** does not enter this — it only sets control chatter. That is why this packet is
about the latency budget, and why σ-tuning is cosmetics.

**Subtlety (why σ is cosmetic *in this harness*):** on real hardware σ and latency *are* coupled
— a noisier sensor forces you to low-pass harder, and the filter's group delay eats the margin
(worst on the arm-**rate** channel, which sits on the RHP loop). This harness deliberately models
σ and latency as **orthogonal** knobs (no filter converts one into the other). So here, sweeping σ
moves the margin by 0; only `SENSOR_LAT_*` touches it. The real σ↔latency coupling is a
hardware-transfer concern, recorded below, not simulated.

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
| arm-2 angle θ₁ | 0.10° + LSB 0.05° | 2.5 ms | magnetic encoder (~13-bit), clean |
| arm-2 rate θ̇₁ | 1.0 °/s | 2.5 ms | differentiated encoder, noisier |

**Env knobs:** `USE_SENSOR_NOISE`, `SENSOR_SEED`, `SENSOR_POS_DRIFT`,
`SENSOR_LAT_EKF_{ATT,VEL,POS}_MS`, `SENSOR_LAT_ARM_IMU_MS`.

**Arm-sensing rationale (hardware):** the central tube is hollow (structural), so no magnetic
encoder fits the arm-1 root → sense arm-1 *angle* by two IMUs relative to the body IMU. A gyro
integrates to unbounded angle drift, so the accel **gravity vector** must anchor it — and that
anchor lies under acceleration (specific force = gravity + linear accel), worst during the catch.
Two accels (common-mode/differential) recover a maneuver-robust reference at the cost of fusion
latency (→ the 8 ms on θ₀). The IMU is *strong* on rate (clean differential gyro, 2.5 ms) and that
rate is exactly the RHP-critical term. Arm 2 uses a magnetic encoder (clean, quantized).

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

  → **margin ≈ 10× (single) / 5.6× (double)** — comfortably `≪ 1/p`.

**The actuator dominates.** The ~15 ms accel-realisation lag is ~60% of the budget; every sensor
latency (2.5–20 ms) is secondary, and the 20 ms EKF vel/pos sits on the slower cart-*centering*
loop, not the RHP-critical θ path. So if the double-arm margin ever proves tight, the lever is the
**accel loop / reaction FF**, not the sensors. *(Note: concept #3's "~40 ms" is the retired
**bridge** figure — RC-pipeline latency was a harness artifact; native Packet A measured ~15 ms and
supersedes it here.)* The *empirical* margin break-test (adding latency until balance fails) needs
an RHP pole *in the loop* — that only exists once the balance controller closes it, so it lives in
**Packet D**, not here.

### Assumptions to revisit at hardware bring-up
1. **Arm-sensing hardware is undecided.** σ / LSB / latency above are defensible mid-grade guesses
   (2×IMU arm 1, magnetic encoder arm 2). Re-derive once parts are chosen.
2. **Velocity drift** is folded into the position random-walk (no explicit velocity-drift channel).
   If Packet D is velocity-sensitive, add one — or add **optical flow** (a velocity sensor, hitting
   the IMU's weakest channel) rather than a position sensor.
3. **Arm-2 absolute angle inherits arm-1 drift** (the encoder is a *relative* joint measurement).
   Omitted — `truth["theta1"]` is fed absolute. Add if the double-arm estimate looks fragile.
4. **Maneuver-coupled arm-1 angle error** (the accel gravity anchor tilting under catch accel) is
   *not* modelled — only its extra fusion **latency** (8 ms) is. **This is the #1 thing to add if
   Packet D balance is fragile**, since the error is a transient angle *bias* correlated with the
   very maneuver that's stabilising the pole.

---

## Appendix — Packet C (reaction-force FF validation)

Validates the arm reaction-force feed-forward (`mode_accel.cpp:199-218`): a *swinging* arm must barely
perturb the drone's commanded acceleration. Open-loop arm, **no balance controller**. Native path,
clean (truth arm state, `USE_SENSOR_NOISE` off — isolate the FF from sensing; Packet B covered noise).

### The FF, one line
Each arm's massless rod (a two-force member → force along the rod) applies a hinge reaction
`T = m_b·[(g − a_d)·û + L·θ̇²]` — a weight/effective-gravity term (`(g − a_d)·û`, tilts with the drone's
own commanded accel) plus a centripetal term (`L·θ̇²`, a pure-swing pull that exists even at zero drone
accel). The FF Smith-predicts θ,θ̇ forward by `PEND_PREDICT_S`, projects `T` onto lat/up, and **adds the
negative** to the thrust command (`w = a_d − g` in code). Pivot is at the CoM, so it's pure translation.

### Method
Env `REACTION_FF=0` (`main.gd`) passes `0` as the arm-count to `set_command`, so the controller's
reaction-FF loop is skipped while the physical arms (built from `NUM_ARMS`) keep swinging. **Same plant,
same swing, only the compensation toggles** → the FF's effect is isolated in the truth-based grader
(`_grade`, body dv/dt, independent of the IMU signal the loop closes on). The AUTOTEST profile
self-excites the swing (sustained lat ±6 for 6 s each → ~31° swing). 5 conditions: `NUM_ARMS={0,1,2}` ×
`REACTION_FF={1,0}`. Run: `USE_NATIVE=1 AUTOTEST=1 NUM_ARMS=n REACTION_FF=b godot --headless --path
sim-v-1 res://main.tscn`; grade with `sitl/autotest/analyze.py`.

### Result — RMS / max tracking error (m/s², over the profile)
| condition | lat RMS/max | up RMS/max | zero-cmd resid lat/up | analyzer flags |
|---|---|---|---|---|
| 0 arms (baseline) | 0.26 / 0.90 | 0.20 / 0.47 | 0.22 / 0.08 | 0 |
| **1 arm, FF ON** | **0.26 / 0.89** | **0.20 / 0.69** | 0.20 / 0.17 | **0** |
| 1 arm, FF OFF | 0.50 / 1.79 | 0.46 / 1.24 | 0.43 / 0.48 | 2 |
| **2 arm, FF ON** | 0.61 / 2.25 | 0.25 / 1.06 | 0.30 / 0.20 | **0** |
| 2 arm, FF OFF | 0.76 / 3.42 | 0.64 / 1.60 | 0.65 / 0.86 | 2 |

- **1 arm: the FF makes the arm invisible** — FF-on RMS equals the 0-arm baseline exactly (0.26/0.20).
- **FF-on ≫ FF-off on every axis, both arm counts.** FF-off ~doubles lat & up error, ~2× the
  zero-command residual (how hard a free swing shoves the *uncommanded* drone), and trips 2 analyzer
  error-flags vs 0 for FF-on. Vertical benefit is extra-large because the FF also carries the arm's
  static weight `m_b·g` that the bare-calibrated hover throttle misses.
- Acceptance (degrades minimally vs 0 arms; FF-on ≫ FF-off): **PASS.**

### Finding — the prediction horizon is stale (bridge-era)
2-arm FF-on lateral (0.61) didn't fully return to baseline and *overshot* (settled +6 → 6.20) —
over-correction, not gain droop. `PEND_PREDICT_S = 0.042 s` is the retired UDP bridge's ~40 ms
round-trip; the native loop is ~17 ms (Packet A actuator ~15 ms + compute/ZOH ~2.5 ms; truth arm state
adds no sense delay here). Re-running FF-on at **0.017 s**:

| case | 42 ms | **17 ms** | baseline |
|---|---|---|---|
| 2 arm lat RMS/max | 0.61 / 2.25 | **0.29 / 0.82** | 0.26 / 0.90 |
| 1 arm lat RMS | 0.26 | 0.27 | 0.26 |

At the native horizon the 2-arm lateral **halves and returns to baseline**; 1 arm is unchanged (its
reaction is small enough that over-prediction barely bit). **Reverted to 0.042** — adopting a new value
is a capstone-affecting design decision. *Recommendation:* set `PEND_PREDICT_S` to the native regime
for Packet D. It's regime-dependent — **~17 ms matches the clean loop, ~25 ms matches the noisy
capstone** (Packet B adds ~8 ms arm-angle IMU-fusion latency) — so pick it with the Packet-D noise
model in the loop. Either beats 42 ms.

### Constants sanity check
| constant | value | check |
|---|---|---|
| `PEND_MASS_RATIO` | 0.10 | = bob 0.1 kg / drone 1.0 kg (`main.tscn`) ✓ |
| `PEND_L` | 0.60 m | = Bob offset `(0,−0.6,0)`; rod is a mesh with no collision → massless, so the point-mass model is exact ✓ |
| `PEND_PREDICT_S` | 0.042 s | **stale** (bridge); native ~17–25 ms — see finding |
| pivot vs CoM | ≈0 offset | Drone CoM at origin (`center_of_mass_mode=1`, mass 1.0); `Pivot` hinge at world origin → **at the CoM**, so the reaction is pure translation (no attitude torque) and the FF's lat/up-only scope is justified ✓ |
