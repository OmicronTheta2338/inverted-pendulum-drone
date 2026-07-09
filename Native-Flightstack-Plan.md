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
| Sensor-noise layer | **Scaffolded, off** — `main.gd:_sensed_state()` (`:146-177`, `USE_SENSOR_NOISE=1`) | Enable, tune, validate the **latency budget** (Packet B) |
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
Packet 0 (baseline + git)  ─┬─►  Packet A (accel, no arms) ─┐
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

## Packet A — Native accel loop, **no arms**  *(Task 3a + finish Task 1)*

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

## Packet B — Sensor-noise + latency layer (harness)  *(Task 2)*

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

## Appendix — SPEC reconstruction (filled in by Packet A)

> `mode_accel.cpp` references a spec that has no file. Packet A reconstructs it here:
> response target (~40 ms to 5%), the inverse-motor overdrive FF rationale, the
> drag-cancel constant-velocity property, and the calibrated constants with their
> 1.0 kg provenance. *(To be written.)*
