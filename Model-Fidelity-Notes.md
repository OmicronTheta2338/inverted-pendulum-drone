# Model Fidelity Notes — sim vs. reality

A living list of known ways the Godot sim diverges from the real vehicle, to work
through in a proper accuracy sweep rather than patch piecemeal (compensating errors
are the trap). Seeded from the triple-pendulum sandbox investigation (2026-07-12).

## Realism pass — IMPLEMENTED on the DOUBLE (`main.gd`, 2026-07-14)

Plant now models the below (all in `main.gd`; controllers unchanged except the single-arm pump).
Double swing-up + catch + balance AND single-arm swing-up all still PASS headless under noise (seeds 1/3/5).

- **A6 quadratic body drag** — `F=-BODY_DRAG_K|v|v` at the CoM; replaces the linear `linear_damp` placeholder.
- **A3 bob air drag** — small quadratic drag on each bob (`ARM_DRAG_K`).
- **A2 hinge friction** — viscous + smoothed-Coulomb torque at each hinge (`HINGE_VISC/HINGE_COUL`); the arms
  now DECAY like a real rig, and the balance shows a realistic ~few-degree friction micro-wobble.
- **A5 rotor gyroscopics** — `τ = -Ω×ΣH_i`, `H_i=GYRO_KH·√T·spin·axis`; cancels at balanced thrust.
- **A1 distributed-mass arms** — uniform rod (`ARM_ROD_FRAC`, default 0.7) + tip mass, via custom CoM+inertia.
  **KEY FINDING:** the point-mass-tuned LQR + swing-up *absorb* the ~18% effective-length shift — the full EOM
  re-derivation cascade (`compute_lqr2.py`/`swingup2_traj.py`/C++ FF + rebuild) was **NOT** needed. Bare uniform
  rod (frac=1.0) breaks the swing-up basin; realistic 0.7 is safe. Balance survives even at 1.0.
- **C1 maneuver-coupled arm-angle bias** — small low-passed anchor-lean under lateral accel (`C1_ANCHOR_GAIN`),
  common-mode-rejected by the 2×IMU differential (so it stays small).
- **Single-arm swing-up pump** raised k=3→6 (`_swingup_setup`): the new dissipation drains swing energy that
  the idealized k=3 no longer overcomes; the double kept enough margin at 3.

**Margin watch:** each realism layer eats the swing-up cushion (steady hold 1.0° → ~6.5°, gate 15°). Still safe,
but this is the binding constraint — re-check before adding more.
**Deferred → mostly resolved 2026-07-17 (see next subsection):** A4/A7/C4 now have STRUCTURE in the sim; only
their COEFFICIENTS remain bench-measurements. A8-mass / A9-cant / B3-CoM were reclassified as DESIGN INTENT (not
gaps) — see below. True high-frequency IMU aliasing still deferred (needs a multi-rate sensor model, absent here).
**Pump (items 1–2 below):** cause CONFIRMED = forward-hold pitching ⊗ tiltable swing plane; zero arm-drag is
**not** the cause. Banked (already mitigated in both stacks). Item 1's "no air sink drives the pump" framing is
superseded — see the fidelity report (`~/.claude/plans/as-you-can-see-kind-blossom.md`).
**Triple sandbox (`main3`):** NOT yet given this realism pass — mirror it there if the triple becomes a show-off.

### Actuator / aero / sensor-spectrum pass (2026-07-17) — A7, A4, C4 structure added

User ruled three earlier "weaknesses" OUT as deliberate DESIGN INTENT, not fidelity gaps:
- **MASS=1.0 > ~0.85 kg estimate** is a chosen over-budget *margin* — tuning against a heavier plant means
  lower accel authority, so a lighter as-built vehicle only *gains* headroom. Optimistic (0.85) would be the bug.
- **CoM pinned AT the hinge** is deliberate and load-bearing: the arm's hinge-reaction force then passes through
  the body CoM ⇒ **zero body moment** ⇒ the translational-only reaction FF is *exactly* sufficient (no moment
  term needed). This is the decision that makes the FF legal; an offset CoM would reintroduce B3.
- **90° cant** is deliberate: sin90°=1 maxes the *scarce* lateral axis; cos90°=0 keeps thrust out of the
  *surplus* vertical axis. Canting less trades scarce authority for surplus. Not a bug.

Added (all `main.gd`, controllers untouched; double swing-up+catch+balance PASS seeds 1/3/5, hold_max ~6.3–6.7°):
- **A7a asymmetric motor lag** — `MOTOR_TAU` split into `MOTOR_TAU_UP=0.03` (rising, current-driven, fast) /
  `MOTOR_TAU_DOWN=0.045` (falling, freewheel drag, slow); per-motor by sign of cmd−actual. A catch reversal is
  paced by the *slower* spin-down. `= TAU_UP` if the real ESC runs active braking.
- **A7b battery sag** — all 8 motors share one pack, so **total** thrust (a proxy for total current: prop
  torque ∝ thrust ∝ motor current) droops the shared voltage: `V/Vnom = 1 − BATT_SAG_K·ΣT`, thrust ∝ V².
  A **correlated** hit on every motor at once, worst at the catch's peak demand; the voltage-blind flight code
  can't compensate. Default `BATT_SAG_K=0.0008` (good low-R pack): ~1.6% loss at hover, ~6–7% at catch peak.
  `_sag_factor` reads last tick's realized ΣT (explicit → no current↔voltage loop within a tick).
- **A4 advance-ratio thrust droop** — a fixed-pitch prop unloads as the vehicle advances along its thrust axis:
  `T *= max(0, 1 − ADV_RATIO_K·v_axial)`, `v_axial>0` only (conservative — droops, never boosts). This is at
  *fixed RPM* (aero, not electrical) ⇒ it does **not** touch the gyro angular-momentum term, which still reads
  the sagged thrust. Default `k=0.015` (~3% at 2 m/s). The opposing-lateral-prop *wash* proper stays unmodelled
  (needs CFD; the mixer never co-fires the opposing pair anyway).
- **C4 rotor-vibration tone** — the coherent line pulled OUT of the white `sig_accel` (0.35→0.25) and re-injected
  as a throttle-coupled sinusoid (amp ∝ thrust, freq ∝ √thrust RPM proxy, decorrelated phase/axis), ~power-
  preserving at hover. **Stability-inert at these params**: the ~90–180 Hz line sits above *both* the loop
  bandwidth *and* the motor-lag cutoff, so both reject it → reassuring (rotor-line vibration is safe while
  out-of-band). CAVEAT: the dangerous aliasing-*into*-band failure needs a multi-rate IMU model the 400 Hz sim
  lacks — cannot be shown faithfully here.

**Honesty note:** the margin barely moved because the defaults are *conservative* (good pack, small droop,
out-of-band tone), NOT because the loop is immune. Every coefficient is a BENCH placeholder — env overrides
`BATT_SAG_K`, `ADV_RATIO_K`, `VIB_ACCEL`. A sensitivity sweep (crank each toward the margin cliff) is the honest
next probe if we want the sim's *failure* boundary, not just its nominal pass.

## Surfaced this session

### 1. Arms have ZERO aerodynamic drag  *(deferred — do not patch in isolation)*
The pendulum rods/bobs run `linear_damp = 0`, `angular_damp = 0` — no air resistance
at all. Real bobs sweep through air at up to ~4 m/s (though edge-on, low frontal area,
so the true term is **small**).
- **Consequence:** with no air sink, ~100% of the arms' swing energy must exit through
  the drone hinge, which drives a slow pitch/yaw *parametric pump* (item 2).
- **Evidence it's the only sink:** pin the drone (`POSTREL_FREEZE_DRONE=1`) → the triple
  barely decays on a fixed pivot (wmax ~80°/s still at 120 s ⇒ Jolt's solver is
  ~energy-conserving, no hidden numerical damping). Free the drone → the arms settle ⇒
  **the drone is the sink.** So "settling" and "the pump" are the same energy transfer.
- **When swept:** add a *small* quadratic drag on the bobs. Expect it to *reduce*, not
  erase, the pump — the remainder is control-side (item 2).

### 2. Pitch↔arm parametric coupling  *(interacts with #1; partly mitigated)*
When the drone pitches θ, a sliver `R·sinθ` of the large arm reaction `R` leaks into the
fore/aft axis → the forward-hold pitches to correct → redirects more → pumps. Confirmed
by the bare-drone test (`POSTREL_FREEZE=1`): with the arms gone, pitch is dead-flat 0.0°.
Mitigated (not eliminated) by `ModeAccel3::hold_level` keeping the drone near-level so the
`sinθ` lever stays tiny.

## Carried from Native-Flightstack-Plan.md (revisit in the same sweep)
- **IMU-only position dead-reckoning drift** (~0.015 m/√s) — realistic GPS-denied; gives
  ~0.1 m / 40 s fore/aft wander (`SENSOR_POS_DRIFT=0` removes it). Real fix = optical flow.
- **Maneuver-coupled arm-angle bias** (Packet-B assumption #4) — the accel gravity anchor
  tilts under catch acceleration; only its *latency* is modeled, not the transient *bias*.
- **Sensor σ / encoder LSB** are mid-grade guesses — re-derive once arm-sensing hardware
  is chosen.
- **`MASS = 1.0 kg`** sim body vs the ~850 g airframe — recal (hover throttle, lateral
  rail LUT, vertical slope) deferred to hardware bring-up.
- **`linear_damp = 0.05`** on the drone is a placeholder linear drag coeff; real drag is
  ~quadratic and near-zero at the low speeds of actual use.

## Diagnostic toggles (in `main3.gd`, for the sweep)
- `POSTREL_FREEZE=1` — detach + freeze the arms after the drop (fly the drone bare).
- `POSTREL_FREEZE_DRONE=1` — pin the drone (triple on a fixed pivot; isolates numerical
  vs. coupling dissipation).
- `POSTREL_FF=0` — reaction FF off after the drop.
- `SENSOR_POS_DRIFT=0` — disable the dead-reckon position drift.
