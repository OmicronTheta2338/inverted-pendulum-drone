# Model fidelity: sim vs. reality

Where the Godot sim still diverges from the real vehicle, collected so I can work through
it in one accuracy sweep instead of patching things one at a time. Patch them piecemeal and
you get two errors that quietly cancel — a model that reads right for the wrong reason.
Started from the triple-pendulum sandbox work, 2026-07-12.

## Realism pass on the double (`main.gd`, 2026-07-14)

The plant now models everything below, all in `main.gd`, with the controllers untouched
apart from the single-arm pump. Double swing-up + catch + balance and single-arm swing-up
both still pass headless under noise (seeds 1/3/5).

- **A6 — quadratic body drag.** `F = −BODY_DRAG_K·|v|·v` at the CoM, replacing the linear
  `linear_damp` placeholder.
- **A3 — bob air drag.** A small quadratic drag on each bob (`ARM_DRAG_K`).
- **A2 — hinge friction.** Viscous plus smoothed-Coulomb torque at each hinge
  (`HINGE_VISC/HINGE_COUL`). The arms now decay like a real rig, and the balance shows a
  realistic ~few-degree friction micro-wobble.
- **A5 — rotor gyroscopics.** `τ = -Ω×ΣH_i`, `H_i=GYRO_KH·√T·spin·axis`; cancels at balanced
  thrust.
- **A1 — distributed-mass arms.** A uniform rod (`ARM_ROD_FRAC`, default 0.7) plus a tip
  mass, from a custom CoM and inertia. The part worth remembering: the point-mass-tuned LQR
  and swing-up absorb the ~18% shift in effective length on their own, so I never needed the
  full EOM re-derivation cascade (`compute_lqr2.py` → `swingup2_traj.py` → C++ FF → rebuild).
  A bare uniform rod (frac = 1.0) breaks the swing-up basin; the realistic 0.7 is fine, and
  balance survives even at 1.0.
- **C1 — maneuver-coupled arm-angle bias.** A small low-passed anchor-lean under lateral
  accel (`C1_ANCHOR_GAIN`), common-mode-rejected by the 2×IMU differential, so it stays small.
- **Single-arm swing-up pump** raised from k=3→6 (`_swingup_setup`): the added dissipation
  drains swing energy the idealized k=3 can no longer overcome. The double kept enough margin
  at 3.

**Margin watch.** Every realism layer eats into the swing-up cushion — steady hold went from
1.0° to ~6.5°, against a 15° gate. Still safe, but it's now the binding constraint, so
re-check it before adding anything else.

Most of what was deferred here is now resolved as of 2026-07-17, in the next subsection:
A4/A7/C4 have structure in the sim, and only their coefficients are still bench-measurements.
A8-mass, A9-cant and B3-CoM got reclassified as design intent rather than gaps (see below).
True high-frequency IMU aliasing is still deferred; it needs a multi-rate sensor model, which
isn't present here.

The pump (items 1–2 below): the cause is confirmed as forward-hold pitching combined with a
tiltable swing plane; zero arm-drag is not the cause. That's banked, and already mitigated in
both stacks. Item 1's "no air sink drives the pump" framing is superseded; see the fidelity
report (`~/.claude/plans/as-you-can-see-kind-blossom.md`).

Triple sandbox (`main3`): it hasn't had this realism pass yet. Mirror it there if the triple
becomes a show-off.

### Actuator / aero / sensor-spectrum pass (2026-07-17) — A7, A4, C4 structure added

I ruled three earlier "weaknesses" out as deliberate design intent rather than fidelity gaps:
- **MASS=1.0 > ~0.85 kg estimate** is a deliberate over-budget margin: tuning against a
  heavier plant means lower accel authority, so a lighter as-built vehicle only gains
  headroom. Going optimistic at 0.85 would be the bug.
- **CoM pinned at the hinge** is deliberate and load-bearing. The arm's hinge-reaction force
  then passes through the body CoM, so there's zero body moment, and the translational-only
  reaction FF is exactly sufficient with no moment term needed. That's the decision that makes
  the FF legal; an offset CoM would reintroduce B3.
- **90° cant** is deliberate: sin90°=1 maxes out the scarce lateral axis, while cos90°=0 keeps
  thrust out of the surplus vertical axis. Canting any less would trade scarce authority for
  surplus. Not a bug.

Added (all in `main.gd`, controllers untouched; double swing-up + catch + balance passes on
seeds 1/3/5, hold_max ~6.3–6.7°):
- **A7a — asymmetric motor lag.** `MOTOR_TAU` splits into `MOTOR_TAU_UP=0.03` (rising,
  current-driven, fast) and `MOTOR_TAU_DOWN=0.045` (falling, freewheel drag, slow), chosen
  per-motor by the sign of cmd−actual. A catch reversal gets paced by the slower spin-down. It
  collapses back to `= TAU_UP` if the real ESC runs active braking.
- **A7b — battery sag.** All 8 motors share one pack, so total thrust (a stand-in for total
  current, since prop torque ∝ thrust ∝ motor current) droops the shared voltage:
  `V/Vnom = 1 − BATT_SAG_K·ΣT`, thrust ∝ V². That's a correlated hit on every motor at once,
  worst at the catch's peak demand, and the voltage-blind flight code can't compensate. Default
  `BATT_SAG_K=0.0008` (a good low-R pack) costs ~1.6% at hover and ~6–7% at the catch peak.
  `_sag_factor` reads last tick's realized ΣT, kept explicit so there's no current↔voltage loop
  within a tick.
- **A4 — advance-ratio thrust droop.** A fixed-pitch prop unloads as the vehicle advances along
  its thrust axis: `T *= max(0, 1 − ADV_RATIO_K·v_axial)`, for `v_axial>0` only (conservative:
  it droops, never boosts). This happens at fixed RPM (aero, not electrical), so it doesn't
  touch the gyro angular-momentum term, which still reads the sagged thrust. Default `k=0.015`
  (~3% at 2 m/s). The proper opposing-lateral-prop wash stays unmodelled; it needs CFD, and the
  mixer never co-fires the opposing pair anyway.
- **C4 — rotor-vibration tone.** The coherent line pulled out of the white `sig_accel`
  (0.35→0.25) and re-injected as a throttle-coupled sinusoid (amp ∝ thrust, freq ∝ √thrust RPM
  proxy, decorrelated phase and axis), roughly power-preserving at hover. It's stability-inert
  at these params: the ~90–180 Hz line sits above both the loop bandwidth and the motor-lag
  cutoff, so both reject it, which is reassuring — rotor-line vibration is safe while it stays
  out-of-band. The caveat is that the dangerous failure, aliasing into band, needs a multi-rate
  IMU model the 400 Hz sim lacks, so it can't be shown faithfully here.

Worth being honest about: the margin barely moved because the defaults are conservative (good
pack, small droop, out-of-band tone), not because the loop is immune. Every coefficient is a
bench placeholder — env overrides `BATT_SAG_K`, `ADV_RATIO_K`, `VIB_ACCEL`. If we want the
sim's failure boundary rather than just its nominal pass, the honest next probe is a sensitivity
sweep that cranks each toward the margin cliff.

## Surfaced this session

### 1. Arms have zero aerodynamic drag  *(deferred — do not patch in isolation)*
The pendulum rods and bobs run `linear_damp = 0`, `angular_damp = 0`, so there's no air
resistance at all. Real bobs sweep through air at up to ~4 m/s, but edge-on and with low
frontal area, so the true term is small.
- With no air sink, ~100% of the arms' swing energy has to exit through the drone hinge, and
  that drives a slow pitch/yaw parametric pump (item 2).
- The evidence it's the only sink: pin the drone (`POSTREL_FREEZE_DRONE=1`) and the triple
  barely decays on a fixed pivot (wmax ~80°/s still at 120 s), so Jolt's solver is close to
  energy-conserving with no hidden numerical damping. Free the drone and the arms settle, which
  makes the drone the sink. "Settling" and "the pump" are the same energy transfer.
- When swept: add a small quadratic drag on the bobs. Expect it to reduce the pump rather than
  erase it; the remainder is control-side (item 2).

### 2. Pitch↔arm parametric coupling  *(interacts with #1; partly mitigated)*
When the drone pitches θ, a sliver `R·sinθ` of the large arm reaction `R` leaks into the
fore/aft axis → the forward-hold pitches to correct → that redirects more → it pumps. The
bare-drone test confirms it (`POSTREL_FREEZE=1`): with the arms gone, pitch is dead-flat 0.0°.
`ModeAccel3::hold_level` mitigates it without eliminating it, keeping the drone near-level so
the `sinθ` lever stays tiny.

## Carried from Native-Flightstack-Plan.md (revisit in the same sweep)
- **IMU-only position dead-reckoning drift** (~0.015 m/√s) — realistic for GPS-denied flight;
  gives ~0.1 m / 40 s of fore/aft wander (`SENSOR_POS_DRIFT=0` removes it). The real fix is
  optical flow.
- **Maneuver-coupled arm-angle bias** (Packet-B assumption #4) — the accel gravity anchor tilts
  under catch acceleration; only its latency is modeled so far, not the transient bias.
- **Sensor σ / encoder LSB** are mid-grade guesses; re-derive them once the arm-sensing hardware
  is chosen.
- **`MASS = 1.0 kg`** sim body against the ~850 g airframe — the recal (hover throttle, lateral
  rail LUT, vertical slope) is deferred to hardware bring-up.
- **`linear_damp = 0.05`** on the drone is a placeholder linear drag coeff; the real drag is
  ~quadratic and near-zero at the low speeds of actual use.

## Diagnostic toggles (in `main3.gd`, for the sweep)
- `POSTREL_FREEZE=1` — detach and freeze the arms after the drop (fly the drone bare).
- `POSTREL_FREEZE_DRONE=1` — pin the drone (triple on a fixed pivot; isolates numerical
  vs. coupling dissipation).
- `POSTREL_FF=0` — reaction FF off after the drop.
- `SENSOR_POS_DRIFT=0` — disable the dead-reckon position drift.
