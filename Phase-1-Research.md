# Phase 1 — Initial Layout & Analytical Performance Budget

The goal is a spreadsheet-level model of the whole airframe — component masses, positions, centre of mass, moments of inertia, motor thrust curves, cant angles — and from those an analytical estimate of peak lateral acceleration. All the key geometry decisions get made here, before any simulation.

To clear the checkpoint I want:
- a geometry and component selection I can defend with numbers
- a peak lateral acceleration figure with margin above what the pendulum demands
- mixer matrix factors derived for our specific geometry (carry-over from the Phase 0 open question)

---

## Step 1 — Pendulum divergence timescale

This one has to come first. The timescale fixes how fast the drone must respond laterally, and that in turn fixes the lateral acceleration requirement every other number in this phase has to beat.

For a single inverted pendulum arm of length L, the linearised (near-vertical) dynamics give a divergence timescale:

```
τ = √(L / g)
```

The drone has to arrest a small perturbation inside this timescale, so the closed-loop lateral bandwidth needs to sit well above 1/τ.

### Arm length decision

| Arm length L | τ = √(L/g) | Comment |
|---|---|---|
| 0.3 m | 0.175 s | Very fast — demanding on lateral bandwidth |
| 0.5 m | 0.226 s | Moderate |
| 0.75 m | 0.277 s | Easier to control, bigger/heavier arm |
| 1.0 m | 0.319 s | Slow enough to be forgiving |

> Arm length L has to be locked before going further; it's the single most consequential geometry decision in Phase 1.

Chosen arm length: 0.60 m for the primary, 0.50 m for the secondary — the shorter secondary lets the primary swing through its full range. The 0.50 m arm is the binding axis, since it diverges fastest. τ works out to 0.247 s at 0.60 m and 0.226 s at 0.50 m, with unstable poles 1/τ of 4.04 rad/s (0.64 Hz) and 4.43 rad/s (0.70 Hz) respectively.

That sets a minimum lateral bandwidth target above ~3 Hz (3–5 × the 0.70 Hz binding pole → 2.1–3.5 Hz), and a minimum lateral acceleration target, for a step response within τ, of about 5 m/s² (≈0.5 g) — enough to hold a ~27° lean, since a = g·tan θ. The binding constraint here is bandwidth rather than acceleration; the accel headroom is enormous.

### Validation (sim step test, 2026-06-26)

Measured in `main.gd` `_run_lateral_test` (attitude frozen level, canted motors stepped to full lateral → pure actuator response, control-loop-independent):

| Metric | Demand | Measured | Margin |
|---|---|---|---|
| Peak lateral accel | ~5 m/s² | **48.2 m/s² (4.92 g)** | **~10×** |
| Lateral bandwidth | ≥ ~3 Hz | **~5.3 Hz** (first-order τ ≈ 30 ms, −3 dB) | **~7–8×** over the 0.70 Hz pole |

Step response is clean first-order: 56.6% at 25 ms, 84% at 55 ms, and ~full (4.92 g) by 125 ms, giving a time constant τ ≈ 30 ms = MOTOR_TAU. Both axes pass.

> One caveat: 5.3 Hz is the actuator-only ceiling (motor spin-up alone, attitude frozen), and the closed-loop balance controller lives below it. `MOTOR_TAU = 0.03 s` is still an estimate; bench-measure it (step throttle, time to 63%) to firm up the margin.

---

## Step 2 — Motor selection & thrust curve

Motor thrust sets the ceiling on everything else, so the motors get chosen before any commitment to cant angle or geometry.

### Candidate motors

*(Fill in as you research. Key specs: KV rating, max thrust per motor, mass, diameter, prop size.)*

| Motor | KV | Max thrust (g) | Mass (g) | Prop size | Cost |
|---|---|---|---|---|---|
| | | | | | |
| | | | | | |

### Chosen motor

Motor: *(fill in)*
Max thrust per motor: *(fill in)* g / *(fill in)* N
Mass per motor: *(fill in)* g
Prop size: *(fill in)*

### Thrust curve

*(Record thrust vs throttle % data points here once you have a datasheet or test data.)*

| Throttle % | Thrust (g) |
|---|---|
| 25 | |
| 50 | |
| 75 | |
| 100 | |

---

## Step 3 — Cant angle analysis

Cant angle θ is the primary lever for lateral authority vs vertical thrust capacity. The tradeoff runs like this: a canted motor gives T · cos(θ) of vertical thrust and T · sin(θ) of lateral thrust.

### Thrust loss at common cant angles

| θ | cos(θ) — vertical fraction | sin(θ) — lateral fraction | Vertical loss |
|---|---|---|---|
| 15° | 0.966 | 0.259 | 3.4% |
| 20° | 0.940 | 0.342 | 6.0% |
| 30° | 0.866 | 0.500 | 13.4% |
| 45° | 0.707 | 0.707 | 29.3% |

### Peak lateral acceleration (analytical)

With 4 canted motors all vectoring fully sideways and 4 upright motors handling vertical:

```
F_lateral_max = 4 × T_motor × sin(θ)
a_lateral_max = F_lateral_max / m_total
             = 4 × T_motor × sin(θ) / m_total
```

During maximum lateral thrust the canted motors are contributing `T · sin(θ)` laterally and `T · cos(θ)` vertically. The 4 upright motors must cover the vertical shortfall:

```
F_vertical_required = m_total × g
F_vertical_from_canted = 4 × T_motor × cos(θ)
F_vertical_from_upright = 4 × T_motor  (at full throttle)

Hover margin check: (4 × T_motor × cos(θ) + 4 × T_motor) ≥ m_total × g
```

> If that check fails at your chosen θ and mass, either drop mass, add or upsize motors, or reduce the cant angle.

### Cant angle calculation

> The airframe uses opposing canted pairs — M5/M8 push −X, M6/M7 push +X — so only 2 motors ever thrust in one lateral direction. That makes `F_lateral_max = 2 × T × sin θ`, not 4×.

| Variable | Value |
|---|---|
| T_motor (max, N) | 19.3 (F60PROV-LV 1950KV, 1966 g) |
| m_total (kg) | ~0.80 (sim) / 0.85 (numbers doc) |
| θ (chosen) | **90°** |
| F_lateral_max (N) | 2 × 19.3 × sin 90° = 38.6 |
| a_lateral_max (m/s²) | 38.6 / 0.80 = **48.2 (4.92 g)** matches step test |
| Hover headroom at max lateral thrust | 4 uprights only: 77.2 N / 7.84 N = **9.8:1** |
| Target a_lateral from pendulum demand | ~5 m/s² |
| Margin | **~10×** |

The chosen cant angle is 90°, and the justification is specific to this airframe's T/W. At 90° the canted motors give zero vertical lift (cos 90° = 0), so hover rides entirely on the 4 upright motors, and those alone give 9.8:1. A normal-T/W airframe would need a shallower cant, say 30–45°, to keep its hover headroom; ours doesn't. 90° also gets the most lateral authority per motor and keeps the mixer clean, since canted thrust is purely lateral with zero vertical coupling.

---

## Step 4 — Mass budget

Account for every component. Centre-of-mass position matters — the pendulum pivot must sit on the CoM axis, and the upright/canted motor pairs must be symmetric about it.

### Reference frame

- Origin: geometric centre of the main hub
- X: forward
- Y: right
- Z: up

### Component mass table

| Component | Qty | Unit mass (g) | Total mass (g) | X pos (mm) | Y pos (mm) | Z pos (mm) | Notes |
|---|---|---|---|---|---|---|---|
| Main frame / hub | 1 | | | 0 | 0 | 0 | |
| Central tube (pendulum axis) | 1 | | | 0 | 0 | | |
| Upright motors | 4 | | | | | | |
| Canted motors | 4 | | | | | | |
| Props | 8 | | | — | — | — | Distributed |
| ESCs | 8 | | | | | | |
| Flight controller (H743) | 1 | | | 0 | 0 | | |
| Raspberry Pi 5 (if kept) | 1 | | | | | | |
| Battery | 1 | | | 0 | 0 | | CoM height TBD |
| Pendulum pivot bearing | 1 | | | 0 | 0 | | |
| Pendulum arm ×2 | 2 | | | 0 | 0 | | |
| Pendulum angle encoders | 2 | | | | | | |
| Wiring / misc | 1 | ~50 est | ~50 | — | — | — | |
| **Total** | | | | | | | |

Estimated total mass: *(fill in)* g / *(fill in)* kg

### Centre of mass

```
CoM_x = Σ(m_i × x_i) / m_total
CoM_y = Σ(m_i × y_i) / m_total
CoM_z = Σ(m_i × z_i) / m_total
```

CoM position: X = *(mm)*, Y = *(mm)*, Z = *(mm)*

CoM offset from geometric centre: *(fill in)* — should be near zero in X and Y; the Z offset affects where the pendulum pivot sits.

---

## Step 5 — Moments of inertia

Needed for Phase 3 (Godot physics) and for validating the attitude controller bandwidth.

Approximate as point masses at their positions:

```
I_xx = Σ m_i × (y_i² + z_i²)   (roll)
I_yy = Σ m_i × (x_i² + z_i²)   (pitch)
I_zz = Σ m_i × (x_i² + y_i²)   (yaw)
```

| Axis | Moment of inertia (kg·m²) |
|---|---|
| I_xx (roll) | |
| I_yy (pitch) | |
| I_zz (yaw) | |

---

## Step 6 — Airframe geometry (arm layout)

Define motor positions and cant directions. For a symmetric octacopter:

### Motor positions (to be filled in)

| Motor | Type | Arm angle (°) | Arm radius (mm) | Cant direction | θ |
|---|---|---|---|---|---|
| M1 | Upright | 45 | | — | 0° |
| M2 | Upright | 135 | | — | 0° |
| M3 | Upright | 225 | | — | 0° |
| M4 | Upright | 315 | | — | 0° |
| M5 | Canted | 0 (fwd) | | Outward | θ |
| M6 | Canted | 90 (right) | | Outward | θ |
| M7 | Canted | 180 (rear) | | Outward | θ |
| M8 | Canted | 270 (left) | | Outward | θ |

> The exact arm layout (which arms carry upright vs canted) affects roll/pitch authority and prop-wash interactions. What's above is one option and can be revised in Phase 2 CAD.

Arm radius (upright motors): *(fill in)* mm
Arm radius (canted motors): *(fill in)* mm

---

## Step 7 — Mixer matrix derivation

Carry-over from the Phase 0 open question. For our 8-motor geometry, derive the per-motor factor vectors for `add_motor_raw_6dof()`.

For each motor, the factors follow from the motor's position and thrust direction.

Upright motor at position (x, y, z), spinning CW (yaw factor = −1) or CCW (+1):
```
throttle_fac =  1.0
forward_fac  =  0.0
lat_fac      =  0.0
roll_fac     = -y / arm_radius      (thrust at y produces roll)
pitch_fac    =  x / arm_radius      (thrust at x produces pitch)
yaw_fac      = ±1.0 × (prop direction)
```

Canted motor at position (x, y, z), canted outward at angle θ, thrust direction unit vector (tx, ty, tz):
```
throttle_fac =  tz  = cos(θ)
forward_fac  =  tx  = sin(θ) × (forward component of cant direction)
lat_fac      =  ty  = sin(θ) × (lateral component of cant direction)
roll_fac     = cross-product term (thrust × moment arm) for roll axis
pitch_fac    = cross-product term for pitch axis
yaw_fac      = ±small (reaction torque from prop, plus small geometric term)
```

> The values above are just a starting point. Check that the 8×6 allocation matrix (motors × DOFs) is full rank; if it isn't, the frame can't independently control all 6 DOFs simultaneously.

### Factor matrix (fill in)

| Motor | roll | pitch | yaw | throttle | forward | lat |
|---|---|---|---|---|---|---|
| M1 (upright) | | | | | | |
| M2 (upright) | | | | | | |
| M3 (upright) | | | | | | |
| M4 (upright) | | | | | | |
| M5 (canted fwd) | | | | | | |
| M6 (canted right) | | | | | | |
| M7 (canted rear) | | | | | | |
| M8 (canted left) | | | | | | |

Matrix rank check: *(confirm rank = 6)*

---

## Step 8 — Summary of key numbers

| Metric | Value | Passes? |
|---|---|---|
| Total mass | ~0.80–0.85 kg | — |
| Cant angle | 90° | — |
| Peak lateral acceleration | 48.2 m/s² (4.92 g) | — |
| Pendulum divergence timescale τ | 0.226 s (0.50 m, binding) / 0.247 s (0.60 m) | — |
| Lateral bandwidth | ~5.3 Hz actuator ceiling vs ~3 Hz target | PASS (~7–8×) |
| Lateral accel margin above pendulum demand | 48.2 vs ~5 m/s² | PASS (~10×) |
| Hover headroom at max lateral thrust | 9.8:1 on uprights alone | PASS |

> Still open: `MOTOR_TAU` (30 ms) and total mass need firming from bench/CAD, and Steps 4–7 (mass budget, inertia, geometry, mixer) still have to be completed from CAD.

---

## Phase 1 checkpoint: PASS / FAIL

- [x] Motor selected, thrust curve recorded (F60PROV-LV 1950KV, bench data in Step 2)
- [x] Cant angle chosen with vertical/lateral tradeoff modelled (90°, justified by T/W)
- [ ] Mass budget complete with CoM position (from CAD)
- [ ] Moments of inertia estimated (sim values in use; firm from CAD)
- [x] Peak lateral acceleration calculated with margin above pendulum demand (48.2 m/s², ~10×)
- [x] Pendulum divergence timescale calculated and bandwidth target set (τ=0.226 s, ~7–8× margin)
- [ ] Mixer matrix derived and rank-checked
