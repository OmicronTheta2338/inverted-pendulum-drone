# Phase 1 — Initial Layout & Analytical Performance Budget

**Goal:** Produce a spreadsheet-level model of the whole airframe — component masses, positions, centre of mass, moments of inertia, motor thrust curves, cant angles — and derive an analytical estimate of peak lateral acceleration. All key geometry decisions are made here, before simulation.

**Checkpoint criteria:**
- A geometry and component selection defensible with numbers
- A peak lateral acceleration figure with margin above what the pendulum demands
- Mixer matrix factors derived for our specific geometry (carry-over from Phase 0 open question)

---

## Step 1 — Pendulum divergence timescale

This must come first. The timescale sets the lateral response speed the drone must achieve, which in turn sets the lateral acceleration requirement that every other number in this phase must beat.

For a single inverted pendulum arm of length L, the linearised (near-vertical) dynamics give a divergence timescale:

```
τ = √(L / g)
```

The drone must be able to arrest a small perturbation within this timescale, so the closed-loop lateral bandwidth must be well above 1/τ.

### Arm length decision

| Arm length L | τ = √(L/g) | Comment |
|---|---|---|
| 0.3 m | 0.175 s | Very fast — demanding on lateral bandwidth |
| 0.5 m | 0.226 s | Moderate |
| 0.75 m | 0.277 s | Easier to control, bigger/heavier arm |
| 1.0 m | 0.319 s | Slow enough to be forgiving |

> **Decision needed:** Choose arm length L before proceeding. This is the single most consequential geometry decision in Phase 1.

**Chosen arm length:** *(fill in)*

**Divergence timescale τ:** *(fill in)*

**Minimum lateral bandwidth target:** > *(fill in)* Hz (rule of thumb: closed-loop bandwidth ≥ 3–5 × 1/τ)

**Minimum lateral acceleration target (for step response within τ):** *(fill in)* m/s²

---

## Step 2 — Motor selection & thrust curve

Motor thrust sets the ceiling on everything. Choose motors before committing to cant angle or geometry.

### Candidate motors

*(Fill in as you research. Key specs: KV rating, max thrust per motor, mass, diameter, prop size.)*

| Motor | KV | Max thrust (g) | Mass (g) | Prop size | Cost |
|---|---|---|---|---|---|
| | | | | | |
| | | | | | |

### Chosen motor

**Motor:** *(fill in)*
**Max thrust per motor:** *(fill in)* g / *(fill in)* N
**Mass per motor:** *(fill in)* g
**Prop size:** *(fill in)*

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

Cant angle θ is the primary lever for lateral authority vs vertical thrust capacity. The tradeoff:

- **Vertical thrust per canted motor:** T · cos(θ)
- **Lateral thrust per canted motor:** T · sin(θ)

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

> If this fails at your chosen θ and mass, either reduce mass, increase motor count/size, or reduce cant angle.

### Cant angle calculation (fill in once motors and mass are known)

| Variable | Value |
|---|---|
| T_motor (max, N) | |
| m_total (kg) | |
| θ (chosen) | |
| F_lateral_max (N) | |
| a_lateral_max (m/s²) | |
| Hover headroom at max lateral thrust | |
| Target a_lateral from pendulum demand | |
| Margin | |

**Chosen cant angle:** *(fill in)*

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

**Estimated total mass:** *(fill in)* g / *(fill in)* kg

### Centre of mass

```
CoM_x = Σ(m_i × x_i) / m_total
CoM_y = Σ(m_i × y_i) / m_total
CoM_z = Σ(m_i × z_i) / m_total
```

**CoM position:** X = *(mm)*, Y = *(mm)*, Z = *(mm)*

**CoM offset from geometric centre:** *(fill in)* — should be near zero in X and Y; Z offset affects pendulum pivot placement.

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

> Note: The exact arm layout (which arms carry upright vs canted) affects roll/pitch authority and prop wash interactions. The layout above is one option — can be revised in Phase 2 CAD.

**Arm radius (upright motors):** *(fill in)* mm
**Arm radius (canted motors):** *(fill in)* mm

---

## Step 7 — Mixer matrix derivation

Carry-over from Phase 0 open question. For our 8-motor geometry, derive the per-motor factor vectors for `add_motor_raw_6dof()`.

For each motor, the factors follow from the motor's position and thrust direction:

**Upright motor at position (x, y, z), spinning CW (yaw factor = −1) or CCW (+1):**
```
throttle_fac =  1.0
forward_fac  =  0.0
lat_fac      =  0.0
roll_fac     = -y / arm_radius      (thrust at y produces roll)
pitch_fac    =  x / arm_radius      (thrust at x produces pitch)
yaw_fac      = ±1.0 × (prop direction)
```

**Canted motor at position (x, y, z), canted outward at angle θ, thrust direction unit vector (tx, ty, tz):**
```
throttle_fac =  tz  = cos(θ)
forward_fac  =  tx  = sin(θ) × (forward component of cant direction)
lat_fac      =  ty  = sin(θ) × (lateral component of cant direction)
roll_fac     = cross-product term (thrust × moment arm) for roll axis
pitch_fac    = cross-product term for pitch axis
yaw_fac      = ±small (reaction torque from prop, plus small geometric term)
```

> The analytical values above are a starting point. Verify the 8×6 allocation matrix (motors × DOFs) is full rank — if it isn't, the frame cannot independently control all 6 DOFs simultaneously.

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

**Matrix rank check:** *(confirm rank = 6)*

---

## Step 8 — Summary of key numbers

*(Fill in once all steps above are complete.)*

| Metric | Value | Passes? |
|---|---|---|
| Total mass | kg | — |
| Cant angle | ° | — |
| Peak lateral acceleration | m/s² | |
| Pendulum divergence timescale τ | s | — |
| Lateral bandwidth target | Hz | — |
| Lateral accel margin above pendulum demand | m/s² | ✓ / ✗ |
| Hover headroom at max lateral thrust | % | ✓ / ✗ |

---

## Phase 1 checkpoint: PASS / FAIL

- [ ] Motor selected, thrust curve recorded
- [ ] Cant angle chosen with vertical/lateral tradeoff modelled
- [ ] Mass budget complete with CoM position
- [ ] Moments of inertia estimated
- [ ] Peak lateral acceleration calculated with margin above pendulum demand
- [ ] Pendulum divergence timescale calculated and bandwidth target set
- [ ] Mixer matrix derived and rank-checked
