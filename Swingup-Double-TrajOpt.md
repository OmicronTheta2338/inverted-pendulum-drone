# Double inverted pendulum: swing-up via trajectory optimization

> Handoff spec for a fresh context. The single-arm swing-up already works end-to-end on the real
> stack (see "Current state"). This doc covers only the double, which needs trajectory optimization;
> energy pumping was tried and provably flails. Read it cold and drive it to "runs properly" (both
> links up, caught, held under noise) without re-discovering anything.

## Status: done (2026-07-12). Whole ladder A→E passes.
iLQR trajectory optimization, TVLQR tracking, and an ArmBalance2 catch. On the real native ModeAccel stack
with the sensor-noise model, the double swings up from hanging, gets caught at ~3.54 s, and holds both
links at ~0.1° upright for >13 s. That's a PASS across 6/6 `SENSOR_SEED`s (1–6). Actuator lag didn't force
re-tuning (the doc's "energy-loss" worry): TVLQR feedback rejects it, unlike open-loop energy pumping. The
catch lands right at the 8° gate, and ArmBalance2 pulls it to <0.3°.

**Files built:**
- `native/tools/swingup2_traj.py` — numpy-only model plus self-tests, iLQR, CSV/header export.
  Model self-test: energy conserved to 4e-8 (u=0), upright eigenvalues match `compute_lqr2` to 5e-10.
- `native/tools/swingup2_nom.{npz,csv}` — the canonical optimized trajectory (versioned).
- `native/src/swingup2_traj_data.h` — generated header (nominal U*, X*, gains K) compiled into SwingUp2.
- `native/src/swingup2_controller.{h,cpp}` — SwingUp2, rewritten as replay+TVLQR+gate+catch (was an energy pump).
- `sim-v-1/swingup2_test.{tscn,gd}` — idealized double-cart harness (rungs A/C/D): `MODE=openloop|tvlqr`.

**Reproduce end-to-end:**
```fish
# 1. optimize (numpy iLQR) -> trajectory npz+csv
python3 native/tools/swingup2_traj.py ilqr T=4.0 dt=0.02 wf_ang=500 wf_rate=60 \
    wq_pos=1.0 wq_rate=0.05 seed_lean_deg=0 save=native/tools/swingup2_nom.npz
# 2. bake into the C++ header, then build
python3 native/tools/swingup2_traj.py header npz=native/tools/swingup2_nom.npz out=native/src/swingup2_traj_data.h
scons -C native -j(nproc)
# 3a. idealized-cart validation (rung C open-loop diverges near top; rung D tvlqr catches):
env SWINGUP2_CSV=native/tools/swingup2_nom.csv SWINGUP2_MODE=tvlqr SWINGUP2_HEADLESS=1 \
    godot --headless --path sim-v-1 res://swingup2_test.tscn
# 3b. THE run target (real stack, sensor noise):  PASS
env USE_NATIVE=1 NUM_ARMS=2 SWINGUP=1 USE_SENSOR_NOISE=1 SENSOR_SEED=1 \
    godot --headless --path sim-v-1 res://main.tscn
```
To retune the swing-up, edit the iLQR weights (step 1), regenerate the header (step 2), and rebuild.
`wq_rate` sets how gentle the trajectory is (0.05 tamed link2's peak whip, 869→335°/s); `wq_pos` sets
cart drift.

## Goal / definition of done
From both links hanging, swing the series double pendulum up and balance it upright, on the real native
ModeAccel stack (`sim-v-1/main.tscn`, `NUM_ARMS=2`) under the sensor-noise model, holding ≥ 10 s. Done
means the existing 6-state LQR (`ArmBalance2`) catches and holds both links near upright after the
swing-up, and the headless grader prints `PASS` across a few `SENSOR_SEED`s.

Run target (this must eventually PASS):
```fish
env USE_NATIVE=1 NUM_ARMS=2 SWINGUP=1 USE_SENSOR_NOISE=1 SENSOR_SEED=1 \
    godot --headless --path sim-v-1 res://main.tscn
```

## Current state — reuse this, don't rebuild
- **Single-arm swing-up: done and working on the real stack.** This is the template to imitate.
  - Portable C++ `native/src/swingup_controller.{h,cpp}` + `gdext_swingup.{h,cpp}` (class `SwingUp`):
    2-D energy pump + catch gate, composing `ArmBalanceController` for the catch.
  - Wired in `sim-v-1/main.gd`: env `SWINGUP=1`, `_swingup_setup()` (arm hanging), a branch in
    `_service_native` driving both `lat_stick` (= a_lat/12) and `up_stick` (= a_up/g).
  - `env USE_NATIVE=1 NUM_ARMS=1 SWINGUP=1 godot --headless --path sim-v-1 res://main.tscn` → catches
    ~3.3 s, balances, PASS under noise (pump gain `k=3`, env `SWINGUP_GAIN`).
  - Idealized-cart dev harness: `sim-v-1/swingup_test.{tscn,gd}` — has basin-mapping mode
    (`SWINGUP_BASIN=1`) and a native-controller mode (`SWINGUP_NATIVE=1`).
- **Double swing-up attempt (`SwingUp2`, `native/src/swingup2_controller.{h,cpp}`): built, but flails.**
  Total-energy pump `a = k(E−E_top)·P̂` (P = Σmᵢvᵢ, total bob momentum). Energy reaches `E_top` but flows
  into the internal (links-opposing) chaotic mode; the closest simultaneous both-up over k∈[0.3,12] was
  ~54°, never near the gate. Don't chase energy pumping for the double. The root cause is mode selection:
  pivot translation forcing excites both the fundamental and internal modes, and scalar energy shaping
  can't select one. Keep the file; its kinematics, config, gate, and handoff scaffolding are reusable.
- **Catch controller (reuse as-is):** `native/src/two_arm_balance_controller.{h,cpp}` (`ArmBalance2`),
  6-state LQR `u = −Kx`. `Measurement2` = {θ1, ω1, θ2, ω2, s, v}, angles from straight up, absolute
  (θ2 not relative), `a_max=12`. Gains from `native/tools/compute_lqr2.py`. The balance capstone proved it
  holds both links from a near-upright start, so the catch half isn't the problem; only reaching a
  catchable state is.

## The model (trajectory opt lives or dies on this)
The simplification the whole approach rests on: a double pendulum on a pivot that accelerates with
`(aₓ, a_y)` is exactly a standard fixed-pivot double pendulum in a uniform effective-gravity field
```
g_eff = ( −aₓ , −(g + a_y) )
```
because each bob feels a uniform pseudo-acceleration `−(aₓ, a_y)` plus the real `(0, −g)`. So take any
textbook double-pendulum EOM and swap the gravity vector for `g_eff` (which depends on the controls). That
turns a scary coupled derivation into "textbook double pendulum + variable gravity direction."

- **Params:** L1 = 0.7, L2 = 0.6 m; m1 = m2 = 0.1 kg point-mass bobs, massless rods; g = 9.8.
- **Inputs (bounds):** `aₓ ∈ [−12, +12]` m/s² (full `lat_stick`); `a_y ∈ [−g, +g]` (full `up_stick`, the
  vertical axis, a genuine second actuator that modulates `g_eff`).
- **State for the swing-up dynamics:** `x = [θ1, ω1, θ2, ω2]`. The link angles depend on pivot
  acceleration, not cart position. Optionally augment `[xc, ẋc, yc, ẏc]` and cost their excursion to
  bound drift (the single-arm used explicit centering; here you bake it into the trajectory cost).
- **Angle convention:** θ from straight down (0 = hanging, π = up), absolute for both links. Matches the
  sim sensing (`_update_pendulum`) and reuses the single-arm's `up = wrap(π − θ)`, `rate_up = −ω`.
- **Bob kinematics** (already in `swingup2_controller.cpp`, pivot frame): bob1 = L1(sinθ1, −cosθ1),
  bob2 = bob1 + L2(sinθ2, −cosθ2); velocities add the pivot velocity.
- **Model validation, mandatory** (project culture; cf. `_react_validate` in `main.gd`): before you
  optimize anything, drive the same `(aₓ(t), a_y(t))` through both your numpy integrator and Godot (add a
  scripted-input replay mode to the sim, e.g. reading a CSV) and confirm θ1(t), θ2(t) match. Godot/Jolt is
  ground truth; a wrong model wastes the whole optimization.

## Approach: iLQR (numpy-only) → TVLQR tracking → LQR catch
scipy isn't installed here (numpy is), so don't depend on it. `compute_lqr2.py` already solves the
Riccati equation in pure numpy (Hamiltonian eigen-method), and iLQR and TVLQR are likewise pure linear
algebra. You can `pip install scipy` and use `scipy.optimize` direct collocation instead, which is a fine
alternative, but numpy-only iLQR keeps the project dependency-free and matches the existing toolchain.

1. **Dynamics + validation.** New `native/tools/swingup2_traj.py`: `f(x, u)` from the effective-gravity
   double pendulum; RK4 integrator; finite-difference Jacobians `A_t = ∂f/∂x`, `B_t = ∂f/∂u`. Validate
   against Godot (above) first.
2. **iLQR** (same script): terminal cost = both links up at rest (θ → π, ω → 0); running cost = small `u`
   respecting the box (input squashing or penalty) plus optional cart-drift. Initialize from something
   feasible-ish (e.g. a few energy pumps). Output the nominal `(x*(t), u*(t))` and feedback gains `K(t)`.
3. **TVLQR** around the nominal (discrete backward Riccati, numpy) → time-varying `K(t)`. Open-loop replay
   won't survive the real stack's lag + noise into the ~6–8° basin, so tracking is required.
4. **Export + transfer.** Bake `(u*(t), x*(t), K(t))` into a data file / generated header; add a
   `replay+TVLQR` mode to `SwingUp2`: `u = u*(t) − K(t)(x̂ − x*(t))`, clamped to the rails; hand off to
   `ArmBalance2` once inside the measured double basin (see below). Convert `u` → sticks exactly like the
   single-arm branch: `lat_stick = aₓ/12`, `up_stick = a_y/g`.
5. **Real-stack caveat.** The real ModeAccel loop loses energy the idealized model doesn't, which is why
   the single-arm needed pump gain k 1 → 3. Expect to re-validate or re-tune the nominal against the
   realized accel and/or lean on TVLQR to reject it. Budget for it.

## Measure the double basin first (sets the catch target + gate)
Mirror the single-arm basin mapping (`swingup_test.gd` `_basin_setup` / `_basin_tick`, `SWINGUP_BASIN=1`):
spawn both links inverted at chosen `(θ1, ω1, θ2, ω2)` offsets, run `ArmBalance2`, and grade recover-vs-drop
over a grid. Reuse the Godot pivot-kick trick (Gotchas). That gives the true catchable region, so the
swing-up target and the handoff gate come from data rather than the ~6° folklore. Current double gate
defaults in `main.gd`: `catch = (8°, 40°/s)`, via env `SWINGUP_CATCH_ANG`, `SWINGUP_CATCH_RATE`.

## Interfaces / files (exact)
- **Inject controls:** `sim-v-1/main.gd` `_service_native`, the `if swingup_onboard:` branch, which already
  branches `n_arms>=2 → SwingUp2.update(th0,td0,th1,td1,s,v,h_dev,vy,dt)`. th0/th1 = sensed arm angles
  (from down); s,v,h_dev,vy = cart truth (slow loop). Sticks: `lat_stick = a_lat/12`, `up_stick = a_up/g`.
- **Setup / grading:** `_swingup_setup()` (arms hanging, teleport to `swingup_alt`); `_swingup_record()`
  (grades truth `_link_up_deg(i)` for both links, headless auto-exit). Env: `SWINGUP`, `SWINGUP_GAIN`,
  `SWINGUP_CATCH_ANG/RATE`, `SWINGUP_KS/KV`, `SWINGUP_INIT_DEG`, `SWINGUP_ALT`.
- **Build:** `scons -C native -j$(nproc)` → `sim-v-1/bin/libarmbalance...so`. `Glob("src/*.cpp")` auto-picks
  new files; register new Godot classes in `native/src/register_types.cpp`. One caveat: `native/SConstruct`
  was patched to strip `-static-libgcc -static-libstdc++` (no 64-bit static libstdc++ on this box), so keep it.
- **Sensor model / conventions:** `_sensed_state` (`USE_SENSOR_NOISE=1`, `SENSOR_SEED`); angle from down
  `atan2(rod.x, -rod.y)`, up = `wrap(π − θ)`, rate flips sign; link2 absolute.
- **Precedents:** `native/tools/compute_lqr2.py` (numpy CARE + the linearized 6-state A, B); the single-arm
  swing-up (`swingup_controller.*`) as the working structural template.

## Validation ladder (in order; each is cheap and catches errors early) — all passed
- **A** passed: numpy EOM matches Godot. Self-tests hit energy 4e-8 and eig match 5e-10, and the
  idealized-cart open-loop replay tracks Jolt to ~1.6° over 3 s of aggressive swinging before the
  (expected) top divergence. The exact EOM was already in `main.gd _react_validate` (validated vs Jolt
  3–5%), so I reused it verbatim as `f_cont`.
- **B** passed: iLQR finds a swing-up: both links → up (0.1°/1.3°) at rest, drift 0.5 m, `|u|`=(6.3,6.2) ≪ rails.
  8-state (cart augmented) so drift is bounded; `wq_rate` tames the outer-link whip.
- **C** passed: open-loop replay in the Godot idealized cart diverges near the top (unstable upright) → TVLQR needed.
- **D** passed: TVLQR (`u=U*+K·(x−X*)`) tracks into the 8°/40°/s gate on the idealized cart (catch @3.51 s), ArmBalance2 holds.
- **E** passed: real stack (`NUM_ARMS=2 SWINGUP=1 USE_SENSOR_NOISE=1`, seeds 1–6) → PASS (catch @3.54 s, hold <0.3°, >13 s). Done.

**Gotcha that bit (new):** the idealized double-cart harness snapped the pendulum on tick 1 (arm rate
→ −3600°/s) when the drone was offset from the scene's joint anchors, or when the hinges were re-anchored.
Fix: sit the kinematic cart at the origin in the scene's natural hanging rest pose and don't re-anchor
the hinges (the working single-arm harness does this). The trajectory then starts at exact hanging
`[0,0,0,0]`, and iLQR breaks symmetry on its own. On the real stack the default 5° lean is fine; TVLQR absorbs it.

**Sign gotcha (new):** iLQR exports K with the convention `u = U* + K·(x−X*)` (its forward-pass sign).
The doc's "u* − K dx" folds the sign into K, so use `+` with the exported gains or the feedback comes out
positive (destabilizing). That initially made TVLQR worse than open-loop.

## Gotchas (all hit while building the single arm; don't rediscover)
- **Godot initial rate:** to launch a hinged arm at a rate, set both `angular_velocity` and the matching
  CoM `linear_velocity = ω × r`, on the first `_physics_process` tick (not in `_ready`), or the hinge
  solver nulls it. See `swingup_test.gd` `_basin_setup`.
- **GDScript typing:** `var x := -d[i]` on an untyped `Array` element is a parse error (the whole script
  fails to load → the scene silently does nothing). Cast it: `float(d[i])`.
- **Headless runs:** Godot ignores SIGTERM and block-buffers piped stdout. Use `timeout -s KILL`, redirect
  to a file, and grep the file. A per-tick runtime error freezes sim time (it never quits), so advance `t`
  at the top of `_physics_process` as a safety, so it always terminates and flushes its log.
- **Real-stack energy loss:** expect the real loop to under-drive vs the idealized model (the single-arm
  needed ~3× the pump gain), and the double's tracking has to hold up under it.

## Pointers
`Balance-Controller-Design.md` §9 (double plant + LQR), `Native-Flightstack-Plan.md`, `compute_lqr2.py`.
Auto-memories (`~/.claude/projects/.../memory/`): `swingup-mode-progress`, `godot-rigidbody-pivot-kick`,
`scene-topology-native-vs-balance`, `packetD-capstone-validated`.
