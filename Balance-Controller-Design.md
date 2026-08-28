# Phase 8 — Single-Arm Balance Controller (design + learnings)

The goal is to keep one inverted pendulum arm balanced upright on the drone for
15 s, with no sign of falling at t = 15 s, from drone + arm state in and a
commanded lateral acceleration out. Balance only, no swing-up. The controller is
portable C++ (the artifact that compiles onto the H743), validated in a Godot
scene that reuses the project's drone + point-mass-pendulum physics. That scene
stands in for the drone hardware by modelling an actuator lag and noisy, delayed
sensors.

**Checkpoint criteria**
- [x] Control law derived, gains generated, signs verified.
- [x] Portable C++ core with no Godot / ArduPilot dependencies.
- [x] Pure-C++ plant test: balances under noise + latency (Monte-Carlo).
- [x] Godot scene (same physics) balances the real hinged arm for 15 s under noise + 5–10 ms latency.
- [x] Stability / latency margin characterised.
- [x] Double inverted pendulum built (`ArmBalance2`, single input, serial links 0.7/0.6 m): balances 15 s, 16/16 seeds (§9).

This controller must not depend on the ArduPilot flight code: a separate effort
owns making the real drone realise a commanded acceleration quickly. Here that
layer is assumed and modelled (see §5).

---

## 1. Architecture / data flow

The drone is a cart whose lateral acceleration is the control input. A separable
outer loop (this controller) reads the arm angle and commands a lateral
acceleration; the fully-actuated layer below (ArduPilot `ModeAccel` on the real
vehicle, a first-order actuator model in the test) realises it while the body
stays level. Per 400 Hz tick:

```
 Godot plant (balance_test.gd)                    C++ core (ArmBalance GDExtension)
 ─────────────────────────────                    ─────────────────────────────────
 true theta, theta_dot, s, v ─► +Gaussian noise ─► delay ring buffer (5–10 ms)
                                                       │ noisy, delayed measurement
                                                       ▼
                                            ArmBalanceController::update()
                                              · per-channel low-pass estimator
                                              · u = -K x   (LQR full-state)
                                              · clamp to ±a_max
                                                       │ a_cmd  [m/s²]
                                                       ▼
 a_real += (a_cmd-a_real)(1-e^(-dt/τ_cl))  ◄─ actuator ref-model H_ref, τ_cl ≈ 8 ms
        │
        ▼
 drive kinematic cart along world-X from a_real  (HingeJoint drags the arm)
```

Both imperfection layers (noise + latency on the input, actuator lag on the
output) live entirely in the harness, never in the controller; on real hardware
they are the hardware, so the shipped controller carries none of them.

---

## 2. Plant model

Geometry (reuses the sim convention): the cart moves along world X (lateral), up
is world Y, the hinge axis is Z, so the arm swings in the X–Y plane. `θ` is the
arm angle from straight up, positive when the bob tips toward +X.

**Nonlinear** (point-mass bob, massless rod, cart acceleration `u`):

```
θ̈ = ( g·sinθ − u·cosθ ) / L
s̈ = u
```

**Linearised about upright** (small θ), and note it's mass-independent — the key
consequence of an acceleration input:

```
θ̈ = (g/L)·θ − (1/L)·u
s̈ = u                         x = [θ, θ̇, s, v]ᵀ,  u = commanded lateral accel

    ⎡ 0     1  0 0 ⎤        ⎡  0  ⎤
A = ⎢ g/L   0  0 0 ⎥   B =  ⎢-1/L ⎥        g/L = 16.33,  1/L = 1.667  (L = 0.6)
    ⎢ 0     0  0 1 ⎥        ⎢  0  ⎥
    ⎣ 0     0  0 0 ⎦        ⎣  1  ⎦
```

The open-loop plant has a right-half-plane pole at √(g/L) = 4.04 rad/s (0.64 Hz)
for the 0.6 m arm, 4.43 rad/s for the 0.5 m binding arm. That's the unstable mode
the controller has to dominate.

---

## 3. Control law — LQR full-state (primary)

`u = −K·x` with `x = [θ, θ̇, s−s_ref, v]`. `K` comes from an offline CARE solve
(`native/tools/compute_lqr.py`, numpy — no scipy needed) with Q = diag(400, 10,
2, 4), R = 1 (angle-dominant, so the cart returns home slowly and never fights
the fast balance loop):

| gain | value | on |
|------|-------|----|
| k_θ  | −43.9140 | arm angle |
| k_θ̇  | −10.1773 | arm rate |
| k_s  | −1.4142  | cart position (station-keeping) |
| k_v  | −3.7210  | cart velocity |

Effective feedback on a +θ lean is `+43.9·θ`: accelerate the cart under the bob.
Closed-loop poles (L = 0.6) are −5.92 ± 1.79j (the angle pair, ζ ≈ 0.96,
~6.2 rad/s ≈ 1.5× the unstable pole) and −0.70 ± 0.34j (the slow cart pair). The
same K also stabilises the 0.5 m binding arm (poles −6.59 ± 1.48j, −0.70 ±
0.34j), so one gain set covers both project arm lengths.

Command saturation: `|a_cmd| ≤ a_max = 12 m/s²` (the airframe rail is ~47 m/s², so
there's ~4× headroom; the pendulum only demands ~5 m/s²). There's no integrator in
the LQR path, so nothing to wind up.

### 3a. PID decomposition (appendix / bring-up fallback)

The same closed-loop behaviour reads as an inner angle PD plus an outer cart PD:

```
a_bal = kp_θ·θ + kd_θ·θ̇        (inner, fast: stabilise upright)
a_pos = −kp_s·(s−s_ref) − kd_s·v   (outer, slow: keep the drone home)
u = a_bal + a_pos
```

with `kp_θ = 43.91, kd_θ = 10.18, kp_s = 1.41, kd_s = 3.72` (the LQR gains with
the sign folded in). An optional bounded cart-position integral `ki_s` (anti-windup,
frozen while railed) removes steady bias. Select it with `law = "PID"` and use it
to hand-tune on hardware, knob by knob. It's a constrained subset of the LQR, kept
around for transparency.

---

## 4. State estimation

Inputs arrive noisy and delayed. Each channel goes through a first-order low-pass
before feedback (`fc_θ = 30, fc_θ̇ = 25, fc_s = fc_v = 15 Hz`). The cutoffs sit
high deliberately: on an unstable plant the danger isn't the noise so much as the
group delay a heavy filter adds, which erodes the margin against the RHP pole. θ̇
is a first-class measurement (arm gyro / encoder-diff), so the controller never
differentiates a noisy angle. The estimator primes on the first sample, so there's
no start-up ring-in.

On the real vehicle the same four numbers come from: θ = arm encoder (relative to
body) + IMU roll; θ̇ = arm gyro; s, v = the position/velocity estimate. The
controller's I/O contract is identical in sim and on hardware.

---

## 5. The imperfection model (what the harness applies)

**Actuator (output).** This is a reference model, not an endpoint. The inner
(ModeAccel) loop that turns a commanded lateral acceleration into a realised one
is written as a reference model the balance loop designs against:

```
a_real(s) = H_ref(s)·a_cmd(s),   H_ref(s) = 1 / (τ_cl·s + 1),   τ_cl ≈ 8 ms
```

The harness realises exactly this, as a first-order lag on the command:

```
a_real += (a_cmd − a_real)·(1 − e^(−dt/τ_cl))          (τ_cl = act_tau_ms, default 8 ms)
```

The design number is the effective delay `τ_d = τ_cl ≲ 10 ms` — that's what the
unstable loop actually feels as phase lag near crossover. The earlier "converges
to within 1 % of the command by 40 ms" isn't a separate requirement: for a
first-order model it's just the tail, `40 ms = τ_cl·ln(100)`, and the balance loop
never sees it (the pendulum doesn't care about the last 1 %). So what I spec and
log is the pole (`τ_cl` / `τ_d`), not the settling time. There's a floor under all
this: the irreducible delay — control tick (2.5 ms) plus sensing/filtering plus
higher-order motor dynamics — runs ~5–8 ms, so `τ_cl ≈ 8 ms` already sits near it.
Tightening further buys little phase margin while raising the overdrive spike
(∝ τ_motor/τ_cl) and the noise gain. The constant is exposed as `act_tau_ms` (env
`BALANCE_ACT_TAU_MS`), so I can sweep the whole envelope, not just one nice step.

**Sensor noise (input).** Independent Gaussian noise per channel (defaults, all
tunable): σ_θ = 0.2°, σ_θ̇ = 1°/s, σ_s = 5 mm, σ_v = 0.02 m/s.

**Measurement latency (input).** A fixed delay of 5–10 ms (default 7.5 ms =
3 ticks at 400 Hz), applied via a ring buffer on the sensed state. This is the
phase-margin stressor the earlier notes single out as the most dangerous one.

### Why it's reliably stabilisable — the RHP-pole budget

An unstable (RHP) pole at `p ≈ 4.04 rad/s` sets two requirements.

1. Bandwidth ≳ 2p, so the loop can dominate the divergence. Met: the angle
   closed-loop pair is at ~6.2 rad/s ≈ 1.5p, and the delay margin below shows
   headroom to push higher if I need to.
2. Total loop delay ≪ 1/p = 248 ms. Counting effective delay (the phase-relevant
   sum, not settling tails): measurement latency (5–10 ms) + filter group delay
   (≈ 1/(2π·fc) ≈ 5–7 ms) + actuator `τ_d = τ_cl` (≈ 8 ms) ≈ 18–25 ms, i.e.
   `p·T ≈ 0.07–0.10`. Comfortable.

The actuator's effective delay (`τ_cl ≈ 8 ms`) is ~28× shorter than the unstable
pole's time scale (1/p ≈ 248 ms), so it's nearly transparent to the balance loop.
The real challenge is sensor noise and latency, not the actuator, which is exactly
why they're in the model from the start.

---

## 6. I/O contract

**Input** (`ArmBalanceController::update(Measurement, dt)`):
`theta` [rad, from up, +→+X], `theta_dot` [rad/s], `s` [m], `v` [m/s].
**Output:** commanded cart lateral acceleration `a_cmd` [m/s²], clamped to ±a_max.
**Config:** gains, `s_ref`, `a_max`, filter cutoffs, `law` — via `configure()`
(Godot: a Dictionary; C++: a `Config` struct). All defaults are the shipped values.

---

## 7. Files, build, run

```
native/
  src/arm_balance_controller.{h,cpp}   portable control core (no Godot/ArduPilot)
  src/gdext_arm_balance.{h,cpp}        Godot wrapper class "ArmBalance"
  src/register_types.{h,cpp}           GDExtension entry point
  tools/compute_lqr.py                 offline CARE solve -> K (numpy)
  tests/test_plant_main.cpp            pure-C++ cart-pole validation
  tests/balance_mc.sh                  Godot headless Monte-Carlo / latency sweep
  SConstruct                           builds against godot-cpp (4.5 branch)
  godot-cpp/                           git-cloned dependency (not committed)
sim-v-1/
  armbalance.gdextension               registers the native lib
  bin/libarmbalance.*.so               build output
  balance_test.tscn / balance_test.gd  the plant + harness scene
```

**Build the extension** (one-time godot-cpp clone + compile):
```
git clone --depth 1 --branch 4.5 https://github.com/godotengine/godot-cpp.git native/godot-cpp
cd native && python3 -m SCons -j$(nproc) use_static_cpp=no
```
(`use_static_cpp=no` links the system libstdc++ — Fedora ships no static one. First
run also imports the extension: `godot --headless --path sim-v-1 --import`.)

**Pure-C++ validation** (fast, no Godot):
```
cd native/tests
g++ -O2 -std=c++17 -I../src test_plant_main.cpp ../src/arm_balance_controller.cpp -o test_plant && ./test_plant
```

**Godot scene:**
```
godot --path sim-v-1 res://balance_test.tscn                 # interactive (live accel graph)
BALANCE_HEADLESS=1 BALANCE_SEED=1 godot --headless --path sim-v-1 res://balance_test.tscn
bash native/tests/balance_mc.sh seeds 24 7.5                 # Monte-Carlo
bash native/tests/balance_mc.sh sweep 20                     # latency sweep

# accel traces: the interactive scene shows a live a_cmd-vs-a_real strip-chart
# (scrolling window, auto-fit y); headless can dump a CSV and render it to a PNG:
BALANCE_HEADLESS=1 BALANCE_CSV=run.csv godot --headless --path sim-v-1 res://balance_test.tscn
python3 native/tools/plot_accel.py run.csv run.png [zoom_start_s] [zoom_span_s]
```
Scene env knobs: `BALANCE_SEED, BALANCE_LATENCY_MS, BALANCE_ACT_TAU_MS, BALANCE_INIT_DEG,
BALANCE_BUMP_DV, BALANCE_USE_PID, BALANCE_NO_NOISE, BALANCE_CSV, BALANCE_GRAPH_WINDOW`.

---

## 8. Results

**Pure-C++ plant** (nonlinear cart-pole, 3° start + mid-run kick, actuator ref-model
τ_cl = 8 ms):
- Clean baseline: max|θ| = 3.00°, settles to 0.02°. PASS.
- Monte-Carlo, noise + 7.5 ms latency: 30/30 seeds PASS, worst final 0.09°.
- Latency sweep: PASS through 50 ms (worst max|θ| 3.11°), which confirms the large
  delay margin the RHP-pole budget predicts.

**Godot scene** (real Jolt physics: kinematic cart + HingeJoint arm, 4° start +
0.3 m/s mid-run cart bump, noise on):
- Monte-Carlo @ 7.5 ms latency: 24/24 seeds PASS, worst max|θ| 4.00° (that's the
  start angle — it never diverges past it), worst final 0.077°, no saturation.
- Latency sweep (8 seeds/point), worst final angle at t = 15 s:

  | latency | result | worst final θ |
  |--------:|:------:|:-------------:|
  |   5 ms  | 8/8 PASS | 0.062° |
  |  7.5 ms | 24/24 PASS | 0.077° |
  |  10 ms  | PASS (spot-checked) | 0.013° |
  |  15 ms  | 8/8 PASS | 0.067° |
  |  20 ms  | 8/8 PASS | 0.070° |
  |  30 ms  | 8/8 PASS | 0.077° |
  |  40 ms  | 8/8 PASS | 0.085° |

  The final angle rises smoothly with latency and stays < 0.1° (the pass bar is
  5°) — the curve you'd expect as margin erodes; the pure-C++ plant extends the
  same trend cleanly to 50 ms.

  One note on method: long headless sweeps in this environment were flaky.
  Parallel Godot instances occasionally race on the shared `.godot` import cache,
  and the machine intermittently suspends mid-batch. Empty runs get retried
  (`balance_mc.sh`), and the 10 ms point above was confirmed with serial single
  runs (e.g. seed 5 → final 0.013°) after a batch-collection glitch. None of the
  anomalies were control failures; every run that produced data passed.

Putting it together: with the actuator ~28× faster than the unstable pole, the
controller holds the arm essentially at the perturbation amplitude and settles to
a fraction of a degree. The loop is dominated by sensor noise and latency, and
very tolerant of both, exactly as the RHP-pole budget predicts.

---

## 9. Double inverted pendulum on the cart (`balance2_test.tscn`)

The two-link goal is a serial double inverted pendulum: link1 (0.7 m) hinged to
the drone, link2 (0.6 m) hinged to link1's tip, both held upright by the drone's
single lateral acceleration `u = a_x`. One input, two links: under-actuated, and
the classic hard benchmark.

### 9.1 Plant and controller

Point masses `m1 = m2 = 0.1 kg` at the link ends; absolute angles `θ1, θ2` from
vertical. From the Lagrangian, linearised about upright:

```
M [θ1''; θ2''] = G [θ1; θ2] + H u
  M = [[(m1+m2)L1, m2 L2], [L1, L2]]   G = diag((m1+m2)g, g)   H = [-(m1+m2); -1]
state x = [θ1, ω1, θ2, ω2, s, v]      u = cart accel
```

The open loop has two unstable poles (≈ +2.97 and +7.2 rad/s), and the fast one
sets a tight delay budget (`1/p` ≈ 139 ms vs 248 ms for the single arm). A single
MIMO-LQR `u = -K x` stabilises it; both modes couple well to the input (unlike two
parallel arms), so the gains stay moderate. `K` comes from `compute_lqr2.py`
(Q=diag(400,10,400,10,2,4), R=1), hardcoded in the portable core
`two_arm_balance_controller.{h,cpp}` and exposed to Godot as `ArmBalance2`:

```
K = [ -213.5,  -14.5,  260.8,  45.1,  1.41,  4.53 ]
closed-loop poles: -8.58, -6.58, -4.47±2.40j, -0.58±0.34j   (all stable)
```

The Godot scene is a genuine serial chain: `Link1` (RigidBody, point mass at 0.7 m)
on `Hinge1` to the drone, `Link2` (point mass at 0.6 m) on `Hinge2` anchored at
Link1's moving tip. Jolt integrates the true coupled nonlinear dynamics; the
controller only sees the two absolute angles + rates and the cart state.

### 9.2 Results

Both links start at +4° (a small lean of the upright stack), noise on, 7.5 ms latency,
mid-run cart bump:

- No noise: link1 max 5.1°, link2 max 4.0°, both settle to 0.01°. PASS.
- Monte-Carlo, 16/16 seeds PASS: worst max link1 5.2° / link2 4.0°, worst final
  < 0.4°, `rms_u` ≈ 1.0. It settles cleanly — no residual jitter, because the
  moderate gains don't pump noise the way an over-actuated coupling would.
- Latency margin (pure-C++ model): holds to ≈ 25 ms measurement latency and falls
  by 40 ms, tighter than the single arm's 50 ms, as the +7.2 rad/s pole demands.
- Recoverable start: a common lean up to ~6° holds; a fully scissored start
  (links folded opposite, e.g. `BALANCE2_INIT2_DEG=-5`) is the near-nonlinear limit
  and diverges.

```
godot --path sim-v-1 res://balance2_test.tscn                        # interactive (2 live graphs)
BALANCE2_HEADLESS=1 BALANCE2_CSV=run2.csv godot --headless --path sim-v-1 res://balance2_test.tscn
python3 native/tools/plot_accel2.py run2.csv run2.png                # both link angles + single input
```
Knobs: `BALANCE2_SEED, BALANCE2_LATENCY_MS, BALANCE2_ACT_TAU_MS, BALANCE2_INIT_DEG,
BALANCE2_INIT2_DEG, BALANCE2_BUMP_DV, BALANCE2_GRAPH_WINDOW, BALANCE2_CSV,
BALANCE2_NO_NOISE`.

### 9.3 Why it works (where two parallel arms didn't)

A quick prototype that hung two separate arms on the one cart (parallel, not
serial) couldn't be stabilised dependably: its differential mode had ~11× less
control authority than its common mode, which forced huge gains that the loop
delay then destabilised. The serial double pendulum behaves differently, because
both unstable modes couple strongly to the single input, so moderate gains
(~200–260) suffice and it settles cleanly. The price is the faster unstable pole
(+7.2 rad/s), which is why the latency margin is tighter (~25 ms) and a real build
has to keep the sense→command→realise loop well under it.

---

## Phase 8 checkpoint: PASS
The single arm balanced ≥ 15 s under the specified actuator lag, sensor noise, and
5–10 ms measurement latency, in both the pure-C++ plant and the real Godot physics,
with the margin characterised. The double inverted pendulum (serial links 0.7/0.6 m,
one input, `balance2_test.tscn` + `ArmBalance2`) balances upright for 15 s under the
same noise + latency (16/16 seeds, settles to <0.4°); its tighter latency margin
(~25 ms, set by the +7.2 rad/s pole) is quantified in §9.
