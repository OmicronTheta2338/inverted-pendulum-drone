# Inverted Double-Pendulum Drone — Project Plan

A segmented build plan for a octacopter (4 upright motors + 4 canted lateral motors) carrying a double inverted pendulum on its central beam axis. Sequenced to keep failures isolated, avoid rework, and stay on the fastest safe path to a finished build.

**Guiding principles:** every phase ends in a checkpoint with explicit pass criteria; nothing physical is committed until its design is validated in simulation; the same flight code runs in sim and on hardware to avoid divergence.

[ ] Phase 1 - Initial layout & analytical performance budget
[ ] Phase 2 — Rough parametric CAD skeleton
[ ] Phase 3 — Drone physics in simulation (no pendulum)
[ ] Phase 4 — Sensor noise model
[ ] Phase 5 — Drone flying in simulation
[ ] Phase 6 — Order parts
[ ] Phase 7 — Single pendulum in simulation (physics first)
[ ] Phase 8 — Single pendulum controller
[ ] Phase 9 — Two pendulum arms in simulation
[ ] Phase 10 — Full-detail CAD
[ ] Phase 11 — PLA prototype build & basic control test
[ ] Phase 12 — CF / machined / TPU build & validation
[ ] Phase 13 — Single arm on hardware
[ ] Phase 14 — Two arms on hardware


---

## Phase 0 — Flight controller architecture decision

**Goal:** Settle, before any other work, whether the flight stack is custom-from-scratch or an extension of existing firmware (ArduPilot). This single decision shapes the entire downstream plan.

**Considerations:**
- The mixer matrix for your geometry is easy either way (a day or two of linear algebra). The genuinely hard, time-consuming parts are ESC/DShot output timing, IMU filtering (lowpass + dynamic/RPM notch), attitude/state estimation, failsafes, and tuning infrastructure. From scratch you must build all of these to flight quality; on ArduPilot you inherit them.
- Your defining requirement — **translate laterally without tilting** — breaks the standard multicopter control assumption (stock controllers steer by tilting the thrust vector). This is the *fully-actuated / omnidirectional* problem. It is not solved by a custom mixer alone; the controller above the mixer must command lateral force directly.
- ArduPilot already supports this class: there is an upstreamed 6DOF omnicopter control path, and people are actively running canted-motor frames via `add_motor_raw`. You'd be on a less-documented path, not a new one.
- ArduPilot has SITL (software-in-the-loop) that can connect to an external physics simulator — meaning your real flight firmware can run against your Godot physics. This removes the risk of sim-tested logic diverging from flight logic, directly serving the future-proofing priority.
- **Lua scripting is NOT suitable** for the inner control loop — it runs too slowly. Anything time-critical must be compiled C++ in the firmware. Lua is only for high-level glue.

**De-risking step (do this first, ~1 day):** Read ArduPilot's 6DOF/omni frame code and the canted-octocopter Discourse threads. Confirm your specific six-motor layout (2 upright, 4 canted) maps cleanly onto the existing 6DOF control path. This is the cheapest possible way to validate the biggest fork in the plan.

**Checkpoint:** A written decision with justification, and a confirmed understanding of how lateral force is represented in the chosen stack.

---

## Phase 1 — Initial layout & analytical performance budget

**Goal:** Produce a spreadsheet-level model of the whole airframe: component masses, positions, centre of mass, moments of inertia, motor thrust curves, cant angles, and an **analytical estimate of peak lateral acceleration / deceleration**.

**Considerations:**
- Lateral acceleration is set primarily by cant angle, motor placement radius, and motor thrust — all decided *here*, not later in simulation. Don't defer this; going into Godot with an unjustified geometry risks discovering fundamental problems mid-sim.
- Model the cant-angle tradeoff explicitly: vertical thrust loss scales as cos(cant) (≈30% loss at 45°, ≈13% at 30°), traded against lateral authority and any passive roll stability.
- Account for the front/rear motors carrying a disproportionate vertical share during hard lateral maneuvers, when the canted motors are vectoring sideways.
- Compute the pendulum's divergence timescale (~1/√(g/L) per arm) now — it sets the lateral response speed the drone must achieve, which feeds every later performance target.

**Checkpoint:** A geometry and component selection you can defend numerically, including a peak-lateral-accel figure with margin above what the pendulum demands.

---

## Phase 2 — Rough parametric CAD skeleton

**Goal:** A rough parametric CAD model confirming the geometry is physically realisable.

**Considerations:**
- Catch physical impossibilities early: motor mount clearances, prop overlap/wash interactions, central tube diameter vs pendulum pivot bearing, wiring routes.
- Keep it parametric so the full-detail CAD later (Phase 11) builds on it rather than restarting.
- Central tube: larger diameter gives bending stiffness scaling ~r³, but weigh against mass and pivot-bearing constraints.

**Checkpoint:** No geometric showstoppers; key dimensions locked enough to proceed in sim.

---

## Phase 3 — Drone physics in simulation (no pendulum)

**Goal:** Build the drone in Godot with custom physics — mass, CoM, inertia tensor, per-motor thrust, reaction torque, canted thrust directions — and connect the chosen flight stack (ideally ArduPilot SITL against the Godot physics).

**Considerations:**
- Validate the physics against analytical results before trusting it: hover thrust should match weight; a commanded differential should produce the lateral force your Phase 1 model predicted.
- If using ArduPilot SITL, get the sim↔firmware bridge working here — this is the foundation everything else runs on.
- Model reaction torque of the props correctly; with mixed orientations the yaw coupling is non-trivial.

**Checkpoint:** Drone holds a stable hover in sim under the real flight code.

---

## Phase 4 — Sensor noise model (timeboxed, ~2–3 days)

**Goal:** Add a representative randomised sensor-error model and use it to answer one question: **is IMU-only state estimation accurate enough to drop the Pi5/AprilTag system entirely** (a significant weight saving, increasing lateral accel)?

**Considerations:**
- Keep it deliberately simple — this is a robustness stress-test, not an attempt to reproduce a specific IMU.
  - **Gyro:** Gaussian white noise (σ ≈ 0.05 °/s) + slow bias random walk (~0.5 °/s/√hr) → drives slow attitude drift.
  - **Accelerometer:** Gaussian noise (σ ≈ 5 mg) + a vibration term (band-limited noise, ~50–100 mg peak at motor frequencies).
  - **AprilTag:** position noise σ ≈ 5–10 mm at operating distance, ~30 Hz, with occasional missed detections.
- Test **two** failure modes separately: (a) slow drift over 30–60 s with IMU only, and (b) vibration-induced corruption of the pendulum state estimate even without drift.
- **Avoid the rabbit hole:** do not model RPM-correlated harmonic vibration structure. Broadband noise at realistic RMS amplitude is enough.

**Checkpoint:** A clear yes/no/"depends on vibration" answer on whether the AprilTag system is needed. This decision feeds back into the mass budget.

---

## Phase 5 — Drone flying in simulation

**Goal:** Full lateral-translation flight in sim under the flight controller, hitting the lateral accel/decel targets from Phase 1.

**Considerations:**
- This validates the fully-actuated lateral-force control independently of any pendulum — preserve this separation deliberately.
- Tune attitude-hold tightly here; the pendulum controller later will assume the base stays level.
- Confirm lateral force is genuinely decoupled from attitude (no parasitic tilt when translating).

**Checkpoint:** Drone translates laterally on command while holding level attitude, within target response time.

---

## Phase 6 — Order parts (PARALLEL with Phases 7–10)

**Goal:** Place orders once geometry and component selection are locked, so delivery lead times (often 3–6 weeks, esp. overseas motors/ESCs) sit *off* the critical path.

**Considerations:**
- Order generous spares — e.g. ~10 motors if you need 6, covering both experimental needs and breakages.
- Order extra props and a couple of spare ESCs for the same reason.
- This phase runs concurrently with the pendulum simulation work; you've already locked the design by Phase 5.

**Checkpoint:** All parts on order with tracked lead times.

---

## Phase 7 — Single pendulum in simulation (physics first)

**Goal:** Add one pendulum arm to the sim and **validate its physics before any control work**.

**Considerations:**
- Check the linearised natural frequency against √(g/L) for your arm length. If it doesn't match, the model is wrong — fix it now, before tuning a controller against a broken plant.
- Model the pivot (bearing friction, encoder for angle sensing) realistically enough to matter.

**Checkpoint:** Pendulum dynamics match analytical predictions.

---

## Phase 8 — Single pendulum controller

**Goal:** C++ pendulum control software (compiled into the flight stack, on the H743 in the real build) that balances one arm, interfacing with the flight controller.

**Considerations:**
- Architect the pendulum controller as a **separable outer layer**: it reads arm angle and outputs a desired lateral force/position setpoint; the fully-actuated controller below executes it. This separation is what makes the build incremental.
- Sensing: arm angle encoders wire to the H743 (the Pi is AprilTag-only).
- Stretch goal: swing-up control to kick the arm from hanging to upright.

**Checkpoint:** Drone balances a single inverted arm robustly in sim, including under the sensor noise model.

---

## Phase 9 — Two pendulum arms in simulation

**Goal:** Add the second arm; validate the coupled double-pendulum physics, then extend control.

**Considerations:**
- Re-validate physics first (as in Phase 7) — the double pendulum has a faster higher mode; confirm it against analysis.
- The double inverted pendulum is genuinely harder (chaotic, faster divergence). Expect significant controller iteration.
- Reuse the separable-layer architecture from Phase 8.

**Checkpoint:** Drone balances both arms in sim under realistic sensor noise.

---

## Phase 10 — Full-detail CAD

**Goal:** Complete, manufacturable CAD of the drone frame, building on the Phase 2 skeleton.

**Considerations:**
- Incorporate every component now finalised (including whether the AprilTag system survived Phase 4).
- Design for the eventual CF/machined build even if the first physical build is PLA — keep mounting interfaces consistent so the PLA→CF transition isn't a redesign.

**Checkpoint:** CAD ready to print, with a clear path to the CF version using the same interfaces.

---

## Phase 11 — PLA prototype build & basic control test

**Goal:** 3D-print the frame in PLA, assemble, and validate basic drone control (no pendulum yet) on real hardware.

**Considerations — define pass criteria *before* building:**
- **Attitude stability:** e.g. ±2° steady-state, ±5° during disturbance.
- **Lateral step response:** back-calculated from motor thrust × cant to a value with margin over the pendulum's demand (from Phase 1/7).
- **Hover time:** enough for meaningful test runs (3–4 min), as a battery sizing check.
- These need not be perfect — refine against what the sim showed the pendulum controller actually demands — but writing them down prevents both over-testing PLA and prematurely jumping to CF.

**Checkpoint:** PLA drone meets the written criteria for bare-drone flight.

---

## Phase 12 — CF / machined / TPU build & validation

**Goal:** Rebuild with carbon fibre plates, machined attachments where possible, and TPU vibration dampeners; re-validate drone control.

**Considerations:**
- The stiffer, lower-vibration build should *improve* the IMU situation — re-check the Phase 4 vibration conclusion on real hardware.
- Validate against the same Phase 11 criteria; differences from PLA isolate material/stiffness effects.

**Checkpoint:** Production-build drone matches or exceeds PLA control performance.

---

## Phase 13 — Single arm on hardware

**Goal:** Mount one pendulum arm; deploy the Phase 8 controller; test balancing on real hardware.

**Considerations:**
- The sim↔firmware bridge pays off here: the deployed code is the code you validated in sim.
- Expect real-world surprises (encoder noise, bearing friction, prop wash on the arm) that the sim under-modelled — this is why the single-arm step exists before two arms.

**Checkpoint:** Real drone balances one arm.

---

## Phase 14 — Two arms on hardware

**Goal:** Mount the second arm; deploy the Phase 9 two-arm controller; final validation.

**Considerations:**
- Only attempt after single-arm hardware is solid — this is the highest-risk, highest-divergence step, so it must be maximally isolated.
- Keep the swing-up (if implemented) as a separate test from balancing.

**Checkpoint:** Real drone balances the full double inverted pendulum. **Project complete.**

---

## Critical-path notes

- The **Phase 0 decision** gates everything. Spend the day de-risking it before anything else.
- **Parts ordering (Phase 6)** runs in parallel — never let lead times block progress.
- The **separable controller architecture** (pendulum outer layer / fully-actuated inner layer) is what makes every increment testable on its own. Protect it.
- **Same code in sim and hardware** (via SITL) is the single biggest future-proofing lever; it's a strong argument for the ArduPilot path over from-scratch.
