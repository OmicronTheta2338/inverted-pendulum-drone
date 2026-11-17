# Document plan

## Positioning

This should read as a rigorous engineering case study with the pace of a good project feature.
The result is unusual enough to attract attention; the design decisions and evidence should do
the convincing.

**Primary audience:** controls, robotics, embedded, and simulation engineers; technically
curious hiring managers; serious maker-project readers.

**Target:** 2,500–4,000 words for the main story, 10–14 main figures, a short technical appendix,
and one embedded 15–25 second demonstration. A web page is the natural primary format; it can be
exported to a landscape-friendly PDF later.

**One-sentence claim:**

> A simulation-validated, flight-targeted control stack uses an eight-rotor base with independent
> lateral thrust in the pendulum plane
> to swing a serial double pendulum upright, catch it, and balance it while the drone remains
> approximately level under modeled actuator dynamics, sensor noise, and latency.

Do not shorten this to “a drone that balances a double pendulum” until the hardware does it.

## The story in nine beats

### 0. Hero: show the whole maneuver first

Lead with the existing 17.5-second continuous simulation video or a cleaner recut: hanging links, swing-up,
catch, hold, then release. Pair it with three measured callouts: catch time, hold duration, and
nominal effective delay. State “simulation” in the deck, not in a footnote.

The opening text in [`draft/case-study.md`](draft/case-study.md) is enough to begin laying this
out before every plot exists.

### 1. The constraint that shapes everything

Explain why a normal quadcopter is the wrong base. It translates by tilting, which moves the
pendulum pivot and rotates the frame the pendulum is balanced against. The project instead
separates upright lift from lateral force: four upright motors support weight and four horizontal
motors translate the base along the pendulum's control plane.

Use a clean side-by-side illustration—ordinary tilt-to-move versus level translation—not a page
of mixer mathematics.

### 2. Turn instability into a design budget

Introduce only the physics needed to establish urgency:

- the single-link divergence scale, approximately `sqrt(L/g)`;
- the faster unstable mode of the serial double;
- the resulting acceleration, bandwidth, and end-to-end delay budgets;
- the motor/geometry authority estimate.

Then show how those numbers drove link length, motor direction, loop rate, and the decision to
keep the fast control path in-process. Use current validated geometry consistently; preserve
earlier values only in an explicit design-evolution callout.

### 3. Architecture decisions—and the pivot that mattered

Tell this as a decision chain:

1. Reuse ArduPilot's 6-DOF mixer interface and hardware services rather than rebuild a flight
   ecosystem. The current physical factor matrix populates five command axes; forward force is zero.
2. Connect ArduPilot SITL to Godot through its external-physics interface.
3. Measure roughly 20–40 ms of bridge dead time.
4. Port the latency-critical acceleration, attitude, mixer, and pendulum layers into a small,
   dependency-free C++ core that runs in-process in simulation and is intended for the H743.

This is the strongest “design process” section because a measured constraint changed the
architecture. Be precise: the current portable core is flight-targeted, not yet flight-proven,
and the complete ArduPilot integration remains future work.

### 4. Build a simulator that can disagree with the controller

Describe the boundary between the portable controller and the Godot/Jolt plant. The plant owns
rigid-body dynamics, motors, drag, joints, battery sag, sensor degradation, and timing. The
controller consumes only delayed/noisy measurements and commands thrust.

The important point is test independence: metrics are graded on simulator truth while the
controller runs on degraded sensing. A compact “truth → sensor model → controller → actuator →
plant” illustration makes this legible.

Also explain limits. Coefficients are estimates until bench measurement; prop-wash CFD and
high-rate vibration aliasing are not validated merely because structural hooks exist in the sim.

### 5. Build the controller in layers

Give each layer a job and one visual:

- **Acceleration mode:** invert the motor lag to create a faster reference response, then add a
  small delayed-feedback trim.
- **Reaction feed-forward:** estimate coupled hinge reaction and oppose it before it becomes an
  acceleration error.
- **Balance:** full-state MIMO LQR treats the level drone as a moving cart.
- **Swing-up:** iLQR generates a feasible double-link trajectory, TVLQR tracks it, and a state
  gate hands over to the upright LQR.

Keep derivations in an appendix. In the main text, show the signal flow and the plots that reveal
why each layer exists.

### 6. Show the failed paths

This is where the case study becomes a design story instead of a software demo. Use three short
callouts:

- Two nominally separate/parallel pendulums created a weak differential mode; correcting the
  model to the intended serial chain changed controllability and delay requirements.
- Scalar energy pumping could not control how energy split between the two coupled modes;
  trajectory optimization replaced it.
- Turning all reaction feed-forward off exposed a vertical-control bug: zero vertical
  acceleration is not altitude hold. Separating altitude hold from lateral reaction cancellation
  clarified which feed-forward contribution was stability-critical.

Include evidence or a diagram for each, but avoid presenting debugging accidents as polished
foresight. The lesson is the valuable part.

### 7. Results as a validation ladder

Order results from simplest to most integrated:

1. mixer and portable C++ smoke tests;
2. bare-base acceleration response;
3. idealized single and double balance;
4. full rigid-body double balance under modeled sensing;
5. delay-to-failure sweep;
6. swing-up, catch, and hold across seeds;
7. reaction-feed-forward release comparison.

Do not use a wall of PASS tables. One summary table plus four strong plots is enough; link the
full run manifest and appendix for the rest.

### 8. From simulation to hardware

Show current CAD and component layout, then make the next experiments concrete: thrust stand,
motor step response, vibration spectrum, sensor latency, restrained bare-airframe response,
single link, then double link. A sim-to-real closure table should pair every provisional model
parameter with its future measurement.

This section allows the first edition to end satisfyingly before the vehicle exists: the output
of simulation is not merely a video, but a prioritized hardware test plan.

### 9. Close on what was learned

The conclusion should be about design method:

- latency, not nominal force, was the binding resource;
- a clean interface allowed idealized control design to survive a more realistic plant;
- failures separated concerns that initially looked coupled;
- simulation earns trust by making assumptions and failure boundaries visible.

End with the next physical milestone and a link to the repository/demo.

## Main text versus appendix

Keep these in the main text:

- system concept and motor layout;
- the three major architecture changes;
- controller layer diagram;
- four or five representative results;
- current limitations and physical-validation plan.

Move these to an appendix or expandable notes:

- full state-space matrices and LQR gains;
- iLQR cost and convergence details;
- coupled reaction-force derivation;
- complete sensor parameter table;
- Monte Carlo seed table and exact commands;
- mixer coefficients, electrical architecture, and full BoM.

## Source map

| Case-study section | Primary repository source | Use it for |
|---|---|---|
| Constraint and architecture | `Phase-0-Research.md`, `Phase-1-Research.md` | Original decision criteria, unstable time scale, geometry and authority calculations |
| Acceleration and integration pivot | `Native-Flightstack-Plan.md` | Bridge latency, inverse motor model, sensing budget, feed-forward, capstone results |
| Balance | `Balance-Controller-Design.md` | Cart abstraction, LQR law, poles, idealized validation, double-chain controllability |
| Double swing-up | `Swingup-Double-TrajOpt.md` | Failed energy-pump context, iLQR/TVLQR design, handover gate, validation ladder |
| Model scope and limitations | `Model-Fidelity-Notes.md`, `Sensor-Requirements-BoM.md` | Implemented imperfections, deferred effects, hardware sensor requirements |
| Physical design and build | `Hardware-BoM.md`, `numbers`, `CAD/` | Selected parts, current mass/inertia/geometry, wiring, CAD evolution |
| Current integrated behavior | `sim-v-1/main.gd`, `sim-v-1/simulation/`, `native/src/` | Canonical current implementation when older prose disagrees |

Treat the final row as authoritative for what the software does today. Treat the other rows as
the design record; date any earlier configuration shown from them.

## Publication stages

### Edition 1 — simulation design case study

Ready when priority-1 simulation plots have archived raw data, the system illustrations are
drawn, and every headline number has a claim-register entry. The ending is “ready for model
identification and hardware bring-up.”

### Edition 2 — built airframe and bench characterization

Add detailed CAD/build photography, measured thrust curve and motor lag, mass/inertia, vibration
PSD, sensor latency, and a parameter-identification comparison against simulation.

### Edition 3 — flight validation

Add restrained and free-flight response, single-link balance, double-link balance/swing-up if
achieved, failure statistics, and sim-to-real overlays. Revise the title/deck only when the
headline claim has flight evidence.

## Editorial rules

- Put the result before the derivation.
- Every plot answers one question and has one sentence stating the answer.
- Use present tense for implemented behavior, past tense for design decisions, and future tense
  for hardware validation.
- Keep analysis, simulation, bench, and flight numbers visually distinct.
- Explain why a metric matters before giving its value.
- Preserve negative results that caused a design change.
- Avoid “realistic simulation” as a blanket phrase; name the modeled effects and the missing ones.
