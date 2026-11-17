# Keeping the Base Level

## Designing a drone to swing up and balance a double inverted pendulum

*A simulation-first engineering case study in independent lateral thrust, latency-aware control, and
turning failed approaches into a testable architecture.*

> **Current status — simulation:** The complete double-link swing-up, catch, balance, and release
> sequence runs on a flight-targeted C++ controller against a Godot/Jolt rigid-body plant with
> modeled actuator dynamics, sensor degradation, and latency. The physical airframe has not yet
> been built.

<!-- HERO: V00, continuous run. Caption catch time, configured hold, and release. -->

A quadcopter balances itself by tilting. To move right, it leans right and redirects part of its
thrust sideways. That is a problem if its payload is an inverted pendulum: the maneuver that moves
the pivot also rotates the base the pendulum must balance against.

This project takes a different approach. Four rotors point upward to support the vehicle, while
four point horizontally to translate it along the pendulum plane. The base can therefore move beneath a falling pendulum
without deliberately pitching into the correction. On top is a serial double pendulum—two unstable,
coupled links, one lateral control input, and a faster mode that makes tens of milliseconds matter.

The result so far is a complete simulation maneuver. An optimized controller swings both links up
from hanging, catches them near vertical, and hands over to a balance controller. The same portable
C++ core commands a full rigid-body drone whose motors lag, saturate, unload in axial flow, and
share a sagging battery, while its measurements arrive noisy and late. When balance is disabled
and the links fall, a coupled reaction model predicts part of the load before the base has moved.

The interesting part is not that one carefully chosen run works. It is how the design changed as
the tests became less forgiving.

<!-- RESULT STRIP: cite C03–C07 only after raw evidence is archived. -->

## One constraint shaped the airframe

<!-- FIGURE V01: normal quad tilt-to-translate vs independent level translation in the pendulum plane. -->

Near upright, an inverted link's error grows on a time scale set approximately by
`sqrt(L/g)`. The serial double has two unstable modes; the faster one is more demanding than either
link considered alone. That turns a visual stunt into an engineering budget: the vehicle needs
enough lateral authority, but more importantly it needs to sense, decide, and realize a correction
before phase lag consumes the stabilizing margin.

The early analysis compared that demand with motor thrust and lag. It led to a deliberately unusual
layout: four independent lift motors and two opposing pairs of lateral motors. Because the lift
group has substantial thrust-to-weight margin in the current design, the lateral group can be
horizontal rather than sharing lift duty through a shallower cant angle.

<!-- FIGURE V02: dated current geometry, motor IDs, thrust axes, center line, link dimensions. -->
<!-- FIGURE V11 or small budget graphic: unstable time scales vs nominal loop delay. -->

Before publication, put three configurations side by side here:

| | Early analytical baseline | Validated simulation | Current physical design |
|---|---|---|---|
| Body mass | TODO | 1.0 kg, deliberately conservative | about 0.85 kg estimate; reweigh when built |
| Link lengths | 0.60 / 0.50 m | 0.70 / 0.60 m | TODO: freeze from CAD |
| Motor/prop | early thrust reference | modeled thrust curve | F60 Pro V 1950 KV / T5147-3 selected; bench test pending |
| Control rate | 400 Hz target | 400 Hz | 400 Hz target |

This table is important. Without it, design evolution looks like contradictory documentation.
With it, the reader can see which values were conservative simulation assumptions and which have
become physical commitments.

## The first architecture was correct in principle—and too slow in practice

The project initially chose ArduPilot rather than a clean-sheet flight stack. That decision still
makes sense for hardware: DShot output, IMU filtering, estimation, logging, failsafes, and a mature
bring-up environment are larger jobs than the pendulum controller itself. ArduPilot also has a
6-DOF motor path in which lateral force is an explicit command axis.

Godot was connected to ArduPilot SITL through the external-physics UDP interface. This made the
simulator exercise actual flight firmware, but the bridge introduced roughly 20–40 ms of dead time.
For ordinary position control that may be modest; beside the double pendulum's fast unstable mode,
it is a large part of the budget before sensor filtering or motor response is counted.

The response was not to discard the flight ecosystem. The latency-critical subset was separated
into a small in-process C++ core: acceleration mode, attitude control, the eight-rotor mixer,
reaction feed-forward, balance, and swing-up. Godot bindings adapt the simulator to that core, but
simulator code does not enter it. The same narrow interfaces are intended to support an H743 target
later.

The current allocation has five commanded axes—roll, pitch, yaw, lift, and lateral force in the
pendulum plane. Godot still integrates all six rigid-body degrees of freedom, but the forward-force
factor is zero. Calling the current vehicle a fully actuated six-axis platform would therefore
overstate what has been implemented.

<!-- FIGURE V03: architecture timeline with measured bridge latency as the pivot. -->
<!-- CALLOUT C08/C09: “portable and flight-targeted” is demonstrated; deployment is not. -->

This was the project's most consequential design change. It reduced the fast-loop timing cost and,
just as importantly, created a controller that could be tested without Godot at all.

## A simulator that can disagree with the controller

<!-- FIGURE V04: truth → sensor degradation → controller → actuators → plant, plus truth-only grader. -->

The simulation is split across a deliberate boundary. Godot/Jolt owns the world:
six-degree-of-freedom rigid-body motion, serial hinge reactions, distributed link mass, motor force and torque,
asymmetric motor response, propeller saturation and unloading, battery sag, drag, and joint losses.
A separate sensing layer delays and degrades IMU, estimated vehicle state, fused first-link state,
and encoder-derived second-link state.

The controller never receives pristine simulator state. It sees the modeled measurements that
hardware would have to supply. The grader does the opposite: it stays on simulator truth so a noisy
measurement cannot make a poor response look good.

That separation also makes the limits clear. The model contains the *structure* needed to represent
vibration, drag, battery and sensing effects, but many coefficients remain conservative estimates.
The 400 Hz simulator cannot establish the real aliasing behavior of higher-frequency motor
vibration. Opposing-prop wash is not a substitute for CFD or a bench measurement. The point is not
to call the simulation realistic in the abstract; it is to name what is represented and design the
tests that will identify what is not.

<!-- FIGURE V12: fidelity layers, with “implemented,” “identified,” and “deferred” columns. -->

## Four control layers solve four different problems

### 1. Make the base respond before the pendulum escapes

Motor thrust behaves roughly like a lag. The acceleration controller applies an inverse-model
feed-forward term so the commanded response follows a faster chosen reference, then uses modest
delayed feedback to trim model error. The goal is not an infinitely sharp step; it is a repeatable
response inside the outer loop's latency budget without amplifying noise or living on the motor
rail.

<!-- FIGURE V05: +6 m/s² command, realized truth, 5% band, 15 ms marker. -->

### 2. Predict the load from moving links

Each link pushes back through its hinge. For the serial double, the two reactions are coupled. A
model estimates the bob accelerations from measured link states and commands an opposing force.
An independent truth-based check found close agreement in simulation, and a paired release test
shows the practical effect.

Later ablation produced a more useful conclusion than “feed-forward is necessary.” With altitude
hold carrying link weight, the balance loop rejected much of the remaining lateral disturbance.
Reaction cancellation still improved acceleration tracking and release behavior, but it was not
the sole reason the pendulum stayed upright.

<!-- FIGURE V09: paired post-release traces and synchronized video. -->

### 3. Catch and balance both unstable modes

Near upright, a full-state MIMO LQR commands lateral acceleration from both absolute link angles
and rates plus base position and velocity. The serial chain is under-actuated—one base input controls
two links—but its two unstable modes remain controllable with moderate gains. A state gate only
hands control to this law once both links enter the measured catch region.

<!-- FIGURE V07: noisy full-plant balance trace, seed summary inset, bump marker. -->

### 4. Reach the catch region

The single-link controller could add or remove pendulum energy with a scalar energy-pumping law.
On the double, that strategy could not decide how energy should be distributed between coupled
modes. It flailed without reliably arranging both angles and rates for capture.

The replacement uses iLQR offline to find a bounded swing-up trajectory. A time-varying LQR tracks
the nominal path through the unstable approach, then the state gate transfers control to the
upright LQR. Open-loop replay is intentionally not enough: small errors grow as the links reach the
top, which is exactly where TVLQR earns its place.

<!-- FIGURE V10: failed energy pump, optimized state path, TVLQR corridor, catch handover. -->
<!-- FIGURE V06: full integrated maneuver with mode and event annotations. -->

## Failure boundaries are more informative than a perfect run

A nominal pass says the chosen configuration works. A delay sweep says how much room remains.
When angle and rate sensing are delayed together, the double eventually fails as rate feedback
loses the phase lead that stabilizes the fastest mode. The existing design notes place the full-loop
boundary near 70 ms of estimated total effective delay, compared with roughly 25–30 ms nominal.

That number should appear in the finished case study only after the sweep is rerun into this
workspace. Plot injected sensing delay directly, add the separately estimated actuator/filter/ZOH
contribution, and preserve representative failed traces. A shaded “nominal” point and a visible
transition into oscillation will communicate margin better than a sentence claiming robustness.

<!-- FIGURE V08: pass rate and peak angle vs shared angle/rate latency. -->

Other failures sharpened the design in different ways. A parallel-arm prototype had a weak
differential mode that demanded impractical feedback gains; correcting the topology to the intended
serial chain changed the controllability picture. A zero-acceleration vertical command allowed a
constant-velocity sink, forcing altitude hold to become an explicit loop. Each became a regression
test or an architectural boundary rather than an anecdote to hide.

## What is demonstrated now

<!-- Replace this table with links into the archived manifest before release. -->

| Test | Present result | Evidence class | Publication state |
|---|---|---|---|
| Portable mixer/controller smoke tests | Pass | code / idealized plant | Rebuild and archive |
| Acceleration response | about 15 ms to a 5% band for a moderate command | simulation | Re-run and archive full-rate trace |
| Noisy double balance | documented 8/8 passes over 15 s | simulation | Archive batch and full-state representative run |
| Double swing-up/catch/hold | documented 6/6; catch around 3.5 s | simulation | Archive batch and full-state representative run |
| Shared-delay margin | failure near 70 ms total effective delay | simulation | Re-run dense boundary sweep |
| Reaction feed-forward | documented roughly 2.2× disturbance reduction | simulation | Archive synchronized numeric ablation |

The video is the overview. The step response, state histories, seed distributions, and ablations
are the evidence.

## The next model is the physical vehicle

<!-- FIGURE V16/V20: CAD evolution and current assembly when available. -->

The airframe currently exists as rough CAD, evolving clamps, a component selection, and an
electrical layout. The next phase is not to tune the controller blindly on a finished drone. It is
to identify the parameters that dominate the simulation assumptions:

1. measure the finished mass, center of mass, motor positions, link properties, and pivot losses;
2. measure the actual motor/prop thrust curve, current, saturation, and step response across
   battery voltage;
3. measure vibration spectra and end-to-end sensor latency;
4. repeat the lateral response profile on a restrained bare airframe;
5. progress through fixed-base, single-link, and double-link tests.

<!-- FIGURE V17–V19: thrust curve, vibration PSD, and sim-to-real overlay as later-edition slots. -->

The same commands and metrics should be used on both sides of the simulator boundary. A good
sim-to-real result is not that every curve matches on the first attempt. It is that mismatches can
be traced to an explicit parameter, updated without changing the experiment, and tested again.

## What the design process changed

The project began as a mechanically unusual drone with a difficult controller. It became a study
in interfaces and timing. Independent lateral force made the mechanical problem possible. A
measured communication delay forced the controller into a portable core. Separating simulator
truth from controller sensing made results gradeable. Failed swing-up and feed-forward experiments
split coupled-looking problems into distinct layers.

The physical build will decide whether the assumed margins survive motors, vibration, joints, and
air. The simulation has already done its most useful job: it has turned those unknowns into a
short, measurable list.

<!-- END: repository, full demo, data/experiment manifest, and build-status links. -->
