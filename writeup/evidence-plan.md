# Evidence and visual plan

The document does not need dozens of plots. It needs a small set that establishes the problem,
shows the decisions, and proves the integrated result. Priority 1 is the minimum simulation
edition; priority 2 adds depth; priority 3 belongs mainly to the hardware update.

## Figure register

| ID | Pri. | Question the reader has | Best visual | Source available now | Gap / capture needed |
|---|:---:|---|---|---|---|
| V00 | 1 | Does the complete idea work at all? | 15–20 s hero video plus a still with catch/hold callouts | `assets/drone-demo.mp4` and preview | Keep the current continuous run as technical evidence; capture a cleaner, closer view for the page hero if time permits. |
| V01 | 1 | Why not use a normal quadcopter? | Two-panel vector illustration: tilt-to-translate versus independent level translation in the pendulum plane | Geometry and controller concept | Draw as a clean SVG. Show force arrows, base attitude, and pivot motion; do not imply an independently actuated forward axis. |
| V02 | 1 | What is physically different about this drone? | Top/side motor-layout diagram with axes and upright/lateral motor groups | `numbers`, `main.tscn`, rough CAD | Reconcile current physical geometry first. Label this “design geometry,” not a manufactured assembly. |
| V03 | 1 | How did the architecture evolve? | Four-stage timeline: analytical budget → ArduPilot/SITL → measured bridge latency → in-process C++ core | Phase 0 and native-flightstack notes | Draw; include the 20–40 ms bridge observation and the reason the hardware ecosystem is still retained. |
| V04 | 1 | How does information flow? | Control/data-flow diagram distinguishing simulator truth, degraded sensing, portable controller, and plant | README diagram and current code | Redraw for publication; show the truth-only grader as a separate dashed path. |
| V05 | 1 | Is the base fast enough? | Lateral step plot: command, realized acceleration, 5% band, and 15 ms marker | Full-rate `[trace]` output exists | Run deterministic no-arm case, archive raw trace and metadata, generate vector plot. |
| V06 | 1 | What happens during the full maneuver? | Four aligned panels: link angles, cart position/velocity, lateral/vertical command, and controller mode; catch marker at handover | Demonstration and coarse logs exist | Add/export a full-state CSV from `main.gd`; current idealized CSVs do not contain all needed fields. |
| V07 | 1 | Is balance repeatable, not just a lucky run? | Link-angle time series for one representative run plus an inset/strip for all seeds | Summarized 8/8 result | Archive all seed summaries and one full trace; show start perturbation and any injected bump. |
| V08 | 1 | Where is the stability boundary? | Pass-rate and peak-angle versus shared arm-sensing latency, with nominal and failure regions shaded | Packet-D summary | Re-run serially around 40–60 ms injected latency and save failed runs as well as passes. |
| V09 | 1 | Does reaction feed-forward matter? | Synchronized side-by-side release video and a quantitative paired disturbance plot | `render_ff_comparison.sh`; documented 2.2× result | Export numeric FF-on/off traces using the same initial state and fixed measurement window. |
| V10 | 2 | How did the controller become more sophisticated? | Controller ladder: scalar energy pump fails → iLQR nominal path → TVLQR tracking → LQR catch | Trajectory CSV/NPZ and controller code | Capture a failed energy-pump trace and an optimized state-space trajectory. |
| V11 | 2 | Why is latency the binding constraint? | Dimensionless delay budget graphic comparing actuator/sensing delay with single, double, and optional triple unstable time scales | Analytical notes and measured break tests | Use consistent definitions of injected and total effective delay; do not mix time constant with settling time. |
| V12 | 2 | What makes the simulation more than an animation? | Layered plant illustration/table: rigid body, actuators/electrical, aero/friction, sensors, timing | `main.gd`, `simulation/`, model-fidelity notes | Visually distinguish implemented structure, identified coefficients, and deferred physics. |
| V13 | 2 | How was the LQR catch region selected? | Catch-basin heat map over link angle/rate or a 2-D slice with pass/fail points | Existing harness knobs | Requires a systematic basin sweep and machine-readable summary. |
| V14 | 2 | How much authority does the mixer have? | Force/torque allocation diagram or normalized motor-output bars for hover and +X command | Mixer smoke test and geometry | Export exact current allocation; avoid using incomplete Phase-1 factor table. |
| V15 | 2 | What does robustness mean here? | Sensitivity tornado/heat map for mass, motor lag, friction, noise, and latency | Model knobs exist | Run only after defining plausible ranges; current notes say a sensitivity sweep is still missing. |
| V16 | 3 | How did the physical design change? | CAD evolution montage and one annotated exploded view | Rough body and clamp STLs | Add dated screenshots/renders and photograph prototypes; explain each revision. |
| V17 | 3 | Do the assumed actuator parameters match hardware? | Thrust/current curve and step-response overlays, sim versus thrust stand | No bench data | Build thrust stand protocol; capture voltage, PWM/DShot, RPM, current, thrust, and temperature. |
| V18 | 3 | Is vibration within the sensing budget? | IMU acceleration PSD at motors off, hover-equivalent restrained throttle, and relevant filters | No bench data | Log raw IMU at full rate with RPM telemetry; avoid relying only on RMS. |
| V19 | 3 | Does simulation predict the physical vehicle? | Sim/bench/flight overlays with the same commanded input and confidence bands | No hardware data | Define identical test profiles and synchronize clocks before hardware bring-up. |
| V20 | 3 | What was actually built? | Clean build photography: overall, electronics stack, pivot/sensors, and safety rig | No build photographs yet | Photograph during assembly, not only after completion. Include scale and annotations. |

## The five plots to make first

1. **V05 — acceleration step response.** It supports the architectural pivot and the central
   timing argument.
2. **V06 — full swing-up state history.** It converts the video from spectacle into evidence.
3. **V07 — noisy double-balance trace plus seed summary.** It establishes repeatability.
4. **V08 — delay sweep through failure.** It shows engineering margin rather than a nominal pass.
5. **V09 — paired reaction-feed-forward comparison.** It is an intuitive ablation with a visible
   effect.

Together with V01–V04 illustrations, those are enough for Edition 1.

## Existing demonstration visual audit

The current MP4 is a good technical overview: one continuous 17.5-second, 2560×1440 run with a
visible release event and commanded-versus-realized acceleration. The green double link reads
clearly against the gray background, and the right-hand state panel explains the mode.

At portfolio thumbnail size, however, the airframe is small and the five left-hand telemetry
strips dominate the composition. Use the existing video in the evidence/results section. For the
opening hero, consider a second capture with:

- a closer camera and more frame area given to the drone and links;
- only controller mode, link angles, and catch/release event labels onscreen;
- no more than one small acceleration plot;
- the same fixed seed and continuous, uncut maneuver;
- a clear “simulation” label.

This is a presentation change, not a replacement experiment. The detailed dashboard remains more
useful when the reader reaches the quantitative results.

## Tables worth keeping

Only three tables deserve space in the main document:

1. **Dated configuration:** analytical baseline, validated simulation, physical design.
2. **Validation summary:** test, disturbance/model, number of runs, result, evidence link.
3. **Sim-to-real closure plan:** uncertain parameter, current assumption, measurement, acceptance
   rule.

The full sensor model, gains, BoM, and experiment manifest belong in appendices or linked files.

## Illustration language

- Use one consistent lateral axis and one angle convention throughout.
- Color the first and second links consistently across the video, diagrams, and plots.
- Reserve a third color for commands and a neutral dark line for realized/truth values.
- Draw simulator-only paths with dashed boundaries; draw flight-targeted code with solid ones.
- Put shaded regions behind plots for catch gates, 5% bands, nominal latency, and actuator rails.
- Caption every result with configuration, seed count, and evidence class.

## Material to collect during the build

Photograph or record these moments even if they never reach the final article:

- parts laid out before assembly, with measured masses;
- each CAD revision beside the problem it fixes;
- motor/prop thrust-stand setup and step test;
- center-of-mass suspension/balance check;
- electronics and wiring before covers make them inaccessible;
- pivot, arm IMUs, encoder/magnet alignment, and mechanical stops;
- raw vibration test setup;
- restrained first spin, first controlled lateral step, and failures;
- single-link bring-up before the double link;
- a fixed-camera final run plus synchronized onboard log.
