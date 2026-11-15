# Claim register

This is the editorial source of truth for headline statements. “Available” means the repository
contains code or prose supporting a claim; “archived” means raw evidence and run metadata have
been copied into this workspace. The latter is the standard for a polished public result.

| ID | Candidate claim | Evidence available now | Status for Edition 1 | Required action or wording |
|---|---|---|---|---|
| C01 | The eight-rotor layout can command lateral translation while keeping the body approximately level. | Mixer smoke test; Godot plant and demonstration; motor geometry in `numbers`. | **Simulation supported; not archived** | Plot body pitch and lateral motion together during a step. Say “in simulation.” |
| C02 | The early 0.80 kg design had roughly 48.2 m/s² calculated/simulated peak lateral authority, about 10× its 5 m/s² requirement. | `Phase-1-Research.md`; historical actuator-only Godot step. The current 1.0 kg native plant is calibrated to a 38.0 m/s² rail. | **Historical analysis/simulation** | Do not attach 48.2 m/s² to the current plant. Re-run the current configuration; at the same 5 m/s² demand its nominal force margin is about 7.6×. Hardware authority still awaits the actual motor/prop thrust curve. |
| C03 | A moderate lateral command enters a 5% band in about 15 ms on the native simulation path. | `Native-Flightstack-Plan.md`; truth grader emits full-rate step traces. | **Simulation supported; raw data missing** | Archive a deterministic no-arm trace and define “enters the band” and command amplitude in the caption. |
| C04 | The double inverted pendulum balances for at least 15 s under the modeled noise and latency. | Current Godot implementation; documented 8/8 capstone results. | **Simulation supported; batch data missing** | Archive per-seed summaries and at least one full-state trace. Do not imply physical balancing. |
| C05 | The double swing-up catches at about 3.5 s and then holds upright. | Current demonstration; documented 6/6 runs; a current seed-1 run caught at 3.50 s and passed the current 8 s hold. The root README separately says more than 13 s. | **Simulation supported; partial reproduction** | Choose one release protocol, archive the batch and full state/control history, and report the duration actually tested. Use `SWINGUP_HOLD_TIME=13` if retaining the older >13 s claim. |
| C06 | The double balance loop has about 2.5× nominal timing margin and fails near 70 ms total effective delay. | Packet-D sweep summarized in `Native-Flightstack-Plan.md`. | **Simulation summary only** | Re-run a dense serial sweep around the boundary, archive all runs, and show injected versus estimated total delay separately. |
| C07 | Coupled reaction feed-forward reduces the release disturbance by about 2.2×. | Packet-C RMS results and deterministic comparison-video script. | **Simulation summary only** | Save synchronized FF-on/off numeric traces and define the RMS window and axes. The video is supporting, not quantitative, evidence. |
| C08 | The latency-critical controller is portable, dependency-light C++ separated from Godot. | `native/src/`; pure-C++ smoke/plant tests pass. | **Code-supported** | Show a boundary diagram and quote code size/interface rather than saying it is already deployed. |
| C09 | The same controller can run on the selected H743 flight controller. | H743 selected; C++ written with portability in mind. | **Not yet demonstrated** | Use “flight-targeted” or “intended to compile onto the H743” until cross-compiled and run on hardware. |
| C10 | The plant includes rigid-body coupling, motor lag, saturation, drag, battery sag, sensor noise, and per-channel latency. | Current Godot code and model-fidelity notes. | **Implementation supported** | A model block diagram/table can claim inclusion. Do not claim parameter accuracy before identification. |
| C11 | Nominal sensor assumptions are achievable with the proposed hardware. | `Sensor-Requirements-BoM.md` comparison against example parts. | **Analysis/provisional** | Phrase as requirements and selection rationale. Validate vibration, optical-flow velocity, and joint sensing on the rig. |
| C12 | The physical airframe is built and balances the pendulum. | None; rough CAD and selected components only. | **False at present** | State explicitly that it has not yet been built. Reserve bench/flight result sections for later editions. |
| C13 | The current physical design mass and geometry are locked. | `numbers` says 0.85 kg; main sim uses 1.0 kg; early notes and validated double use different arm lengths. | **Unresolved** | Publish a dated configuration table. Separate “current simulation,” “current CAD estimate,” and “early design” values. |
| C14 | iLQR/TVLQR was necessary for the double swing-up. | Failed scalar energy-pump attempt; optimized trajectory and validation ladder in `Swingup-Double-TrajOpt.md`. | **Design-process supported** | Show the failed energy-pump state/energy trace next to the optimized trajectory; avoid claiming mathematical impossibility beyond the tested controller. |
| C15 | Lateral reaction cancellation is required for balance stability. | Packet-D ablation found balance similar with it off once altitude hold carried weight. | **Not supported** | Say it improves acceleration/release disturbance performance; altitude hold, not lateral reaction cancellation, was stability-critical in that ablation. |
| C16 | The current vehicle is fully actuated in all six rigid-body axes. | Godot integrates 6-DOF motion and the mixer type has a 6-DOF heritage, but the current factor matrix has zero forward force and commands roll, pitch, yaw, lift, and one lateral axis. | **Not supported for current geometry** | Say “independent lateral thrust in the pendulum plane” or “five commanded axes.” Reserve “6-DOF” for plant dynamics or the mixer interface unless the geometry gains an independent forward-force axis. |

## Configuration discrepancies to resolve before publication

These are good design-history material, but poor hidden footnotes:

- Early Phase-1 work selected 0.60/0.50 m links; the validated serial double uses 0.70/0.60 m.
- The CAD estimate is about 0.85 kg; the main simulation deliberately uses a conservative 1.0 kg
  body. Pendulum masses must be listed separately.
- The historical 48.2 m/s² authority figure uses the 0.80 kg baseline; the current 1.0 kg native
  plant is calibrated to a 38.0 m/s² lateral rail.
- `numbers` cites a T5146-3 thrust point while the current BoM selects T5147-3 propellers.
- The current authored swing-up run grades an 8 s post-catch hold; the root README says more than
  13 s. Standardize and archive the release protocol before publication.
- The original architecture decision chose an ArduPilot/SITL path; the validated fast loop is the
  later in-process portable subset. ArduPilot still supplies the intended hardware ecosystem.
- The root README calls the vehicle fully actuated; the current mixer has no forward-force factor.
  Distinguish six-degree-of-freedom dynamics from the five-axis control allocation.
- `Project-Plan.md` checkboxes lag behind the actual simulation work and should not be used as a
  current status graphic.

The clean fix is one dated configuration table in the case study, with separate columns for
“analytical baseline,” “validated simulation,” and “physical design.”
