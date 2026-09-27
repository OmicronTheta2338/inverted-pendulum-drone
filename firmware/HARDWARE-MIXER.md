# Physical motor mapping

Body axes are forward/right/down. In the supplied top view, forward is image top,
right is image right, and up points towards the camera. Rotation is viewed from
each motor's shaft end towards its body. The lateral shaft ends are assumed to
face outwards as shown in the CAD. User confirmed left motors push the aircraft
left and right motors push it right; the air moves the opposite way.

| Output | Position | Role | Rotation | Force on aircraft |
|---|---|---|---|---|
| 1 | Front left outer | Lift | CCW | Up |
| 2 | Front left inner | Lateral | CCW | Left |
| 3 | Front right outer | Lift | CW | Up |
| 4 | Front right inner | Lateral | CW | Right |
| 5 | Rear right outer | Lift | CCW | Up |
| 6 | Rear right inner | Lateral | CCW | Right |
| 7 | Rear left outer | Lift | CW | Up |
| 8 | Rear left inner | Lateral | CW | Left |

The supplied Betaflight dump maps outputs 1..8 to PB0, PB1, PA0, PA1, PA2, PA3,
PD12, PD13. Those match the pinned ArduPilot StellarH7V2 hwdef output pins.
Betaflight `motor_output_reordering` is identity. Its board-alignment settings
are zero and the user confirmed the model's front matches the physical front.
These are firmware/configuration facts, not independent PCB identification.
The dump uses OCTOX8: useful for enabling eight individual test outputs, but
its coaxial lift allocation does not describe this airframe.

## Assumed geometry

Reported mass is 1.75 kg. Whether that includes the flight battery and pendulum
is unconfirmed. Mass is recorded, not inserted into a simulated hover constant.

The user measured an outer rectangle 740 mm fore/aft by 453 mm left/right,
and a lateral-motor rectangle 255 mm fore/aft, both centred on the geometric
centre. CoM is approximately 10 mm forward, towards the battery/FC. Lift hubs
are interpreted as 42 mm above CoM; lateral axes are approximately level with it.

| Group | Forward X relative to CoM | Right Y | Down Z |
|---|---|---|---|
| Front lift M1/M3 | +360 mm | -226.5/+226.5 mm | -42 mm |
| Rear lift M7/M5 | -380 mm | -226.5/+226.5 mm | -42 mm |
| Front lateral M2/M4 | +117.5 mm | Unmeasured; left/right | approximately 0 |
| Rear lateral M8/M6 | -137.5 mm | Unmeasured; left/right | approximately 0 |

Lateral Y is stored as null, not an invented width. It drops out of the moment
calculation for force purely along Y. A nonzero X/Z force component would require
that coordinate. Mass/configuration scope and vertical CoM estimates remain uncertain.
Reaction torque/thrust coefficient k is still assumed 0.01 m. Inertia is not used
for static allocation; it matters for dynamic response and gain tuning.

## Mixer factors

Regenerate the Lua and script-checksum parameters with:

```bash
python3 firmware/generate_hardware_mixer.py
python3 firmware/tests/check_hardware_mixer.py
```

Collective factors are 1 at the front and 360/380 at the rear. Thus the front
pair supplies 380/740 = 51.351% of total lift, cancelling pitch torque about the
offset CoM. Total lift per unit collective is slightly lower than the former
four-identical-factors convention. Hover throttle must still be established on hardware.

Roll and pitch use the lift rotors. This avoids using opposing lateral thrusters
to generate pitch. Yaw uses both lift reaction torque and lateral thrust; its
lift weight is k/0.1275. Equal front/rear lateral thrust is retained, so left/right
translation alone would generate yaw from the 10 mm offset. The right-demand
column adds a zero-net-lift diagonal differential of +/-0.5 to the lift rotors,
producing the cancelling reaction yaw torque. This correction is scaled with
collective by the upstream horizontal-demand path, as is lateral thrust.

The checks parse the emitted Lua and calculate physical forces/moments from
`r cross F - spin * k * shaft_outward`. They apply throttle-scaled horizontal
demand and final one-way thrust clipping at level, below the allocator's global
scaling thresholds. Both signs of pure roll/pitch/yaw/lateral requests have the
correct moment/force and zero unwanted cross-axis moments under these assumptions.
They test collective balance at several levels and combined yaw/lateral demands.

This is a fixed approximate allocation, not a general inverse or constrained
optimiser. Combined yaw/lateral commands give unequal front/rear one-way lateral
thrust and a prop-reaction pitch residual: at level below saturation its magnitude
is bounded by `2*k*min(abs(throttle*right), abs(yaw))` in the normalised thrust model.
Pitch feedback must reject it; it has not been tested in a closed hardware loop.
The assumed k also limits the accuracy of lateral-yaw compensation. Large tilt,
saturation, unequal thrust, propeller airflow and motor dynamics require further tests.
No simulated hover constant, mass or inertia was silently changed.

## Commission on hardware

1. Save the complete supplied Betaflight dump as a local recovery file, and save
   AM32 settings separately. Reflashing the FC does not intentionally rewrite ESC
   configuration. Check directions after flashing nonetheless.
2. Flash the rebuilt board image and load its matching parameters with props removed.
3. Confirm attitude follows the actual roll, pitch and yaw axes and the displayed
   front. Change FC orientation only after observing the physical discrepancy.
4. Test outputs 1..8 independently and compare every position and rotation to this table.
5. Verify positive torque commands and disturbance corrections by their differential
   outputs, using controlled bench tests. Roll-right torque needs more left lift;
   nose-up torque needs more front lift; clockwise yaw torque favours the CCW lift
   pair M1/M5 and the lateral pair M4/M8. Reverse demands reverse these preferences.
   Closed-loop outputs also include existing attitude error, filters and integrators;
   this is not a promise of fixed PWM readings from a stationary unarmed aircraft.
6. Verify propeller handedness/orientation produces the force directions in the table
   in an appropriate restrained loaded test. Props-off rotation cannot prove thrust.
7. Measure CoM and lever arms relative to it, validate sensor estimates, failsafes,
   battery sensing and initial gains before free flight. No flight-readiness claim is
   made by a successful build or a disarmed simulator boot.
