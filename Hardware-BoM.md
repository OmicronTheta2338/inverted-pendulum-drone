# Hardware BoM and Purchase Record

This is the hand-off record for the physical build. It separates parts already owned or agreed for
purchase from decisions that still need closing. Delivery status was not tracked in the discussion,
so **purchase agreed** means the exact part and quantity were settled, not necessarily received.

**Last updated:** 2026-08-07  
**Immediate goal:** get the bare eight-motor vehicle flying reliably; pendulum and navigation sensors
come later.

## Owned or purchase agreed

| Subsystem | Qty | Exact selection | Status / notes |
|---|---:|---|---|
| Flight controller | 1 | StingBee Stellar H7 V2 (`STELLARH7V2`, STM32H743) | Already owned. Betaflight identifies it over USB as `StingBee Stellar H7 V2`. Intended flight firmware is ArduPilot. |
| Motors | 8 | T-Motor F60 Pro V, 1950 KV | Selection settled. |
| Propellers | 8 fitted, plus owned spares | T-Motor F5146 tri-blade | User confirmed ownership. Match CCW props to M1/M2/M5/M6 and CW props to M3/M4/M7/M8, viewed from each outward shaft end. Lift props blow down; lateral props blow inward toward the frame centre. |
| Battery | 1 | Tattu R-Line Version 6.0, 1480 mAh, 6S, 160C, XT60 | Purchase agreed. The Unmanned Tech listing photograph shows 160C; confirm against the pack label when it arrives. Standard 6S operation. |
| Battery | 1 | GNB 1300 mAh, 6S, 160C LiHV | Purchase agreed. Initially charge only to **4.20 V/cell (25.2 V pack)**, not the LiHV maximum, so commissioning stays at the standard 6S voltage point. |
| ESC | 2 | Holybro Tekko32 F4 Metal 4-in-1 65A, AM32, SKU 31097 | Purchase agreed. Two boards provide exactly eight channels; no third shelf spare for now. Each board is 4–6S, 65 A/channel continuous and 75 A burst, has no BEC, and includes a 1000 µF 35 V capacitor. |
| Total-current sensor | 1 | Matek HCS-150A Hall-effect current sensor | Purchase agreed. Install once, in the battery feed before it splits to the two ESCs. Nominal range is 0–150 A; see the measurement-range caveat below. |
| Laptop telemetry link | 1 | Seeed Studio XIAO ESP32C6, product 113991254 / Pi Hut SKU 105921 | Purchase agreed. Flash with generic ESP32-C6 DroneBridge firmware and use its onboard antenna as a direct short-range Wi-Fi/MAVLink bridge. Buy the board with loose headers and solder lightweight wires directly to its pads. |
| Workshop battery charger | 1 | SkyRC B6 AC Neo (`SK-100200`) | Purchase agreed. Single-channel AC/DC balance charger; supports 1–6S LiPo and LiHV, storage charging, 60 W from its internal mains supply and 200 W from a separate DC supply. Use the supplied/appropriate UK mains lead. |
| Battery charging containment | 1 | LiPo safety bag (exact model not recorded) | Already purchased. Charge on a non-combustible surface under continuous supervision; a fabric bag reduces fire spread but is not complete fire containment. |
| Central structural tube | 1 | Roll-wrapped carbon-fibre tube, 25 mm OD / 22 mm ID / 1.5 mm wall | Selection settled for both the printed prototype and final build. Cut length remains a CAD output; buy an oversize stock length and trim only after the bearing and plate interfaces are frozen. |

## Locked electrical architecture

```text
6S battery
    │
    ├── total battery-voltage tap to FC VBAT
    │
    └── one HCS-150A total-current measurement
             │
             └── insulated star/Y split
                    ├── Tekko32 4-in-1 A ── four motors
                    └── Tekko32 4-in-1 B ── four motors
```

- Use a short **10 AWG silicone-wire main feed** around the battery/sensor/split and **12 AWG
  silicone-wire branches** from the split to each ESC. Holybro explicitly specifies at least 12 AWG
  at each Tekko32 battery input. The dimensionally correct rough CAD is sufficient for ordering stock;
  keep the final cut lengths short.
- The gauge changes follow the electrical load. The 10 AWG section carries the combined aircraft
  current before the split and must tolerate the planned 120--140 A short-duration ceiling. Each 12 AWG
  branch carries only one ESC group's share, and 12 AWG is the ESC manufacturer's minimum. The 22 AWG
  FC battery feed carries only the FC and peripheral load, not motor current. The 26--28 AWG conductors
  carry logic, telemetry and sensor currents; making them thicker would add bulk without useful margin.
- Retain the batteries' stock XT60 interface and use a genuine AMASS airframe-side XT60. This avoids
  reterminating the battery leads or adding an adapter. Its temperature must be checked during the
  progressive loaded tests because the possible brief manoeuvre current exceeds its continuous rating.
- Use a lightweight insulated soldered star/Y harness rather than buying a generic PDB. The exact
  junction construction and wire lengths are assembly details still to be drawn after the component
  locations are known.
- Divide the motors across the ESC boards so each board carries two upright and two lateral motors.
  This balances the normal thermal load instead of putting all high-demand motors on one board.
- Fit **one supplied 1000 µF, 35 V electrolytic capacitor to each ESC**, directly across its battery
  pads with the leads as short as practicable and polarity checked. The selected ESC also has substantial
  onboard capacitance. No extra capacitors need to be purchased at this stage.
- Both ESCs have **no BEC output**, so there are no ESC 5 V rails to parallel. The FC's onboard BECs
  power the FC and low-voltage peripherals.
- For battery monitoring, feed raw battery voltage to the FC's `VBAT` input so its built-in divider
  measures pack voltage. Feed the HCS-150A's 0–3.3 V current output to the FC `CURR` ADC, power the
  sensor from regulated 5 V, and connect signal ground to FC ground. Do not also join the two Tekko
  analog current-sense outputs into that ADC.
- Do not assume the Holybro eight-pin harness matches the FC connector pin-for-pin. Repin or solder
  individual signal/ground connections from the documented pinouts during assembly.

## Provisional component placement

- Do not use the flight controller as a counterweight for the battery. The two flight packs weigh
  approximately 198--212 g, whereas the FC is approximately 9 g. Put the **battery mass centre on the
  vehicle X/Y centreline** and make its cradle adjustable by at least 10 mm for final CoM trimming.
- Put the **FC on the same X/Y centreline**, on a separate soft-mounted level above or below the battery
  rather than deliberately putting the two on opposite sides. Keep the IMU close to the intended body
  CoM and preserve access to USB-C and the microSD card.
- Put the XT60 entry, HCS-150A and positive/negative star split close together beside the battery. The
  positive 10 AWG main conductor passes through the Hall-sensor aperture before the split; the sensor is
  not inserted electrically in series with the power cable.
- Put one Tekko32 on each physical half, each close to its four motors and in direct airflow. Each board
  still serves two upright and two lateral motors. This keeps all 24 motor phase conductors local and
  leaves only one ESC power branch and its low-voltage control bundle crossing the centre tube.
- Put the XIAO beside the FC but near a non-carbon exterior edge, with its onboard antenna facing away
  from the battery/ESC power wiring and not covered by the eventual carbon plates.

## Provisional interconnect and wire schedule

These are routed-length design targets, not protocol hard limits. A dimensionally correct rough CAD model
is sufficient for ordering cable stock; printable/manufacturable details are not required. When assembling,
measure the supported path between termination points, allow the real bend radius, then add roughly
15--20 mm total service/termination allowance. Do not add coiled high-current slack.

| From | To | Conductors / interface | Gauge | Preferred routed length | Routing notes |
|---|---|---|---:|---:|---|
| Battery | Airframe | Stock battery leads and XT60 | Stock | Stock | Do not modify the pack leads. Provide strain relief on the airframe connector. |
| Airframe XT60 | Main star split | Positive and negative; positive passes through HCS aperture | 10 AWG silicone | 50--100 mm | Keep the pair together; twist gently if the bend radius permits. Locate the HCS and split within this central run. |
| Main star split | Tekko32 A | Positive and negative | 12 AWG silicone | 50--150 mm; review above 200 mm | Holybro specifies at least 12 AWG. Keep both conductors together. |
| Main star split | Tekko32 B | Positive and negative | 12 AWG silicone | 50--150 mm; review above 200 mm | This is the only high-current pair expected to cross the centre tube under the proposed layout. |
| Each Tekko32 | Its four motors | Three phase conductors per motor; 24 conductors total | Existing motor 20 AWG | Cut the supplied 150 mm leads to suit; avoid extensions | Motor phase lengths need not match exactly. Place ESCs so the stock leads reach without joints. |
| Supplied capacitor | Its Tekko32 battery pads | Positive and negative | Supplied leads | Exposed leads under 10 mm | Fit one 1000 uF/35 V capacitor directly to each board, observing polarity. |
| Post-HCS power node | FC `VBAT` and power ground | Raw battery positive and ground | 22 AWG silicone | Under 150 mm; review above 200 mm | Take this tap after the Hall measurement so FC/peripheral use is included in total current. Twist the pair and secure it against chafing. |
| FC 5 V / GND / `CURR` | HCS `5V` / `G` / `Curr` | Sensor power, ground and analogue current | 28 AWG; twist `Curr` with ground | Under 150 mm; review above 300 mm | Leave HCS `Volt`, `Vbat`, TX and RX unused for this architecture. Configure `BATT_AMP_PERVLT=50` and calibrate. |
| FC motor outputs assigned to ESC A | Tekko32 A signal inputs | Four DShot signals plus signal ground | 26--28 AWG | Under 150 mm; review above 300 mm | Do not rely on connector pin order; map every physical motor explicitly. Keep a signal ground with the bundle. |
| FC motor outputs assigned to ESC B | Tekko32 B signal inputs | Four DShot signals plus signal ground | 26--28 AWG | Under 150 mm; review above 300 mm | Crosses the centre tube beside the remote ESC power pair. Keep the signal bundle distinct from the power pair. |
| Tekko32 A and B telemetry | FC `RX8` / `SERIAL7` | One telemetry conductor from each ESC, joined only at the FC end | 28 AWG | Under 200 mm each; review above 300 mm | ArduPilot permits the ESC telemetry wires to share the configured ESC-telemetry RX input. Do not join the analogue current outputs. |
| FC UART1 and 5 V | XIAO ESP32C6 | TX, RX, 5 V and ground | 28 AWG signal; 26 AWG power | Under 150 mm; review above 300 mm | FC TX1 to XIAO RX and FC RX1 to XIAO TX. Keep away from ESC phase wires. |
| FC | Optional buzzer/status indicator | Signal, 5 V and/or ground as required | 26--28 AWG | Under 200 mm | Part and exact connection remain open, but reserve mounting and pad access. |

With the proposed placement, the centre tube normally contains one 12 AWG positive/negative pair,
four ESC-B DShot wires, one signal ground and one ESC-telemetry wire. Reserve room or a draw cord for
four additional 26--28 AWG sensor conductors. Do not route motor phase wires or the 10 AWG main feed
through the tube. The intended centre-tube geometry is **25 mm outer diameter, 22 mm inner diameter**
(1.5 mm wall). Its bore is comfortably larger than the approximately 16 mm clear bore the planned bundle
needs, so wiring capacity does not constrain it; material, tube strength, bearing interfaces and the
pendulum mechanics still need validation before the geometry is frozen. The current rough STL does not
yet represent the complete hollow tube: its separate central cylinder is approximately 22 mm OD by
60 mm long.

Carbon fibre is conductive and its cut edges are abrasive. No conductor may touch a raw tube edge: use
bonded insulating sleeves or retained grommets at both exits, provide strain relief, and do not use the
frame as an electrical return. All low-voltage devices require an explicit copper ground connection.

## Locked control architecture

- The **laptop is the operator interface**. Do not purchase a conventional RC transmitter or receiver
  for this build unless later testing establishes a specific need for an independent manual backup.
- All time-critical control stays onboard the vehicle. The flight controller, or an onboard companion
  computer if one is later required, runs the attitude, motor and pendulum-control loops. The laptop
  must not close the 400 Hz control loop over a wireless link.
- The laptop link is for experiment start/stop, supervisory commands, target and parameter changes,
  telemetry, plotting and log transfer. Direct manual input from a USB joystick/gamepad remains
  optional and would be transported over the same ground-control link.
- Use the XIAO ESP32C6 as the airborne link, connected to FC UART1 and configured for MAVLink2 at
  115200 baud. Connect XIAO `D3/GPIO21` TX to FC RX1 and `D2/GPIO2` RX to FC TX1, plus regulated 5 V
  and ground. Leave RTS/CTS disabled. The laptop connects directly to its Wi-Fi access point over UDP
  port 14550.
- Do not connect the XIAO's USB and FC-provided 5 V simultaneously unless its power-path behaviour has
  first been verified. Flash and configure it over USB before making the final FC power connection.
- Configure a deterministic response to loss of the laptop link before free flight. ArduPilot's GCS
  failsafe and, if joystick RC overrides are used, the RC-override timeout must be explicitly set and
  tested rather than left as unexamined defaults.

## Current limits and current-sensor caveat

- Expected ordinary total current: roughly **15–20 A**.
- Expected brief worst manoeuvre: roughly **110 A** total (two lateral motors near maximum plus the
  vertical motors maintaining lift).
- Configure a total-current/output limit around **120–140 A** during commissioning. The theoretical
  all-eight-motors-at-maximum figure is about 394 A and is not an allowed operating point.
- The HCS-150A is appropriate for that intentionally limited envelope. Its sensitivity is 20 mV/A,
  so a 3.3 V ADC reaches full scale at about 165 A. It therefore **does not measure 200 A+ peaks**;
  it will saturate. If logging 200 A or more becomes a real requirement, replace this BoM line with
  a genuine 200 A Hall sensor such as the Mauch HS-200-LV and verify its analogue interface before
  purchase. This is a measurement-range issue, not a reason to permit 200 A flight operation.

## Remaining purchases for the basic-hover prototype

Assuming the major items in the first table have been ordered, the following are the remaining build
items. Printed geometry replaces bespoke brackets, but not the fasteners and retention materials.

| Item | Quantity to buy | Notes |
|---|---:|---|
| 10 AWG high-strand-count silicone wire | 1 m red + 1 m black | Combined battery feed before the split. |
| 12 AWG high-strand-count silicone wire | 1 m red + 1 m black | Two ESC power branches. |
| 22 AWG silicone wire | 1 m red + 1 m black | FC raw-battery feed. |
| 26--28 AWG multicolour silicone hookup wire | At least 10 m aggregate | DShot, telemetry, sensor and XIAO wiring. |
| Genuine matching airframe-side XT60 | 2 | One installed and one spare; verify gender and polarity against the batteries. |
| Heat-shrink, braided sleeve and strain-relief materials | 1 assortment | Must cover small signal wiring through the insulated high-current Y junction. |
| M3 socket-head screws, washers and nyloc nuts | 1 assorted-length kit | Through-bolt printed joints; also select motor screws that cannot reach the windings. |
| M3 spacers/standoffs | 1 assortment | Sets electronics clearances without relying on printed threads. |
| FC soft-mount grommets/bobbins | 1 set of 4, if not supplied | Do not mount the FC rigidly to the printed frame. |
| Hook-and-loop battery straps | 2, approximately 20 x 250 mm | Pass through printed frame slots; do not use a printed snap latch as sole retention. |
| Non-slip battery/clamp liner | 1 small sheet | Thin neoprene/rubber; can be replaced by correctly printed TPU parts. |
| Small cable ties | 1 pack | Retain cables and small boards through printed slots. |
| microSD card | 1, if not already fitted | Required for useful ArduPilot onboard logging. |
| Smoke stopper/electronic current limiter | 1 | Must be explicitly rated for at least 25.2 V; prefer at least 27 V for LiHV headroom. |
| Buzzer/status indicator | 1 | Exact part and FC connection still need selection. |
| Charger-to-XT60 output lead | 1, if not supplied with the charger | Verify the SkyRC package contents before buying. |
| Solder, flux and suitable soldering equipment | As required | The iron must have enough thermal capacity for XT60 and 10/12 AWG joints. |
| Indoor restraint/test hardware | To suit the test setup | Use a tether, test rig or enclosure appropriate for an eight-propeller prototype. |
| Carbon tube and print material/service | As required | Required unless already ordered/available. |

Print the component bosses and cradles, cable channels, zip-tie and battery-strap slots, broad two-piece
carbon-tube clamps, landing features and protected tube-exit bushings directly into the prototype. Use
PETG/nylon or TPU as appropriate for the exit bushings and compliant parts; do not leave wire against a
raw carbon edge. Heat-set inserts are optional and need not be bought for the first prototype.

## Decisions still open

- **Airframe detail:** use a 3D-printed plate/body prototype to establish basic hover, vibration and
  packaging behaviour, then replace the body plates with carbon-fibre parts while retaining the
  validated interfaces. The central tube cross-section is selected at 25 mm OD / 22 mm ID; its cut
  length, bearing choice, detailed mounts and final component positions remain open.
- **Arming/safety hardware and later sensors:** not selected yet. The FC already contains the body
  gyro/accelerometer and barometer. Compass, optical flow/rangefinder, arm IMUs and the magnetic
  joint encoder are deliberately deferred until after basic flight.

## Bring-up constraints worth carrying forward

- Check resistance/continuity before connecting a battery.
- First power-up with a smoke stopper/current limiter and no propellers.
- Verify all eight signal mappings and motor directions with props removed.
- With propellers removed, test laptop disconnect, telemetry-radio disconnect and GCS application
  failure; confirm that each produces the configured control release and failsafe behaviour.
- Before installation, soak-test the XIAO/DroneBridge link and then repeat with the motors powered to
  check for resets, packet loss and electromagnetic interference.
- Add propeller load gradually while monitoring total current, ESC temperature and connector
  temperature.
- Provide strong airflow over both Tekko32 boards during high-current tests.
