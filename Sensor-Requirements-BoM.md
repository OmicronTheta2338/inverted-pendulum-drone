# Sensor Requirements — Pendulum-Balancing Octocopter (for the BoM)

The specs the real sensors must hit for the hardware to behave like the balance simulation. Every
figure here is backed out from the sim's sensor-noise model (`sim-v-1/main.gd`, sensor layer). If a
part meets these, the controller that swings up and balances the double pendulum in sim is running on
inputs no worse than the real ones.

- **Source:** `sim-v-1/main.gd` sensor layer · **Control loop:** 400 Hz · **State:** GPS-denied · **Rev:** 2026-07-18

## Read the numbers right — the load-bearing bit

The sim's noise values are per-sample RMS at the 400 Hz loop rate, not datasheet units. Datasheets
quote noise density (µg/√Hz, mdps/√Hz). Convert with:

```
σ_sample = density × √BW
```

Every "hardware target" below uses BW = 200 Hz (Nyquist, the worst case, all noise in-band). Any
anti-alias or low-pass filter you add makes the real requirement easier than shown. Shop against the
density target rather than the raw σ, or you'll badly over-buy.

## Binding items — what to actually sweat

Everything else on this sheet is met by any modern flight IMU. These four are the constraints; three are
mechanical, not a purchase.

1. **Optical flow + rangefinder** *(subsystem)* — what makes velocity and position possible. GPS-denied, so
   3 cm/s velocity needs a flow sensor plus a rangefinder for scale. The whole fore/aft hold depends on it.
2. **≥14-bit magnetic encoder + diametric magnet** *(part)* — arm-2 joint angle. Needs an on-axis chip
   and a magnet on the shaft; plan the mechanics, not just the BoM line.
3. **Vibration isolation** *(mechanical)* — keep vibration at the flight IMU ≲ 25 mg RMS. Soft-mount the
   FC and balance the props; the accel "noise" budget is really this.
4. **Rigid, aligned arm-1 IMU mount** *(mechanical)* — arm-1 angle error maps 1:1 to mount misalignment.
   Spend the effort here, not on an exotic IMU.

## Flight IMU — gyro + accelerometer on the FC

Comfortably met by current parts; the only real constraint here is mechanical vibration.

| Channel | Sim assumption | Hardware target | Example part | Verdict |
|---|---|---|---|---|
| Gyro noise | 0.05 °/s RMS @400 Hz | ≤ 3.5 mdps/√Hz | ICM-42688-P (2.8), BMI270 (~3.5) | Easy |
| Gyro bias stability | 0.5 °/s/√hr walk | good-MEMS in-run bias | any flight-grade IMU | Easy |
| Accel noise | 0.25 m/s² (≈25 mg) RMS | ≤ 1.8 mg/√Hz (sensor part ~350 µg/√Hz) | any flight IMU | Easy |
| **Vibration at IMU** | 36 mg tone + throttle-scaled | isolate to ≲ 25 mg RMS residual | soft-mount + balanced props | **Binding** |
| Range | ±36 mg + catch accels | ±8–16 g · ±2000 °/s | standard | Easy |
| Rate / latency | 2.5–8 ms, 400 Hz loop | ≥400 Hz out (≥1 kHz internal) · ≤8 ms | standard | Easy |

The accel "noise" figure is dominated by the modelled rotor-vibration residual, so it's a
vibration-isolation target, not a chip-selection one; no accelerometer datasheet fixes a shaking mount.

## State estimate — attitude / velocity / position

EKF outputs, not direct sensors. Attitude falls out of the IMU above; velocity and position are the binding
subsystem because the vehicle is GPS-denied.

| Channel | Sim assumption | Implication for the BoM | Example part | Verdict |
|---|---|---|---|---|
| Attitude | 0.1° RMS · 8 ms lag | EKF output — met by the IMU above; no separate buy | — | Easy |
| **Velocity** | 0.03 m/s RMS · 20 ms lag | needs optical flow + rangefinder (3 cm/s indoors) | flow camera + TF-Luna / VL53L1X | **Binding** |
| **Position** | 0.02 m RMS + drift 0.015 m/√s | position may drift; you need velocity *aiding*, not an absolute fix | (as above) | **Binding** |

This is the one place the sim is lenient in assumption but demanding in hardware: it tolerates slow
position drift, but holding 3 cm/s velocity with no external reference realistically requires a flow sensor
plus a rangefinder for flow scale/altitude. Scrutinise this line hardest.

## Arm-angle sensors — the pendulum instrumentation

Deliberately asymmetric by design: arm 1 (proximal) has no room for an encoder so it senses angle via dual
IMUs; arm 2 (distal) carries an over-the-joint magnetic encoder.

| Sensor | Sim assumption | Hardware target | Example part | Verdict |
|---|---|---|---|---|
| Arm 1 · angle | 0.15° RMS · 8 ms | 2× IMU fused vs body — noise easy, **mount is the spec** | ICM-42688-P ×2 | Moderate |
| Arm 1 · rate | 0.4 °/s RMS | differential gyro | (same IMUs) | Easy |
| Arm 2 · angle | 0.10° RMS · LSB 0.05° (~13-bit) · 2.5 ms | ≥14-bit on-axis magnetic encoder + diametric magnet | AS5048A / AS5047P / MT6835 | Moderate |
| Arm 2 · rate | 1.0 °/s RMS | differentiated encoder (noisier, expected) | (same encoder) | Easy |

Arm-1's 0.15° is achievable noise-wise with any decent IMU because the gravity anchor is heavily low-passed,
so don't over-buy the IMU; buy mounting rigidity and alignment. The residual maneuver-lean (the anchor
tilting under lateral acceleration) is fundamental, and no sensor removes it.

**Verdict key:** *Easy* = any modern part clears it · *Moderate* = specific part or mechanical effort ·
*Binding* = the constraint, scrutinise.

## Not captured by the sim

Real requirements the sensor model doesn't represent; don't let their absence here drop them from the BoM.

- **Heading / magnetometer** — no compass or yaw source is modelled, but real ArduPilot will want one. Budget
  a compass or an alternative yaw reference.
- **Temperature drift, scale-factor, misalignment** — all real IMU error terms, none in the model; factor into margin.
- **Battery voltage / current sense** — the sim assumes the FC cannot compensate voltage sag, so it's not
  strictly required, but a voltage sense is what would let you compensate it later. Worth a line.

## These are soft targets

The σ values were seeded as mid-grade engineering guesses, not hard-derived requirements. The BoM target is
"match or beat these." If one turns out to be the cost driver (most likely the optical-flow subsystem or
the encoder), the right move isn't to pay for the guessed number: send the real candidate part's datasheet
spec and re-run the balance demo at that number to confirm it still holds. That closes the loop the correct
way round: the hardware sets the sim, not a guess setting the hardware.

## Appendix — raw sim parameters

The exact values these targets derive from, for traceability back to `sim-v-1/main.gd`.

| Parameter | Value | Meaning |
|---|---|---|
| `sig_gyro_dps` | 0.05 °/s | gyro white noise, per-sample RMS |
| gyro bias walk | 0.5 °/s/√hr | gyro bias random walk |
| `sig_accel` | 0.25 m/s² | accel broadband white (≈25 mg) |
| `vib_accel` / `vib_f_hover` | 0.35 m/s² / 90 Hz | rotor-vibration tone amp @hover / fundamental |
| `sig_att_deg` | 0.1 ° | EKF attitude noise |
| `sig_vel` | 0.03 m/s | velocity NED noise |
| `sig_pos` | 0.02 m | position NED noise |
| `pos_drift_rate` | 0.015 m/√s | IMU-only dead-reckon position drift |
| `sig_theta0_deg` / `sig_thetadot0_dps` | 0.15 ° / 0.4 °/s | arm-1 angle / rate (2×IMU fused) |
| `sig_theta1_deg` / `sig_thetadot1_dps` | 0.10 ° / 1.0 °/s | arm-2 angle / rate (encoder) |
| `enc_lsb_deg` | 0.05 ° | arm-2 encoder quantization (~13-bit) |
| `lat_imu_ms` | 2.5 ms | IMU (accel/gyro) latency |
| `lat_ekf_att` / `vel` / `pos_ms` | 8 / 20 / 20 ms | EKF attitude / velocity / position lag |
| `lat_arm_ms` / `lat_arm_ang_imu_ms` | 2.5 / 8 ms | arm-2 encoder read / arm-1 fused-angle lag |
