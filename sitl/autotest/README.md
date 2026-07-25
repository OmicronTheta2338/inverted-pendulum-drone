# ModeAccel headless auto-test

Automated, no-human test of the direct-acceleration flight mode (**ModeAccel / mode 30**). It boots
Godot + ArduPilot SITL, takes off, commands a scripted sequence of accelerations, and grades the
**realised** base acceleration against **independent Godot physics truth** — so you can verify and
tune the controller from data without flying it by hand.

## What it does

1. Launches **Godot headless** (`sim-v-1`) with env `AUTOTEST=1`. That makes `main.gd` ignore the
   keyboard and run a built-in state machine: park-auto-takeoff to ~10 m, then step through
   `auto_profile` (a list of `[t_end, lat_level, vert_level]` commands), then hold.
2. Launches **`arducopter` SITL** using the Godot rigid body as the physics backend
   (`--model JSON:127.0.0.1`, JSON port **9002**; MAVLink on **tcp:5760**).
3. `drive.py` connects over MAVLink, waits for a GPS fix, **sets mode ACCEL (30)** and **force-arms**.
   (SITL sits at *"Waiting for connection"* until a MAVLink client connects — that client is `drive.py`.)
4. The Godot-side grader (`_grade()` in `main.gd`) prints `[grade] lat cmd=… true=… | up cmd=… true=…`
   ~4×/s. `true` is the drone's own `dv/dt` (physics truth), **not** the IMU signal the controller
   closes on — an independent check.
5. `analyze.py` groups those lines per command and prints settled accuracy + overshoot.

## Prerequisites

- `godot` (v4.x) on `PATH` (headless works — no display needed).
- ArduPilot SITL built: `cd ../../../ardupilot && ./waf copter` (binary at
  `ardupilot/build/sitl/bin/arducopter`; override with `AP_BIN=…`).
- `pymavlink`: `pip install --user pymavlink`.
- Run from anywhere — paths resolve relative to this script.

## Run it

```bash
cd sitl/autotest
./run_autotest.sh                 # ~60 s (EKF cold-start alone is ~25 s); 1 pendulum arm (default)
./run_autotest.sh 45              # custom duration
NUM_ARMS=0 ./run_autotest.sh 45   # BARE drone (no pendulum) — the accel-mode calibration reference
NUM_ARMS=1 ./run_autotest.sh 45   # one pendulum arm
NUM_ARMS=2 ./run_autotest.sh 45   # two pendulum arms
```

**Pendulum arms (0/1/2).** Each hanging arm applies a reaction force on the frame; ModeAccel cancels it
with an **analytic reaction-force feed-forward** so the *base* accel still tracks the command. Godot sends
each arm's angle + rate + the arm count to the controller over spare RC channels (`rc_6`=count,
`rc_7/8`, `rc_9/10` = per-arm angle/rate); a real rig would read these from a hinge encoder. The
feed-forward is `a_pend = (m_b/m_d)·[(a_d−g)·u − L·θ̇²]·u` summed over arms (see `mode_accel.cpp`), which
covers both the static hanging weight and the dynamic swing. Set `NUM_ARMS` to pick the config; it
overrides the `enable_pendulum` export in `main.gd`.

**Rebuild the firmware first if you changed the controller** (`ArduCopter/mode_accel.cpp`):
`( cd ../../../ardupilot && ./waf copter )` — `run_autotest.sh` does **not** rebuild.

## Reading the output

```
cmd(lat,up)       lat_set  lat_max    up_set   up_max   (set=settled mean, max=worst overshoot)
(  0.0,  3.3)  n=10     -0.00    0.02     3.25     6.18
(  3.0,  0.0)  n= 8      2.94    3.01    -0.00    -0.01
```

- `lat_set` / `up_set` — settled realised accel (mean of last 3 samples of that command). Compare to
  the commanded value in `cmd(lat,up)`. **This is the accuracy number.**
- `lat_max` / `up_max` — worst-magnitude sample during the command = transient overshoot.
- Lines flagged `LAT/UP steady err …` exceed ~15% + 0.2 m/s² on a commanded axis.

Full logs land in `sitl/autotest/out/` (`godot.log` has every `[grade]` line; `sitl.log`,
`drive.log`). The ArduPilot binary log (`.BIN`) is in `sitl/logs/` — use it for **fine-grained /
rise-time** analysis (the grader is only ~4 Hz; the `.BIN` IMU/RCIN is full-rate).

**Rise time / 40 ms check (use the full-rate Godot truth).** The grader emits per-physics-frame `[trace]`
lines for 150 ms after every command change — full-rate *independent* truth (the drone's `dv/dt`). Analyse
them with:
```bash
python3 tracetime.py out/godot.log 1.0    # rise-to-±1% (holds 18 ms) + settled err, flags ≤40ms & ≤TOL%
python3 summary.py   out/godot.log        # per step: dead-time, reach±5%, reach±1%, settled, overshoot
```
`risetime.py ../logs/00000NN.BIN` does the same from the **ArduPilot IMU** — but note the IMU is what the
controller closes on; with a pendulum attached the **`[trace]` grader is the true base accel**, so prefer
`tracetime.py`/`summary.py`. (The sim IMU now reports true specific force *including* the hinge reaction,
so a real accelerometer's feedback is modelled correctly and the integral no longer fights the reaction FF.)

**~22 ms transport artifact.** `summary.py`'s `dead` column shows ~22 ms before *any* response — that is
the async Godot↔ArduPilot UDP round-trip (RC command → servo), **not** controller lag, and it is identical
on the bare drone. On real hardware the controller is on-board, so this delay is absent; the controller's
own response (post-transport) is ~13–20 ms. Read the 40 ms numbers with this offset in mind.

## Changing the test

- **Command profile**: edit `auto_profile` in `sim-v-1/main.gd` (rows `[t_end_seconds, lat_level(-4..4),
  vert_level(-3..3)]`; level→accel is `level/4·ACCEL_LAT_CMD` lateral, `level/3·g` vertical). No rebuild
  (GDScript) — just re-run.
- **Controller gains / calibration**: edit the `#define`s in `ArduCopter/mode_accel.cpp`
  (`ACCEL_KP_*`, `ACCEL_KI_*`, `ACCEL_LAT_MAX_MSS`, `ACCEL_VERT_THR_PER_MSS`, `ACCEL_REF_TAU`, …),
  **rebuild**, then re-run.

## Gotchas

- **Manual flying is unaffected**: the auto-test only activates when `AUTOTEST=1` is set (the
  `run_autotest.sh` sets it just for that Godot process). Launching Godot normally = keyboard control.
- **EKF cold-start ~25 s**: the drone won't take off until the EKF is GPS-locked. That's why a run is
  ~55–60 s and the takeoff marker appears late. `SIM_SPEEDUP` can't help — the Godot bridge is
  real-time (`no_lockstep`).
- If the accuracy table is **empty** and there is no `[auto] hover settled` marker, **the drone never
  took off** — the profile only starts once it reaches ~9 m. This is a *controller* problem, not a
  harness one: park's climb throttle must exceed the *true* hover throttle. Check `sitl/logs/*.BIN`
  (`CTUN.ThO` vs hover, and `RCIN.C2==2000` = park engaged). If throttle sits ≈ hover with no climb,
  the vertical FF hover-baseline / slope is mis-calibrated. (If the profile started but didn't finish
  in time, increase `RUN_SECONDS` instead.)
- If SITL logs *"Waiting for connection"* forever, nothing connected to 5760 — check `drive.py` ran /
  pymavlink is installed.
- Stale processes: the script `pkill`s Godot/arducopter on entry and exit. If a run wedges,
  `pkill -f 'godot --headless'; pkill -f arducopter`.

## Files

| file | role |
|---|---|
| `run_autotest.sh` | orchestrator (launch, drive, analyze, clean up) |
| `drive.py` | MAVLink: wait-for-fix, set mode ACCEL, force-arm, monitor |
| `analyze.py` | parse `[grade]` lines (~4 Hz) → per-command **settled** accuracy table |
| `tracetime.py` | parse full-rate `[trace]` truth → rise-to-±TOL% (holds 18 ms) + settled, flags ≤40 ms |
| `summary.py` | parse `[trace]` → dead-time, reach±5%, reach±1%, settled, overshoot per step |
| `risetime.py` | parse a `.BIN` → rise from the **IMU** (grader/`tracetime.py` is the independent truth) |
| `out/` | run logs (git-ignored) |

The Godot-side hooks live in `sim-v-1/main.gd`: `auto_test`, `auto_profile`, `_auto_drive()`, and the
`AUTOTEST` env read in `_ready()`.
