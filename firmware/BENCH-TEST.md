# Run a timed props-off bench test

Use this guide with the physical StellarH7V2, the embedded mixer checksum 934390,
and the existing calibrated parameters. Do not use this program for flight.

## Check USB communication with the battery disconnected

1. Remove all propellers. Keep the battery disconnected and USB connected.
2. Close MAVProxy with Ctrl+C. Only one program can own the FC's USB connection.
3. From the repository directory, run:

   ```bash
   firmware/.venv/bin/python firmware/bench_test.py
   ```

   The program automatically selects the unique StellarH7V2 USB by-id path. It
   displays armed state, attitude and battery voltage, records telemetry in
   `firmware/logs/bench-*.jsonl`, then exits. It sends no RC overrides or arming
   requests. USB-only battery voltage is not a calibrated battery reading.
   Use `--port /dev/ttyACM0` if automatic selection fails, substituting the actual
   port. Never run it alongside MAVProxy.

## Test roll correction with the stationary frame tilted

1. With battery power disconnected, support and secure the entire frame with its
   right side 3–10 degrees lower than its left side. Keep pitch within 3 degrees
   of level. Leave the FC secured in its normal mount. Keep all shafts and wires
   clear; do not hold or move the frame during the powered test.
2. Connect the battery and keep USB connected.
3. Run:

   ```bash
   firmware/.venv/bin/python firmware/bench_test.py --test roll-right-down
   ```

4. The program requires STABILIZE, disarmed state, fresh telemetry, battery voltage
   21.6–25.8 V, the expected configuration, and the specified tilt. It does not
   change parameters or flight mode. Parameter checks use paced requests, retry missing replies, and stop
   after a 30-second overall deadline. Missing replies do not indicate wrong values.
   At the prompt, type `RUN` to authorise the
   test, or press Enter to cancel. It checks state again after the prompt.
5. It sends neutral inputs, requests ordinary arming, then commands throttle 1050
   with centred lateral/pitch/yaw inputs for two seconds. Ctrl+C ends it early.
   It then requests minimum throttle and ordinary disarm. It releases overrides
   only after fresh telemetry confirms disarmed state and stopped outputs 1–8.
6. Require `STOP CONFIRMED` before approaching the frame. If it prints
   `STOP NOT CONFIRMED`, disconnect the battery. A program crash, disconnected
   cable or failed USB write can prevent cleanup; the configured FC failsafes
   remain the fallback and still require separate hardware verification.
7. Inspect the recorded attitude and motor outputs. For right-side-down roll,
   expect increased right-side lift M3/M5 relative to left M1/M7, accounting for
   simultaneous pitch/yaw correction. This checks correction direction; it does
   not establish stability or gains under propeller load.

## Select another test

Use `--test NAME`. The duration is fixed at two seconds. Command tests require
the frame within 3 degrees of level; their roll/pitch/yaw/throttle tuples are:

| Name | Channels 1, 2, 3, 4 |
|---|---|
| pitch-up | 1500, 1600, 1050, 1500 |
| pitch-down | 1500, 1400, 1050, 1500 |
| right | 1700, 1500, 1050, 1500 |
| left | 1300, 1500, 1050, 1500 |
| yaw-right | 1500, 1500, 1050, 1700 |
| yaw-left | 1500, 1500, 1050, 1300 |
| roll-right-down | 1500, 1500, 1050, 1500 |
| nose-down | 1500, 1500, 1050, 1500 |

For `nose-down`, support the stationary frame nose-down by 3–10 degrees with
roll within 3 degrees of level. Expect increased front lift M1/M3 relative to
rear M5/M7. Set up supports with battery disconnected before every tilt test.

## Test ground failsafes

For the installed DroneBridge link, add `--port tcp:192.168.2.1:5760` to
each command below. Connect the laptop to its Wi-Fi and close MAVProxy first.
The tool selects the FC heartbeat (component 1) and excludes bridge heartbeats
from its armed-state checks. USB may remain attached for power; the selected
connection determines the data path being tested.

Keep the frame secured and level, all propellers removed, battery and USB connected,
and MAVProxy closed. These tests arm at minimum throttle and never request throttle
above 1000. The six-axis mixer should keep the motors stopped at ground idle.

1. Run `firmware/.venv/bin/python firmware/bench_test.py --test rc-loss`.
2. Type `RUN` after checks pass. The tool establishes neutral inputs, arms normally,
   keeps them for 2.5 seconds, then stops RC overrides for four seconds while
   continuing GCS heartbeats. It sends no disarm command in that observation window.
3. Require a `FC FAILSAFE RESULT` with `passed: True`, followed by `STOP CONFIRMED`.
   Expected FC text includes `Radio Failsafe - Disarming`. The result requires
   radio-failsafe text, fresh disarmed state and stopped outputs before cleanup,
   with no GCS failsafe in the RC-only window. Failure triggers ordinary disarm
   cleanup and an error; resolve the result before running the next test.
4. Run `firmware/.venv/bin/python firmware/bench_test.py --test link-loss`.
   Type `RUN`. This time the tool stops all outgoing traffic for four seconds,
   while still receiving the FC's telemetry. It requires radio and GCS failsafe
   messages plus disarmed/stopped telemetry before cleanup. It restores only
   heartbeats for two seconds and verifies the FC remains disarmed.
5. Require the same successful result and stop confirmation. Disconnect the battery
   if stop is not confirmed. Save the printed telemetry paths for review.

The complete-link test does not isolate the GCS action while armed: RC loss can
disarm first. Both tests exercise ground disarming, not airborne LAND behaviour.
The configured 10-second auto-disarm timer exceeds the observation window.
Motor-command tests still use explicit cleanup and are not failsafe tests.

## Check the software without hardware

```bash
firmware/.venv/bin/python firmware/tests/test_bench_test.py
```

The fake-link tests cover command duration, normal cleanup, interruption,
arming rejection, cancellation, already-armed refusal, telemetry loss, parameter
retries and missing replies. Loss-test checks verify withheld traffic, absence
of tool disarm commands during observation, and cleanup if FC failsafe is absent.
They do not validate USB timing, FC responses or physical motor stops.
