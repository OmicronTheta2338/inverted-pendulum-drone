# Run a short tethered lift test

Use only for a supervised first hop limited to a few centimetres by a suitable
restraint. This program is not a free-flight interface. It does not hold position
or altitude, and its stop action cuts throttle and requests disarm rather than
landing. Printed loop and anchor strength remain physical checks, not software checks.

1. Fit the F5146 props to the verified rotation/thrust mapping. Secure the FC,
   battery, mounts and wiring. Set up the level frame and short tethers with power
   disconnected. Keep all lines clear of every prop, including at maximum permitted
   movement. Keep everyone outside the propeller area and the stop key accessible.
2. Connect battery power, connect the laptop to DroneBridge Wi-Fi, and close MAVProxy.
3. From the repository directory, run:

   ```bash
   firmware/.venv/bin/python firmware/lift_test.py
   ```

4. The program selects the FC heartbeat and requests telemetry. It verifies the same
   commissioning configuration as the bench tool, a connected 6S battery, level
   attitude within 3 degrees, and STABILIZE. It does not change mode or parameters.
5. Read the prompt and type `RUN`. After another state check it arms at minimum
   throttle. Wait for `ARMED at minimum throttle` before adjusting throttle.
6. Press Up arrow to request throttle 1050. Further presses increase it by 25, at most
   once every 0.4 seconds. Down arrow reduces it by 25. The selected setting persists
   until changed or stopped; there is no H requirement, inactivity timer or run-time
   limit. Observe the drone rather than attempting to infer
   lift from displayed throttle. There is no calibrated hover setting yet.
7. At the first lift, or any tipping, oscillation or strain against taut tethers,
   press Space. This cuts throttle and requests ordinary disarm. Expect a drop
   through the small permitted height; this is not a landing command.
8. Require `STOP CONFIRMED` before approaching. If stop cannot be confirmed,
   disconnect battery power. Disconnect the battery before touching props/tethers.

Space or Ctrl+C cuts throttle and requests disarm. Unrelated keys are ignored.
Automatic stops remain for terminal closure, stale telemetry, leaving STABILIZE,
tilt beyond 12 degrees and battery voltage outside 21–25.8 V. The FC's own
arming/failsafe behaviour is unchanged, including auto-disarm at minimum throttle.

The pilot-input ceiling is 1500. It is NOT a ceiling on individual motor outputs:
the FC still runs stabilisation and may request stronger corrections. All eight
motors may run. Software cleanup depends on the program and link remaining
functional; the separately tested FC link-loss response remains the fallback.

Telemetry is saved in `firmware/logs/lift-*.jsonl`. The software tests use
a fake link and do not establish loaded stability or certify the physical restraints:

```bash
firmware/.venv/bin/python firmware/tests/test_lift_test.py
```
