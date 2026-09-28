# Install and check the lift-motor limit update over USB

1. Remove all propellers, disconnect the flight battery, and connect FC USB.
   Close other programs using the FC port before opening MAVProxy.
2. In MAVProxy, save the live configuration before uploading:

   ```text
   param fetch
   param save firmware/logs/before-lift-limits.parm
   ```

   Wait for the parameter fetch to complete before saving. Keep this backup:
   it includes the actual accelerometer calibration and battery-voltage scale.
   Exit MAVProxy to release the USB port.
3. From the repository directory, upload the new application:

   ```bash
   firmware/.venv/bin/python firmware/ardupilot/Tools/scripts/uploader.py --port '/dev/ttyACM*' --baud-flightstack 115200 firmware/dist/base-drone-StellarH7V2/arducopter.apj
   ```

   Connect only the intended FC. Require successful board identification, upload
   and verification. Do not use force or erase options.
4. Reconnect MAVProxy and confirm the startup version contains
   `BaseDrone 4.7.1 liftlimits1`. Confirm DShot300 on outputs 1–8 and no scripting
   errors. Both script checksums remain `934390`; the Lua mixer is unchanged.
5. Fetch and save parameters again as `firmware/logs/after-lift-limits.parm`.
   Compare with the backup. Sensor calibration, AHRS trims/orientation, motor
   mapping, rate gains, battery scale and failsafe settings should be retained.
   Do not load `base-drone.parm` over the live calibration.
6. With props still removed, check the attitude display on level supports and
   during hand tilts. Verify each output and corrective direction against
   `HARDWARE-MIXER.md`. Run the existing props-off bench checks once motor power
   is connected; record their logs and compare low-throttle differential outputs.
7. Review these checks before repeating a restrained loaded test. This update
   limits impossible negative lift requests and reports rotational saturation;
   it does not prevent all integral growth while the ground or a tether blocks
   motion. Do not tune against a taut restraint or infer flight stability from
   props-off outputs. Review attitude, integral, motor outputs and first movement
   together before progressing to free flight.

To restore the previous local package, repeat the upload procedure using
`firmware/dist/before-lift-limits/arducopter.apj`. Its manifest is preserved in
the same directory. This is a rollback image, not a flight-approved baseline.

The update does not change gains, calibration, motor idle, current monitoring,
pilot controls or failsafe actions. The keyboard lift tool remains a short
tethered-test interface; it does not provide steering for a free hover.
