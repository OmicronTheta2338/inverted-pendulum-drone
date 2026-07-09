"""MAVLink side of the headless ModeAccel auto-test.

Connects to the SITL MAVLink endpoint (tcp:127.0.0.1:5760 by default), waits for a GPS 3D fix
(so the PARK auto-takeoff has an EKF origin), sets mode ACCEL (30), force-arms, then just monitors
while the Godot side (main.gd, AUTOTEST=1) auto-takes-off and runs its scripted accel profile.

NOTE: SITL blocks on "Waiting for connection" until a MAVLink client connects to 5760 -- that's us.
Usage:  python3 drive.py [RUN_SECONDS]
"""
import sys, time
from pymavlink import mavutil

RUN = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
m = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
m.wait_heartbeat(timeout=40)
print("HB sys", m.target_system)
m.mav.request_data_stream_send(m.target_system, m.target_component,
                               mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)

# wait for GPS 3D fix (EKF origin) so PARK auto-takeoff has a position reference
t = time.time(); fix = 0
while time.time() - t < 60:
    msg = m.recv_match(type=['GPS_RAW_INT', 'STATUSTEXT'], blocking=True, timeout=2)
    if not msg: continue
    if msg.get_type() == 'STATUSTEXT': print("  ", msg.text)
    elif msg.get_type() == 'GPS_RAW_INT':
        fix = msg.fix_type
        if fix >= 3:
            print("GPS 3D fix"); break
time.sleep(4)  # let EKF origin/velocity settle

def set_mode_accel():
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 30, 0, 0, 0, 0, 0)

def arm():
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)  # 21196 = force

print("set mode ACCEL(30)"); set_mode_accel(); time.sleep(1)
print("arming (force)...")
armed = False; t = time.time()
while time.time() - t < 20:
    set_mode_accel(); arm()
    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        armed = True; print("ARMED  custom_mode=%d" % hb.custom_mode); break
if not armed:
    print("!!! ARM FAILED")

# fly: Godot auto-drives takeoff + profile. Log altitude periodically.
t = time.time(); lastp = 0
while time.time() - t < RUN:
    msg = m.recv_match(type=['GLOBAL_POSITION_INT', 'STATUSTEXT'], blocking=True, timeout=2)
    if not msg: continue
    if msg.get_type() == 'STATUSTEXT':
        print("  ", msg.text)
    elif msg.get_type() == 'GLOBAL_POSITION_INT':
        el = time.time() - t
        if el - lastp >= 3.0:
            lastp = el
            print("  t=%2.0f  alt=%.1f m" % (el, msg.relative_alt / 1000.0))
print("disarm"); m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)
