"""Rise-time + steady-error analysis of a ModeAccel flight, from the full-rate ArduPilot .BIN.

The Godot [grade] log is only ~4 Hz -- too coarse for a 40 ms rise time. This reads the .BIN IMU
(full rate) and RCIN (command), and for each command step reports:
  - rise time  = time from the command change until realised accel first reaches (and holds ~20 ms)
                 within a 5% band of the command,
  - settled    = mean realised accel over the last 30% of the hold, and its % error vs command.
Flags steps that meet BOTH <=40 ms rise AND <=5% steady error.

Command decode matches the firmware: lat = (C1-1500)/500*ACCEL_LAT_CMD_MSS, up = (C3-1500)/500*g.
NOTE: realised accel here is the IMU (what the controller senses); the Godot grader is the truly
independent check. In sim they match closely (no IMU bias), so this is fine for rise-time.
Usage:  python3 risetime.py <path/to/NNNN.BIN>  [LAT_CMD_SCALE]
"""
import sys
from pymavlink import mavutil

LAT_CMD = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
G = 9.81
BAND_FRAC = 0.05
BAND_FLOOR = 0.2      # m/s^2 -- 5% of a small command is below IMU noise; use an absolute floor
HOLD_CONFIRM = 0.020  # s -- realised must stay in band this long to count as "arrived"
RISE_TARGET = 0.040   # s

m = mavutil.mavlink_connection(sys.argv[1])
rc1 = rc3 = 1500
T = []
while True:
    x = m.recv_match(blocking=False)
    if x is None:
        break
    ty = x.get_type()
    if ty == 'RCIN':
        rc1, rc3 = x.C1, x.C3
    elif ty == 'IMU' and getattr(x, 'I', 0) == 0:
        t = x.TimeUS / 1e6
        cl = (rc1 - 1500) / 500.0 * LAT_CMD
        cu = (rc3 - 1500) / 500.0 * G
        T.append((t, cl, cu, x.AccY, -x.AccZ - G))   # t, cmd_lat, cmd_up, a_lat, a_up


def analyze(cmd_idx, val_idx):
    steps = []                       # (i0, cmd)
    prev = None
    for i, row in enumerate(T):
        c = row[cmd_idx]
        if prev is None or abs(c - prev) > 0.05:
            steps.append((i, c))
            prev = c
    out = []
    for s, (i0, cmd) in enumerate(steps):
        t0 = T[i0][0]
        i_end = len(T) - 1
        for j in range(i0 + 1, len(T)):
            if abs(T[j][cmd_idx] - cmd) > 0.05 or T[j][0] - t0 > 0.6:
                i_end = j - 1
                break
        if abs(cmd) < 0.2:
            continue
        band = max(BAND_FRAC * abs(cmd), BAND_FLOOR)
        rise = None
        for j in range(i0, i_end + 1):
            if abs(T[j][val_idx] - cmd) <= band:
                ok = True
                for jj in range(j, i_end + 1):
                    if T[jj][0] - T[j][0] > HOLD_CONFIRM:
                        break
                    if abs(T[jj][val_idx] - cmd) > band:
                        ok = False
                        break
                if ok:
                    rise = T[j][0] - t0
                    break
        tail = T[max(i0, i_end - max(1, int((i_end - i0) * 0.3))):i_end + 1]
        settled = sum(r[val_idx] for r in tail) / len(tail)
        out.append((t0, cmd, rise, settled))
    return out


for name, res in [("LATERAL", analyze(1, 3)), ("VERTICAL", analyze(2, 4))]:
    print("== %s ==" % name)
    print("   t(s)    cmd   rise(ms)  settled   err     err%%   meets(<=40ms & <=5%%)")
    for t0, cmd, rise, settled in res:
        err = settled - cmd
        errpct = 100 * err / abs(cmd)
        rms = "%.0f" % (rise * 1000) if rise is not None else "never"
        meets = rise is not None and rise <= RISE_TARGET and abs(errpct) <= 5.0
        print("  %6.1f  %5.1f  %8s  %7.2f  %+.2f  %+5.1f%%   %s" %
              (t0, cmd, rms, settled, err, errpct, "YES" if meets else ""))
