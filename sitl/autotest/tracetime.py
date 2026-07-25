"""Rise-time + settle analysis from the Godot [trace] lines (full-rate, ~400 Hz, INDEPENDENT truth).

Unlike risetime.py (which reads the ArduPilot IMU), this uses the Godot grader's own dv/dt body accel,
which INCLUDES the pendulum reaction force -- the IMU in the sim does NOT, so it is blind to the pendulum.
So this is the correct metric for validating the pendulum reaction-force feed-forward.

main.gd _grade prints, per physics frame for 150 ms after each command change:
    [trace] dt=  12.5 lat_cmd= 12.00 lat=  -5.00 up_cmd=  0.00 up=  -0.83
dt is ms since the command change. Each command change opens a fresh block (dt resets to ~0).

Per block, on whichever axis actually changed, report:
  - rise(ms): first dt at which true accel enters a +-band of cmd AND stays in-band for >= HOLD ms,
  - settled : mean true accel over the last SETTLE_MS of the window, and its error vs cmd.
Flags a step meeting BOTH <= RISE_TARGET ms and <= TOL steady error.
Usage:  python3 tracetime.py <godot.log> [TOL_PCT]
"""
import sys, re

TOL_PCT = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0   # steady-error target (percent)
BAND_FLOOR = 0.12       # m/s^2 -- absolute band floor (1% of a small cmd is below trace resolution)
HOLD = 18.0             # ms the response must stay in-band to count as "arrived"
RISE_TARGET = 40.0      # ms
SETTLE_MS = 40.0        # window (ms, at the END of the 150ms trace) to average for the settled value

pat = re.compile(r"\[trace\] dt=\s*(-?\d+\.\d+) lat_cmd=\s*(-?\d+\.\d+) lat=\s*(-?\d+\.\d+) up_cmd=\s*(-?\d+\.\d+) up=\s*(-?\d+\.\d+)")

blocks = []
cur = None
last_dt = 1e9
for line in open(sys.argv[1]):
    mo = pat.search(line)
    if not mo:
        continue
    dt, lc, lt, uc, ut = map(float, mo.groups())
    if dt < last_dt - 1.0:        # dt reset -> new block
        cur = []
        blocks.append(cur)
    last_dt = dt
    cur.append((dt, lc, lt, uc, ut))


def analyze_axis(block, cmd_i, val_i, prev_cmd):
    cmd = block[-1][cmd_i]
    if abs(cmd - prev_cmd) < 0.05:     # this axis didn't step in this block
        return None
    band = max(TOL_PCT / 100.0 * abs(cmd), BAND_FLOOR)
    win_lo = block[-1][0] - SETTLE_MS
    tail = [r for r in block if r[0] >= win_lo]
    settled = sum(r[val_i] for r in tail) / max(1, len(tail))
    rise = None
    for k, r in enumerate(block):
        if abs(r[val_i] - cmd) <= band:
            ok = True
            for r2 in block[k:]:
                if r2[0] - r[0] > HOLD:
                    break
                if abs(r2[val_i] - cmd) > band:
                    ok = False
                    break
            if ok:
                rise = r[0]
                break
    return cmd, rise, settled, band


print("axis  t_cmd   cmd   rise(ms)   settled    err     err%%   meets(<=%dms,<=%.0f%%)" % (RISE_TARGET, TOL_PCT))
prev_lc = prev_uc = 0.0
n_meet = n_tot = 0
for b in blocks:
    if len(b) < 5:
        prev_lc = b[-1][1]; prev_uc = b[-1][3]; continue
    for label, ci, vi, pc in [("LAT", 1, 2, prev_lc), ("UP", 3, 4, prev_uc)]:
        res = analyze_axis(b, ci, vi, pc)
        if res is None:
            continue
        cmd, rise, settled, band = res
        err = settled - cmd
        errp = 100 * err / abs(cmd) if abs(cmd) > 1e-6 else 0.0
        rms = "%.0f" % rise if rise is not None else "never"
        meets = (rise is not None and rise <= RISE_TARGET and abs(err) <= band)
        n_tot += 1; n_meet += 1 if meets else 0
        print("%-4s %6.1f %6.1f  %8s  %8.2f  %+6.2f  %+6.1f   %s" %
              (label, b[0][0], cmd, rms, settled, err, errp, "YES" if meets else ""))
    prev_lc = b[-1][1]; prev_uc = b[-1][3]
print("\n%d/%d steps meet <=%dms AND <=%.0f%% (band floor %.2f m/s^2)" % (n_meet, n_tot, RISE_TARGET, TOL_PCT, BAND_FLOOR))
