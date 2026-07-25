"""Parse the Godot [grade] lines and report per-command accuracy vs the independent Godot truth.

The grader (main.gd _grade) prints, ~4x/sec:
    [grade] lat cmd=  3.00 true=  2.94 err=... | up cmd=  0.00 true= -0.01 err=...
'true' is the drone rigid body's own dv/dt (Godot physics truth), NOT the IMU signal the
controller closes on -- so this is an independent check (SPEC SS7).

Groups the lines into contiguous blocks of constant (lat_cmd, up_cmd) and, per block, reports the
settled value (mean of last 3 samples) and the worst-magnitude sample (overshoot). Flags a block
if the settled error on a commanded axis exceeds ~15% + 0.2.
Usage:  python3 analyze.py <godot.log>
"""
import sys, re
pat = re.compile(r"\[grade\] lat cmd=\s*(-?\d+\.\d+) true=\s*(-?\d+\.\d+).*up cmd=\s*(-?\d+\.\d+) true=\s*(-?\d+\.\d+)")
rows = []
for line in open(sys.argv[1]):
    mo = pat.search(line)
    if mo:
        lc, lt, uc, ut = map(float, mo.groups())
        rows.append((lc, lt, uc, ut))

blocks = []
for r in rows:
    key = (round(r[0], 2), round(r[2], 2))
    if not blocks or blocks[-1][0] != key:
        blocks.append([key, []])
    blocks[-1][1].append(r)

print("%-16s %8s %8s   %8s %8s   (set=settled mean, max=worst overshoot)" %
      ("cmd(lat,up)", "lat_set", "lat_max", "up_set", "up_max"))
for (lc, uc), samples in blocks:
    if len(samples) < 2: continue
    tail = samples[-3:]
    lat_end = sum(s[1] for s in tail) / len(tail)
    up_end = sum(s[3] for s in tail) / len(tail)
    lat_max = max((s[1] for s in samples), key=abs)
    up_max = max((s[3] for s in samples), key=abs)
    print("(%5.1f,%5.1f)  n=%2d  %8.2f %8.2f   %8.2f %8.2f" %
          (lc, uc, len(samples), lat_end, lat_max, up_end, up_max))
    if abs(lc) > 0.1 and abs(lat_end - lc) > 0.15 * abs(lc) + 0.2:
        print("      LAT steady err %.2f (cmd %.2f)" % (lat_end - lc, lc))
    if abs(uc) > 0.1 and abs(up_end - uc) > 0.15 * abs(uc) + 0.2:
        print("      UP  steady err %.2f (cmd %.2f)" % (up_end - uc, uc))
