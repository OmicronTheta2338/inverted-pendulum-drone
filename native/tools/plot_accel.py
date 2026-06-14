#!/usr/bin/env python3
"""Plot target (a_cmd) vs realised (a_real) lateral acceleration from a balance-run CSV.

The CSV is produced by the Godot harness when BALANCE_CSV is set:
    BALANCE_HEADLESS=1 BALANCE_CSV=run.csv godot --headless --path sim-v-1 res://balance_test.tscn
    python3 native/tools/plot_accel.py run.csv run.png [zoom_start_s] [zoom_span_s]

Top panel  = full run (overview).
Bottom panel = a short window (default 0..0.12 s) with the y-axis fit TIGHT to that
window and per-sample markers, so the tau_cl delay between command and realised is
visible tick-by-tick.
"""
import csv
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

COL_CMD = "#2f9fe0"
COL_REAL = "#e67f2a"


def main() -> None:
    src = sys.argv[1] if len(sys.argv) > 1 else "accel.csv"
    dst = sys.argv[2] if len(sys.argv) > 2 else "accel.png"
    z0 = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    zspan = float(sys.argv[4]) if len(sys.argv) > 4 else 0.12
    z1 = z0 + zspan

    t, cmd, real = [], [], []
    with open(src, newline="") as fh:
        for row in csv.DictReader(fh):
            t.append(float(row["t"]))
            cmd.append(float(row["a_cmd"]))
            real.append(float(row["a_real"]))

    fig, (ax_full, ax_zoom) = plt.subplots(2, 1, figsize=(10, 6.8))

    # --- overview ---
    ax_full.axhline(0.0, color="0.6", lw=0.8)
    ax_full.plot(t, cmd, color=COL_CMD, lw=1.1, label="a_cmd (target)")
    ax_full.plot(t, real, color=COL_REAL, lw=1.1, label="a_real (realised)")
    ax_full.set_xlim(t[0], t[-1])
    ax_full.set_ylabel("lateral accel [m/s²]")
    ax_full.set_title("Commanded vs realised lateral acceleration — full run")
    ax_full.legend(loc="upper right", framealpha=0.9)
    ax_full.grid(True, alpha=0.25)
    # mark the zoom window on the overview
    ax_full.axvspan(z0, z1, color="0.5", alpha=0.15)

    # --- zoom: tight y, per-sample markers ---
    win = [i for i in range(len(t)) if z0 <= t[i] <= z1]
    if len(win) >= 2:
        lo = min(min(cmd[i], real[i]) for i in win)
        hi = max(max(cmd[i], real[i]) for i in win)
        pad = max(0.05 * (hi - lo), 0.05)
        ax_zoom.set_ylim(lo - pad, hi + pad)
    ax_zoom.axhline(0.0, color="0.6", lw=0.8)
    ax_zoom.plot(t, cmd, color=COL_CMD, lw=1.2, marker=".", ms=4, label="a_cmd (target)")
    ax_zoom.plot(t, real, color=COL_REAL, lw=1.2, marker=".", ms=4, label="a_real (realised)")
    ax_zoom.set_xlim(z0, z1)
    ax_zoom.set_xlabel("time [s]")
    ax_zoom.set_ylabel("lateral accel [m/s²]")
    ax_zoom.set_title(f"Zoom {z0:g}–{z1:g} s — realised trails command by τ_cl (each dot = one 2.5 ms tick)")
    ax_zoom.grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(dst, dpi=120)
    print(f"wrote {dst}  ({len(t)} samples, zoom {z0:g}-{z1:g}s)")


if __name__ == "__main__":
    main()
