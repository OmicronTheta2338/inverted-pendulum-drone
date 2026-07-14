#!/usr/bin/env python3
"""Plot a COUPLED two-arm run: both arm angles + the single input's target/realised accel.

CSV produced by the Godot coupled 2-arm harness when BALANCE2_CSV is set:
    BALANCE2_HEADLESS=1 BALANCE2_CSV=run2.csv godot --headless --path sim-v-1 res://balance2_test.tscn
    python3 native/tools/plot_accel2.py run2.csv run2.png

Columns: t,a_cmd,a_real,theta1_deg,theta2_deg  (ONE input drives BOTH arms).
Panel 1 = both arm angles (the balance goal). Panel 2 = the single a_cmd vs a_real.
"""
import csv
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

COL_CMD = "#2f9fe0"
COL_REAL = "#e67f2a"
COL_A1 = "#3a86ff"
COL_A2 = "#e5383b"


def col(rows, key):
    return [float(r[key]) for r in rows]


def main() -> None:
    src = sys.argv[1] if len(sys.argv) > 1 else "run2.csv"
    dst = sys.argv[2] if len(sys.argv) > 2 else "run2.png"

    with open(src, newline="") as fh:
        rows = list(csv.DictReader(fh))
    t = col(rows, "t")
    th1, th2 = col(rows, "theta1_deg"), col(rows, "theta2_deg")
    c, r = col(rows, "a_cmd"), col(rows, "a_real")

    fig, (ax_ang, ax_u) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)

    ax_ang.axhline(0.0, color="0.6", lw=0.8)
    ax_ang.plot(t, th1, color=COL_A1, lw=1.1, label="link1 θ (0.7 m)")
    ax_ang.plot(t, th2, color=COL_A2, lw=1.1, label="link2 θ (0.6 m)")
    ax_ang.set_ylabel("link angle [deg]")
    ax_ang.set_title("Double inverted pendulum on cart (ONE input) — link angles")
    ax_ang.legend(loc="upper right", framealpha=0.9)
    ax_ang.grid(True, alpha=0.25)

    ax_u.axhline(0.0, color="0.6", lw=0.8)
    ax_u.plot(t, c, color=COL_CMD, lw=1.0, label="a_cmd (target)")
    ax_u.plot(t, r, color=COL_REAL, lw=1.0, label="a_real (realised)")
    ax_u.set_ylabel("single input a_x [m/s²]")
    ax_u.set_xlabel("time [s]")
    ax_u.set_xlim(t[0], t[-1])
    ax_u.set_title("Single lateral acceleration command that holds both links")
    ax_u.legend(loc="upper right", framealpha=0.9)
    ax_u.grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(dst, dpi=120)
    print(f"wrote {dst}  ({len(t)} samples)")


if __name__ == "__main__":
    main()
