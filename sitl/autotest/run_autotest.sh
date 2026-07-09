#!/bin/bash
# Headless auto-test for ModeAccel (mode 30).
#
# Launches Godot (headless) + ArduCopter SITL against the Godot JSON physics, arms + sets mode
# ACCEL over MAVLink, runs a scripted acceleration profile (defined in sim-v-1/main.gd -> auto_profile),
# and grades realised base-accel vs command against independent Godot truth. No human / no keyboard.
#
# Usage:
#   ./run_autotest.sh [RUN_SECONDS]          # default 60
#   AP_BIN=/path/to/arducopter ./run_autotest.sh
#
# IMPORTANT: rebuild the firmware first if you changed the controller:
#   ( cd ../../../ardupilot && ./waf copter )
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"                       # project root (…/inverted-pendulum-drone)
SIM="$ROOT/sim-v-1"
SITL="$ROOT/sitl"
AP_BIN="${AP_BIN:-$ROOT/../ardupilot/build/sitl/bin/arducopter}"
OUT="$HERE/out"; mkdir -p "$OUT"
GLOG="$OUT/godot.log"; SLOG="$OUT/sitl.log"; DLOG="$OUT/drive.log"
RUN="${1:-60}"

command -v godot >/dev/null 2>&1 || { echo "ERROR: 'godot' not on PATH"; exit 1; }
[ -x "$AP_BIN" ] || { echo "ERROR: arducopter not at $AP_BIN  (build: cd ardupilot && ./waf copter, or set AP_BIN=)"; exit 1; }
python3 -c "import pymavlink" 2>/dev/null || { echo "ERROR: pymavlink missing (pip install --user pymavlink)"; exit 1; }

cleanup(){ kill "${GPID:-}" "${APID:-}" 2>/dev/null; pkill -f 'godot --headless' 2>/dev/null; pkill -f "$AP_BIN" 2>/dev/null; }
cleanup; sleep 1
trap cleanup EXIT

echo "[autotest] Godot headless (AUTOTEST=1) ..."
( cd "$SIM" && exec env AUTOTEST=1 godot --headless --path . ) >"$GLOG" 2>&1 & GPID=$!
sleep 5
echo "[autotest] arducopter SITL (JSON physics @9002, MAVLink tcp:5760) ..."
( cd "$SITL" && exec "$AP_BIN" --model JSON:127.0.0.1 --defaults mav.parm --home 40.07,-105.23,1600,0 -I0 ) >"$SLOG" 2>&1 & APID=$!
sleep 3

echo "[autotest] MAVLink: arm + mode ACCEL + monitor (${RUN}s) ..."
python3 "$HERE/drive.py" "$RUN" | tee "$DLOG"

sleep 1; cleanup
echo; echo "===== auto markers (Godot) ====="; grep '\[auto\]' "$GLOG" || echo "(none -- takeoff may not have completed; increase RUN_SECONDS)"
echo; echo "===== per-command accuracy (Godot truth vs command) ====="
python3 "$HERE/analyze.py" "$GLOG"
echo; echo "[autotest] logs: $GLOG | $SLOG | $DLOG    ArduPilot .BIN in: $SITL/logs/"
