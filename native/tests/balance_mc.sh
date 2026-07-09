#!/usr/bin/env bash
# Monte-Carlo / latency-sweep driver for the Godot balance scene.
# Runs many headless seeds (in parallel) and tallies PASS/FAIL on the 15 s goal.
#
# Usage:
#   balance_mc.sh seeds  <N> <LATENCY_MS> [JOBS]     # N seeds at one latency
#   balance_mc.sh sweep  <N> [JOBS]                  # N seeds at several latencies
#
# Env passthrough: BALANCE_USE_PID, BALANCE_NO_NOISE, BALANCE_INIT_DEG, BALANCE_BUMP_DV
set -u
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
PROJ="$REPO/sim-v-1"
OUT="$(mktemp -d)"
GODOT="${GODOT:-godot}"

run_one() { # seed latency
  local seed="$1" lat="$2" out="$OUT/r_${lat}_${seed}.txt" tries=0
  # Retry on empty output: parallel headless Godot instances occasionally race
  # on the shared .godot import cache and start-up fails, producing no RESULT.
  while [ "$tries" -lt 3 ]; do
    BALANCE_HEADLESS=1 BALANCE_SEED="$seed" BALANCE_LATENCY_MS="$lat" \
      "$GODOT" --headless --path "$PROJ" res://balance_test.tscn 2>/dev/null \
      | grep '^RESULT' > "$out"
    [ -s "$out" ] && return 0
    tries=$((tries+1)); sleep 1
  done
}
export -f run_one
export OUT GODOT PROJ

tally() { # latency
  local lat="$1" pass=0 total=0 worst_max=0 worst_final=0
  for f in "$OUT"/r_${lat}_*.txt; do
    [ -f "$f" ] || continue
    total=$((total+1))
    grep -q 'PASS' "$f" && pass=$((pass+1))
    local mx fn
    mx=$(sed -n 's/.*max_angle=\([0-9.]*\).*/\1/p' "$f")
    fn=$(sed -n 's/.*final=\([0-9.]*\).*/\1/p' "$f")
    [ -n "$mx" ] && awk "BEGIN{exit !($mx>$worst_max)}" && worst_max="$mx"
    [ -n "$fn" ] && awk "BEGIN{exit !($fn>$worst_final)}" && worst_final="$fn"
  done
  printf "  latency %5.1f ms : %2d/%2d pass   worst max|theta|=%s deg   worst final=%s deg\n" \
         "$lat" "$pass" "$total" "$worst_max" "$worst_final"
}

mode="${1:-seeds}"; N="${2:-20}"
case "$mode" in
  seeds)
    # JOBS default 3: higher parallelism makes concurrent headless Godot instances
    # race on the shared .godot import cache (some start-ups fail -> retried).
    LAT="${3:-7.5}"; JOBS="${4:-3}"
    echo "=== Monte-Carlo: $N seeds @ ${LAT}ms latency (jobs=$JOBS) ==="
    seq 1 "$N" | xargs -P "$JOBS" -I{} bash -c 'run_one "$@"' _ {} "$LAT"
    tally "$LAT"
    ;;
  sweep)
    JOBS="${3:-6}"
    echo "=== Latency sweep: $N seeds each (jobs=$JOBS) ==="
    for LAT in 0 5 7.5 10 15 20 30 40; do
      seq 1 "$N" | xargs -P "$JOBS" -I{} bash -c 'run_one "$@"' _ {} "$LAT"
    done
    for LAT in 0 5 7.5 10 15 20 30 40; do tally "$LAT"; done
    ;;
  *)
    echo "unknown mode: $mode"; exit 1;;
esac
echo "results in $OUT"
