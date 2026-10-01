#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PATH="$HERE/.venv/bin:$HERE/toolchain/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
cd "$HERE/ardupilot"
[[ "$(git rev-parse HEAD)" == dbe792162d06cab66c3475fd5556bf7a120f119e ]] || { echo "Unexpected ArduPilot revision" >&2; exit 1; }
mkdir -p ROMFS/scripts
cp "$HERE/scripts/base_drone.lua" ROMFS/scripts/base_drone.lua
# Apply the same control fix to board and SITL builds.
if ! git apply --reverse --check "$HERE/lift-motor-limits.patch" 2>/dev/null; then
    git apply --check "$HERE/lift-motor-limits.patch"
    git apply "$HERE/lift-motor-limits.patch"
fi
python "$HERE/tests/check_lift_limits.py"
extra=()
if [[ "${1:-StellarH7V2}" == StellarH7V2 ]]; then
    if ! git apply --reverse --check "$HERE/puya-nor.patch" 2>/dev/null; then
        git apply --check "$HERE/puya-nor.patch"
        git apply "$HERE/puya-nor.patch"
    fi
    extra+=(--extra-hwdef "$HERE/stellar-flash.hwdef")
fi
# Host GCC 16 reports upstream warnings absent with the pinned ARM GCC 10.
if [[ "${1:-StellarH7V2}" == sitl ]]; then
    extra+=(--disable-Werror --debug)
    export CXXFLAGS="${CXXFLAGS:-} -Wno-error=maybe-uninitialized"
fi
python waf configure --board "${1:-StellarH7V2}" --enable-scripting "${extra[@]}"
python waf copter -j4

if [[ "${1:-StellarH7V2}" == StellarH7V2 ]]; then
    python "$HERE/package.py"
fi
