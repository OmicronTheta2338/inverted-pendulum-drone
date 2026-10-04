#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -d "$HERE/ardupilot" ]]; then
    git clone --depth 1 --branch Copter-4.7.1 --recurse-submodules --shallow-submodules \
        https://github.com/ArduPilot/ardupilot.git "$HERE/ardupilot"
fi
[[ "$(git -C "$HERE/ardupilot" rev-parse HEAD)" == dbe792162d06cab66c3475fd5556bf7a120f119e ]]
python3 -m venv "$HERE/.venv"
"$HERE/.venv/bin/python" -m pip install -r "$HERE/requirements.txt"
if [[ ! -x "$HERE/toolchain/gcc-arm-none-eabi-10-2020-q4-major/bin/arm-none-eabi-g++" ]]; then
    mkdir -p "$HERE/toolchain"
    archive="$HERE/toolchain/compiler.tar.bz2"
    curl -L --fail --silent --show-error \
        https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2 -o "$archive"
    printf '%s  %s\n' 21134caa478bbf5352e239fbc6e2da3038f8d2207e089efc96c3b55f1edcd618 "$archive" | sha256sum -c -
    tar -xjf "$archive" -C "$HERE/toolchain"
fi
