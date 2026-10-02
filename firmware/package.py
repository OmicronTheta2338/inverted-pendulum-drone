#!/usr/bin/env python3
"""Collect the verified board build and its exact inputs; run after build.sh."""
import base64
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zlib
from intelhex import IntelHex

root = Path(__file__).resolve().parent
source = root / 'ardupilot'
revision = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
assert revision == 'dbe792162d06cab66c3475fd5556bf7a120f119e'
patches = ['puya-nor.patch', 'lift-motor-limits.patch']
assert subprocess.check_output(['git', '-C', str(source), 'diff', 'HEAD']) == b''.join((root/p).read_bytes() for p in patches), 'Unexpected upstream source changes'
hwdef = (source / 'build/StellarH7V2/hwdef.h').read_text()
assert '#define HAL_OS_FATFS_IO 0' in hwdef, 'Unexpected SD filesystem'
assert '#define HAL_USE_FATFS FALSE' in hwdef, 'Unexpected SD driver'
assert '#define HAL_OS_LITTLEFS_IO TRUE' in hwdef, 'Missing onboard-flash filesystem'
assert '#define AP_FILESYSTEM_LITTLEFS_FLASH_TYPE AP_FILESYSTEM_FLASH_JEDEC_NOR' in hwdef, 'Wrong flash driver'
bin_dir = source / 'build/StellarH7V2/bin'
apj = json.loads((bin_dir / 'arducopter.apj').read_text())
image = zlib.decompress(base64.b64decode(apj['image']))
assert apj['board_id'] == 1503, 'Wrong FC board ID'
assert len(image) == apj['image_size']
hex_image = IntelHex(str(bin_dir / 'arducopter_with_bl.hex'))
assert bytes(hex_image.tobinarray(start=0x08020000, size=len(image))) == image
assert hex_image.minaddr() == 0x08000000, 'Missing bootloader'
assert image == (bin_dir / 'arducopter.bin').read_bytes()
assert b'BaseDrone 4.7.1 liftlimits1' in image, 'Missing lift-limit firmware identity'
assert (source / 'ROMFS/scripts/base_drone.lua').read_bytes() == (root / 'scripts/base_drone.lua').read_bytes()
out = root / 'dist/base-drone-StellarH7V2'
out.mkdir(parents=True, exist_ok=True)
# Remove the superseded generated SD overlay from earlier packages.
(out / 'stellar-sd.hwdef').unlink(missing_ok=True)
files = [bin_dir / name for name in ['arducopter.apj', 'arducopter.bin', 'arducopter_with_bl.hex']]
files += [root / name for name in ['base-drone.parm', 'scripts/base_drone.lua', 'README.md', 'VALIDATION.md', 'HARDWARE-MIXER.md', 'hardware-layout.json', 'requirements.txt']]
files += [root/'generate_hardware_mixer.py', root/'tests/check_hardware_mixer.py', root/'tests/check_flash_geometry.py', root/'stellar-flash.hwdef', root/'puya-nor.patch']
files += [root/'lift-motor-limits.patch', root/'tests/check_lift_limits.py', root/'USB-UPDATE.md']
if (root/'logs/boot-result.json').exists():
    files.append(root/'logs/boot-result.json')
for path in files:
    shutil.copy2(path, out / path.name)
manifest = {
    'target': 'StellarH7V2', 'ardupilot_tag': 'Copter-4.7.1', 'revision': revision,
    'board_id': apj['board_id'], 'image_size': len(image),
    'logging_storage': 'onboard Puya NOR LittleFS; stellar-flash.hwdef and puya-nor.patch',
    'control_mode': 'ALT_HOLD', 'hardware_layout': json.loads((root/'hardware-layout.json').read_text()), 'custom_accel_mode': False,
    'status': 'Built for props-off commissioning; hardware and flight validation pending',
    'control_patch': 'lift-motor-limits.patch: non-reversible collective lift lower bounds and existing RPY saturation feedback; lateral signed selection retained',
    'toolchain_archive_sha256': '21134caa478bbf5352e239fbc6e2da3038f8d2207e089efc96c3b55f1edcd618',
    'sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
    'submodules': subprocess.check_output(['git', '-C', str(source), 'submodule', 'status', '--recursive'], text=True),
}
(out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
print(out)
