#!/usr/bin/env python3
"""Run the production NOR identification function with a mocked SPI device.

Checks ID byte order and capacity selection; does not test physical SPI or writes.
"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'ardupilot/libraries/AP_Filesystem/AP_Filesystem_FlashMemory_LittleFS.cpp').read_text()
constants = source[source.index('#define JEDEC_WRITE_ENABLE'):source.index('bool AP_Filesystem_FlashMemory_LittleFS::is_busy')]
function = source[source.index('uint32_t AP_Filesystem_FlashMemory_LittleFS::find_block_size_and_count()'):source.index('bool AP_Filesystem_FlashMemory_LittleFS::mount_filesystem()')]
harness = r'''
#include <cstdint>
#include <cstdio>
#include <cassert>
#include <cstring>
#define CONFIG_HAL_BOARD 1
#define HAL_BOARD_SITL 2
#define AP_FILESYSTEM_LITTLEFS_FLASH_TYPE 1
#define AP_FILESYSTEM_FLASH_W25NXX 2
#define WITH_SEMAPHORE(x)
using lfs_size_t = uint32_t;
struct Device {
    uint8_t reply[4] = {0x85, 0x20, 0x18, 0x85};
    bool transfer(const uint8_t* command, unsigned n, uint8_t* out, unsigned count) {
        assert(n == 1 && command[0] == 0x9f && count == 4);
        memcpy(out, reply, count); return true;
    }
};
struct Scheduler { void delay(unsigned) {} } scheduler;
struct { Scheduler* scheduler; } hal { &scheduler };
struct AP_Filesystem_FlashMemory_LittleFS {
    Device device;
    Device* dev = &device;
    bool use_32bit_address = false;
    struct {
        uint32_t read_size, prog_size, block_size, block_count, metadata_max;
        float compact_thresh;
        uint32_t block_cycles, lookahead_size, cache_size;
    } fs_cfg {};
    bool wait_until_device_is_ready() { return true; }
    uint32_t find_block_size_and_count();
};
'''
checks = r'''
int main() {
    AP_Filesystem_FlashMemory_LittleFS fs;
    assert(fs.find_block_size_and_count() == 0x852018);
    assert(fs.fs_cfg.read_size == 256 && fs.fs_cfg.prog_size == 256);
    assert(fs.fs_cfg.block_size == 65536 && fs.fs_cfg.block_count == 256);
    assert(uint64_t(fs.fs_cfg.block_size) * fs.fs_cfg.block_count == 16777216);
    assert(!fs.use_32bit_address);
    fs.device.reply[0] = 0x20; fs.device.reply[1] = 0x18; fs.device.reply[2] = 0x85;
    assert(fs.find_block_size_and_count() == 0);
    fs.device.reply[0] = 0x00; fs.device.reply[1] = 0x00; fs.device.reply[2] = 0x00;
    assert(fs.find_block_size_and_count() == 0);
    puts("PASS: production NOR ID byte order, 16 MiB geometry, unsupported-ID rejection");
}
'''
with tempfile.TemporaryDirectory(prefix='flash-geometry-') as directory:
    cpp = Path(directory) / 'check.cpp'
    binary = Path(directory) / 'check'
    cpp.write_text(harness + constants + function + checks)
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
