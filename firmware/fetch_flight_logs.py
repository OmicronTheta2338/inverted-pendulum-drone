#!/usr/bin/env python3
"""Read onboard logs over MAVLink. Never arm, override controls or set parameters."""
import argparse
import json
import socket
from pathlib import Path
import time
from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='tcp:192.168.2.1:5760')
    parser.add_argument('--count', type=int, default=8)
    parser.add_argument('--list', action='store_true')
    args = parser.parse_args()
    destination = Path(__file__).resolve().parent/'logs'/'onboard'
    destination.mkdir(exist_ok=True)
    socket.setdefaulttimeout(8)
    link = mavutil.mavlink_connection(args.port, source_system=255, source_component=191, retries=1)
    def receive(kind=None, timeout=1):
        end = time.monotonic()+timeout
        while time.monotonic()<end:
            message = link.recv_match(blocking=True, timeout=min(.25, max(0,end-time.monotonic())))
            if message is None or message.get_srcComponent()!=1:
                continue
            if message.get_type()=='HEARTBEAT' and message.base_mode&128:
                raise RuntimeError('FC is armed; refusing log transfer')
            if kind is None or message.get_type()==kind:
                return message
        return None
    try:
        heartbeat = receive('HEARTBEAT', 10)
        if heartbeat is None:
            raise RuntimeError('No FC heartbeat')
        system = heartbeat.get_srcSystem()
        link.mav.log_request_list_send(system, 1, 0, 65535)
        entries = {}
        deadline = time.monotonic()+12
        while time.monotonic()<deadline:
            message = receive('LOG_ENTRY')
            if message is None: continue
            entries[message.id] = message.to_dict()
            if len(entries)>=message.num_logs: break
        if not entries: raise RuntimeError('No log index received')
        (destination/'index.json').write_text(json.dumps(entries, indent=2)+'\n')
        for identifier in sorted(entries):
            print(f'Log {identifier}: {entries[identifier]["size"]} bytes, UTC {entries[identifier]["time_utc"]}', flush=True)
        if args.list: return
        for identifier in reversed(sorted(entries)[-args.count:]):
            size = entries[identifier]['size']
            if not 0<size<=20_000_000:
                print(f'Skipping log {identifier}: size {size}', flush=True)
                continue
            path = destination/f'log-{identifier:05d}.bin'
            if path.exists() and path.stat().st_size==size:
                print('Already downloaded:', path.name, flush=True)
                continue
            buffer = bytearray(size)
            seen = bytearray(size)
            reported = 0
            print(f'Downloading log {identifier}: {size} bytes', flush=True)
            for start in range(0, size, 90*64):
                end = min(size,start+90*64)
                for attempt in range(5):
                    try: missing = seen.index(0,start,end)
                    except ValueError: break
                    link.mav.log_request_data_send(system,1,identifier,missing,end-missing)
                    deadline = time.monotonic()+4
                    while time.monotonic()<deadline:
                        message = receive('LOG_DATA', .5)
                        if message is None: continue
                        if message.id!=identifier or message.count==0: continue
                        offset=message.ofs
                        count=min(message.count, size-offset)
                        if offset<0 or count<=0:continue
                        buffer[offset:offset+count]=bytes(message.data[:count])
                        seen[offset:offset+count]=b'\1'*count
                        if all(seen[start:end]):break
                    if all(seen[start:end]):break
                else:
                    raise RuntimeError(f'Log {identifier}: missing data at {start}; transfer incomplete')
                if end-reported>=65536 or end==size:
                    print(f'Log {identifier}: {end}/{size}', flush=True)
                    reported=end
            assert all(seen), 'Incomplete log'
            temporary = path.with_suffix('.partial')
            temporary.write_bytes(buffer)
            temporary.replace(path)
            print('Saved', path, flush=True)
        link.mav.log_request_end_send(system,1)
    finally:
        link.close()


if __name__=='__main__':
    try:
        main()
    except (RuntimeError, OSError) as error:
        raise SystemExit(f'Log retrieval stopped: {error}. Check FC power, Wi-Fi connection and disarmed state.')
