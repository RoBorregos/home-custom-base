#!/usr/bin/env python3
"""
Capture WHY an ODrive S1 drops out of CLOSED_LOOP (LED green -> red).

Reads the STM32 telemetry over USB (stdlib only: no ROS, no pyserial) and prints
every change of an axis error / state with the ODriveError bits decoded, plus
the Iq / Ibus / Vbus of that instant. The firmware's auto-rearm clears the error
within ~500 ms, so watching the dashboard by eye usually misses it.

Usage:
    python3 odrive_config/capture_errors.py            # listen only
    python3 odrive_config/capture_errors.py --drive    # + forward/reverse cycle
        --vx 0.3 --half 1.5    m/s and seconds per direction
Safety: --drive moves the base. WHEELS OFF THE GROUND. Ctrl-C sends a zero.
"""
import argparse
import ast
import glob
import os
import re
import sys
import termios
import time

DASHBOARD = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                         'omnibase_ws', 'src', 'odrive_comm', 'odrive_comm',
                         'odrive_dashboard.py')

# ODriveError, fw 0.6.x (heartbeat Axis_Error = active_errors | disarm_reason)
ODRIVE_ERRORS = {
    0x00000001: 'INITIALIZING', 0x00000002: 'SYSTEM_LEVEL',
    0x00000004: 'TIMING_ERROR', 0x00000008: 'MISSING_ESTIMATE',
    0x00000010: 'BAD_CONFIG', 0x00000020: 'DRV_FAULT',
    0x00000040: 'MISSING_INPUT', 0x00000100: 'DC_BUS_OVER_VOLTAGE',
    0x00000200: 'DC_BUS_UNDER_VOLTAGE', 0x00000400: 'DC_BUS_OVER_CURRENT',
    0x00000800: 'DC_BUS_OVER_REGEN_CURRENT', 0x00001000: 'CURRENT_LIMIT_VIOLATION',
    0x00002000: 'MOTOR_OVER_TEMP', 0x00004000: 'INVERTER_OVER_TEMP',
    0x00008000: 'VELOCITY_LIMIT_VIOLATION', 0x00010000: 'POSITION_LIMIT_VIOLATION',
    0x01000000: 'WATCHDOG_TIMER_EXPIRED', 0x02000000: 'ESTOP_REQUESTED',
    0x04000000: 'SPINOUT_DETECTED', 0x08000000: 'BRAKE_RESISTOR_DISARMED',
    0x10000000: 'THERMISTOR_DISCONNECTED', 0x40000000: 'CALIBRATION_ERROR',
}


def decode(err: int) -> str:
    names = [n for b, n in ODRIVE_ERRORS.items() if err & b]
    unknown = err & ~sum(ODRIVE_ERRORS)
    if unknown:
        names.append(f'UNKNOWN(0x{unknown:x})')
    return '|'.join(names) or 'none'


def telem_fields():
    """TELEM_FIELDS from the dashboard, without importing it (it pulls rclpy)."""
    tree = ast.parse(open(DASHBOARD).read())
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') == 'TELEM_FIELDS':
            return ast.literal_eval(node.value)
    sys.exit('TELEM_FIELDS not found in odrive_dashboard.py')


def find_port():
    # STLINK-V3 VCP (0483:374E) — same as _find_stm_port in the dashboard
    for dev in sorted(glob.glob('/sys/class/tty/ttyACM*')):
        usb = os.path.realpath(os.path.join(dev, 'device', '..'))
        try:
            ids = (open(f'{usb}/idVendor').read().strip(), open(f'{usb}/idProduct').read().strip())
        except OSError:
            continue
        if ids == ('0483', '374e'):
            return '/dev/' + os.path.basename(dev)
    return '/dev/ttyACM0'


def open_serial(port):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = 0                                                # iflag: raw
    a[1] = 0                                                # oflag
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL     # 8N1
    a[3] = 0                                                # lflag: no echo/canon
    a[4] = a[5] = termios.B230400
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port')
    ap.add_argument('--drive', action='store_true', help='forward/reverse cycle (wheels lifted!)')
    ap.add_argument('--vx', type=float, default=0.3)
    ap.add_argument('--half', type=float, default=1.5)
    args = ap.parse_args()

    fields = telem_fields()
    port = args.port or find_port()
    fd = open_serial(port)
    print(f'{port} @ 230400  drive={args.drive}  (Ctrl-C to stop)')

    prev = {}            # (axis, 'E'/'S') -> last value
    buf = b''
    t0 = time.monotonic()
    last_tx = 0.0
    reversals = 0
    try:
        while True:
            now = time.monotonic()
            if args.drive and now - last_tx >= 0.1:            # 10 Hz, like the dashboard
                phase = int((now - t0) / args.half)
                vx = args.vx if phase % 2 == 0 else -args.vx
                os.write(fd, f'1 {vx:.4f} 0.0000 0.0000\r\n'.encode())
                reversals = phase
                last_tx = now
            try:
                buf += os.read(fd, 4096)
            except BlockingIOError:
                time.sleep(0.005)
                continue
            *lines, buf = buf.split(b'\n')
            for raw in lines:
                line = raw.decode(errors='ignore').strip()
                ts = f'{now - t0:8.2f}s rev={reversals:3d}'
                if line.startswith('E') and '=' in line and not line[1:2].isdigit():
                    print(f'{ts}  FW {line}')                   # E=code,axis,detail / ELOST=n
                    continue
                d = {}
                for k, v in re.findall(r'(\w+)=([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)', line):
                    if k.isdigit() and int(k) < len(fields):
                        d[fields[int(k)]] = v
                if not all(f'{k}{j}' in d for k in 'ES' for j in range(4)):
                    continue                                    # slim or truncated line
                for j in range(4):
                    err, st = int(d[f'E{j}'].split('.')[0]), int(float(d[f'S{j}']))
                    if prev.get((j, 'E')) == err and prev.get((j, 'S')) == st:
                        continue
                    first = (j, 'E') not in prev
                    prev[(j, 'E')], prev[(j, 'S')] = err, st
                    if first and err == 0 and st == 8:
                        continue
                    print(f'{ts}  axis{j} node={d[f"N{j}"]:>3} state={st} err=0x{err:08x} {decode(err)}'
                          f'  IqSet={float(d[f"IqSet{j}"]):6.2f} IqMeas={float(d[f"IqMeas{j}"]):6.2f}'
                          f' Ibus={float(d[f"Ibus{j}"]):6.2f} Vbus={float(d[f"Vbus{j}"]):5.2f}'
                          f' vel={float(d[f"V{j}"]):6.2f}')
    except KeyboardInterrupt:
        pass
    finally:
        if args.drive:
            for _ in range(3):
                os.write(fd, b'1 0.0000 0.0000 0.0000\r\n')
                time.sleep(0.05)
        os.close(fd)


if __name__ == '__main__':
    assert decode(0) == 'none'
    assert decode(0x04000800) == 'DC_BUS_OVER_REGEN_CURRENT|SPINOUT_DETECTED'
    assert decode(0x80000000) == 'UNKNOWN(0x80000000)'
    main()
