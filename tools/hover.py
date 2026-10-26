#!/usr/bin/env python3

# Interactive automatic flight in ALT mode: take off, hold the altitude, change it and land.
# Requires the range sensor (see docs/usage.md, "Range sensor").
# Connect the computer to the drone's Wi-Fi and run: python3 tools/hover.py
#
# Commands:
#   takeoff [alt]  take off and hold the altitude (m), default is CTL_ALT_TKOFF
#   alt <alt>      change the altitude in flight
#   land           land and disarm
#   status         show the current state
#   quit           land if flying and exit
#
# Safety: the altitude hold runs on the drone. If this program loses the connection
# for SF_LINK_TIME seconds, the drone lands by itself. Moving the throttle stick
# on the remote/virtual joystick takes the control back.

import math
import sys
import time
from threading import Thread
from pyflix import Flix


def fmt(value, unit='m'):
    return f'{value:.2f} {unit}' if math.isfinite(value) else '—'


def status(flix: Flix):
    return (f'mode {flix.mode}, {"ARMED" if flix.armed else "disarmed"}, '
            f'altitude {fmt(flix.altitude)}, target {fmt(flix.altitude_target)}, '
            f'vertical speed {fmt(flix.vertical_speed, "m/s")}, battery {fmt(flix.voltage, "V")}')


def report_reached(flix: Flix, target: float):
    try:
        flix.wait('altitude', value=lambda alt: abs(alt - target) < 0.05, timeout=15)
        print(f'\nHolding {target:.2f} m\n> ', end='', flush=True)
    except TimeoutError:
        print(f'\nDid not reach {target:.2f} m: {status(flix)}\n> ', end='', flush=True)


def report_landed(flix: Flix):
    try:
        flix.wait('armed', value=False, timeout=30)
        print('\nLanded and disarmed\n> ', end='', flush=True)
    except TimeoutError:
        print(f'\nStill armed: {status(flix)}\n> ', end='', flush=True)


DRONE_MESSAGES = ('Takeoff', 'Landing', 'Landed', 'Pilot took control', 'Connection lost', 'denied')


_last_message = ('', 0.0)


def show_drone_message(text: str):
    # show flight messages from the drone console (takeoff, landing, denial reasons)
    global _last_message
    for line in text.splitlines():
        if any(m in line for m in DRONE_MESSAGES) and not line.startswith('>'):
            if line == _last_message[0] and time.time() - _last_message[1] < 1:
                continue  # duplicate from a resent command
            _last_message = (line, time.time())
            print(f'\n[drone] {line}\n> ', end='', flush=True)


def command(flix: Flix, line: str) -> bool:
    words = line.split()
    if not words:
        return True
    cmd, args = words[0].lower(), words[1:]
    try:
        if cmd in ('takeoff', 'alt'):
            if cmd == 'alt' and not args:
                print('Usage: alt <altitude in m>')
                return True
            target = float(args[0]) if args else flix.get_param('CTL_ALT_TKOFF')
            limit = flix.get_param('RNG_MAX') - 0.1
            if target > limit:
                print(f'Limited to {limit:.2f} m by the range sensor (RNG_MAX)')
                target = limit
            if not math.isfinite(flix.altitude):
                print('No altitude data: check the range sensor (rng command in the console)')
                return True
            flix.takeoff(target)
            print(f'Going to {target:.2f} m')
            Thread(target=report_reached, args=(flix, target), daemon=True).start()
        elif cmd == 'land':
            flix.land()
            print('Landing')
            Thread(target=report_landed, args=(flix,), daemon=True).start()
        elif cmd == 'status':
            print(status(flix))
        elif cmd in ('quit', 'exit', 'q'):
            return False
        else:
            print('Commands: takeoff [alt], alt <alt>, land, status, quit')
    except ValueError:
        print('Invalid number')
    except RuntimeError as e:
        print(f'Denied ({e})')
    return True


def main():
    print('Connecting...')
    flix = Flix()
    try:
        flix.wait('altitude', value=lambda alt: math.isfinite(alt), timeout=2)
    except TimeoutError:
        print('Warning: no altitude data. Check RNG_MODEL and the range sensor (rng command).')
    print('Connected.', status(flix))
    flix.on('print_full', lambda text: show_drone_message(text))
    print('Commands: takeoff [alt], alt <alt>, land, status, quit')

    try:
        while True:
            line = input('> ')
            if not command(flix, line):
                break
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        if flix.armed:
            print('Landing before exit...')
            try:
                flix.land(wait=True)
                print('Landed')
            except Exception as e:
                print(f'Landing failed: {e}')
                sys.exit(1)


if __name__ == '__main__':
    main()
