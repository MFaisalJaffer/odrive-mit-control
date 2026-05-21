#!/usr/bin/env python3
"""
Read actuator temperatures by listening for ODrive Get_Temperature
broadcast frames (cmd_id = 0x015).

Each frame is 8 bytes:
  Bytes 0-3: FET temperature   (float32, °C) — power stage / inverter
  Bytes 4-7: motor temperature (float32, °C) — winding thermistor

Usage:
  python3 read_temps.py                 # snapshot both legs, exit
  python3 read_temps.py --leg right     # snapshot one leg, exit
  python3 read_temps.py --leg both --watch     # live update until Ctrl+C
  python3 read_temps.py --leg right --node 5    # single joint

This script is read-only: no MIT commands, no state changes, nothing
written to the bus. Safe to run while other things are active.
"""

import argparse
import can
import math
import select
import struct
import sys
import time

LEGS = {
    'right': {'can': 'can0', 'joints': {3: 'hip_pitch', 4: 'hip_roll',
                                          5: 'hip_yaw',   6: 'knee', 7: 'ankle'}},
    'left':  {'can': 'can1', 'joints': {13: 'hip_pitch', 14: 'hip_roll',
                                          15: 'hip_yaw',  16: 'knee', 17: 'ankle'}},
}

CMD_GET_TEMP = 0x015


def can_id(node_id, cmd):
    return (node_id << 5) | cmd


def collect(bus, node_ids, duration=2.0, debug=False):
    """Request and collect temperatures. Returns {nid: (fet_C, motor_C)}.

    First tries to listen passively (in case the firmware broadcasts).
    Then sends an RTR (remote transmission request) per node to explicitly
    ask for a Get_Temperature reply, and listens for the responses."""
    temp_ids = {can_id(nid, CMD_GET_TEMP): nid for nid in node_ids}
    temps = {}

    def drain(window):
        deadline = time.time() + window
        while time.time() < deadline:
            r = bus.recv(timeout=0.05)
            if r and r.arbitration_id in temp_ids and len(r.data) >= 8:
                nid = temp_ids[r.arbitration_id]
                if debug:
                    print(f"    node {nid}: raw 8 bytes = {bytes(r.data).hex()}")
                fet   = struct.unpack_from('<f', bytes(r.data), 0)[0]
                motor = struct.unpack_from('<f', bytes(r.data), 4)[0]
                temps[nid] = (fet, motor)
                if len(temps) == len(node_ids):
                    return True
        return False

    # 1) Passive listen briefly (works if broadcast is enabled)
    if drain(0.3):
        return temps

    # 2) RTR request per missing node
    for nid in node_ids:
        if nid in temps:
            continue
        msg = can.Message(arbitration_id=can_id(nid, CMD_GET_TEMP),
                          is_remote_frame=True, dlc=8,
                          is_extended_id=False)
        try:
            bus.send(msg)
        except can.CanError:
            pass

    # 3) Listen for the responses
    drain(duration)
    return temps


def colorize(temp_c):
    """Pick a status string based on temperature (rough ODrive thresholds)."""
    if math.isnan(temp_c):
        return "?"
    if temp_c < 40:   return "cool"
    if temp_c < 60:   return "ok"
    if temp_c < 80:   return "warm"
    if temp_c < 100:  return "HOT"
    return "DANGER"


def print_table(legs_data):
    """legs_data: [(leg_name, can_iface, joints_dict, temps_dict)]"""
    print(f"  {'leg':<6} {'joint':<11} {'node':>5}  {'FET °C':>10}  {'motor °C':>10}   status")
    print('  ' + '-' * 60)
    for leg_name, can_iface, joints_dict, temps in legs_data:
        for nid, jname in joints_dict.items():
            fet, motor = temps.get(nid, (float('nan'), float('nan')))
            hot = max(fet, motor) if not (math.isnan(fet) or math.isnan(motor)) else float('nan')
            status = colorize(hot)
            fet_s   = f"{fet:>+10.2f}"   if not math.isnan(fet)   else f"{'n/a':>10}"
            motor_s = f"{motor:>+10.2f}" if not math.isnan(motor) else f"{'n/a':>10}"
            print(f"  {leg_name:<6} {jname:<11} {nid:>5}  {fet_s}  {motor_s}   {status}")


def pick(prompt, options):
    print(f"\n{prompt}")
    for i, opt in enumerate(options, 1):
        print(f"  {i}) {opt}")
    while True:
        try:
            choice = int(input("  Choice: ").strip())
            if 1 <= choice <= len(options):
                return options[choice - 1]
        except (ValueError, KeyboardInterrupt):
            pass
        print("  Invalid.")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--leg',  choices=['right', 'left', 'both'], default='both',
                   help='Which leg(s) to read (default: both)')
    p.add_argument('--node', type=int, default=None,
                   help='Read only this node ID (within the selected leg)')
    p.add_argument('--watch', action='store_true',
                   help='Live-update every 1 second until ENTER or Ctrl+C')
    p.add_argument('--interval', type=float, default=1.0,
                   help='Watch refresh interval in seconds (default 1.0)')
    p.add_argument('--debug', action='store_true',
                   help='Print raw 8-byte response per node (to verify the '
                        'firmware is populating the fields)')
    args = p.parse_args()

    if args.leg == 'both':
        selected = list(LEGS.items())
    else:
        selected = [(args.leg, LEGS[args.leg])]

    # Build per-leg joint subsets (honoring --node)
    leg_specs = []
    for name, cfg in selected:
        joints = dict(cfg['joints'])
        if args.node is not None:
            if args.node not in joints:
                print(f"ERROR: node {args.node} not in {name} leg "
                      f"(valid: {sorted(joints)})")
                sys.exit(1)
            joints = {args.node: joints[args.node]}
        leg_specs.append((name, cfg['can'], joints))

    # Open one bus per leg
    buses = {name: can.interface.Bus(channel=iface, interface='socketcan')
             for name, iface, _ in leg_specs}

    def sample_once():
        legs_data = []
        for name, iface, joints in leg_specs:
            bus = buses[name]
            temps = collect(bus, list(joints.keys()), duration=2.0, debug=args.debug)
            legs_data.append((name, iface, joints, temps))
        return legs_data

    try:
        if not args.watch:
            print(f"\nSampling temperatures (up to 2s per leg)...\n")
            legs_data = sample_once()
            print_table(legs_data)
        else:
            print(f"\nLive temperatures every {args.interval:.1f}s — press ENTER or Ctrl+C to stop\n")
            while True:
                legs_data = sample_once()
                # Clear and reprint
                print("\033[2J\033[H", end='')  # clear screen, cursor home
                print(f"  [{time.strftime('%H:%M:%S')}]  (ENTER or Ctrl+C to stop)")
                print()
                print_table(legs_data)
                # Wait up to --interval, breaking early on ENTER
                t_end = time.time() + args.interval
                while time.time() < t_end:
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        sys.stdin.readline()
                        return
    except KeyboardInterrupt:
        print()
    finally:
        for bus in buses.values():
            bus.shutdown()


if __name__ == '__main__':
    main()
