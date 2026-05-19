#!/usr/bin/env python3
"""
Read encoder.config.index_offset and use_index_offset from a single joint.
Optionally enable use_index_offset and save to flash.

Usage:
  python3 read_offset.py
  python3 read_offset.py --leg right --joint hip_yaw
  python3 read_offset.py --leg right --joint hip_yaw --enable-offset
"""

import argparse
import can
import math
import struct
import time

GEAR_RATIO = 8.0

# Leg → CAN interface + node map
LEGS = {
    'right': {
        'can': 'can0',
        'joints': {
            'hip_pitch': 3,
            'hip_roll':  4,
            'hip_yaw':   5,
            'knee':      6,
            'ankle':     7,
        },
    },
    'left': {
        'can': 'can1',
        'joints': {
            'hip_pitch': 13,
            'hip_roll':  14,
            'hip_yaw':   15,
            'knee':      16,
            'ankle':     17,
        },
    },
}

EP_INDEX_OFFSET     = 362
EP_USE_INDEX_OFFSET = 363
EP_SAVE_CONFIG      = 478


def sdo_read_float(bus, node_id, ep, timeout=1.0):
    payload = struct.pack('<BHB4x', 0, ep, 0)
    bus.send(can.Message(arbitration_id=(node_id << 5) | 0x004,
                         data=payload, is_extended_id=False))
    txsdo_id = (node_id << 5) | 0x005
    deadline = time.time() + timeout
    while time.time() < deadline:
        r = bus.recv(timeout=0.1)
        if r and r.arbitration_id == txsdo_id and len(r.data) >= 8:
            return struct.unpack_from('<f', bytes(r.data), 4)[0]
    return None


def sdo_write_bool(bus, node_id, ep, value):
    payload = struct.pack('<BHBB3x', 1, ep, 0, int(value))
    bus.send(can.Message(arbitration_id=(node_id << 5) | 0x004,
                         data=payload, is_extended_id=False))
    txsdo_id = (node_id << 5) | 0x005
    deadline = time.time() + 0.5
    while time.time() < deadline:
        r = bus.recv(timeout=0.1)
        if r and r.arbitration_id == txsdo_id:
            return True
    return False


def sdo_read_bool(bus, node_id, ep, timeout=1.0):
    payload = struct.pack('<BHB4x', 0, ep, 0)
    bus.send(can.Message(arbitration_id=(node_id << 5) | 0x004,
                         data=payload, is_extended_id=False))
    txsdo_id = (node_id << 5) | 0x005
    deadline = time.time() + timeout
    while time.time() < deadline:
        r = bus.recv(timeout=0.1)
        if r and r.arbitration_id == txsdo_id and len(r.data) >= 5:
            return bool(r.data[4])
    return None


def read_position(bus, node_id, timeout=2.0):
    """Enter closed-loop briefly to get encoder broadcast, return output-shaft rad."""
    enc_id = (node_id << 5) | 0x009
    bus.send(can.Message(arbitration_id=(node_id << 5) | 0x007,
                         data=struct.pack('<I', 8), is_extended_id=False))
    time.sleep(0.5)
    pos_rad = None
    deadline = time.time() + timeout
    while time.time() < deadline:
        r = bus.recv(timeout=0.05)
        if r and r.arbitration_id == enc_id and len(r.data) >= 4:
            pos_rev = struct.unpack_from('<f', bytes(r.data), 0)[0]
            pos_rad = pos_rev * 2 * math.pi / GEAR_RATIO
            break
    bus.send(can.Message(arbitration_id=(node_id << 5) | 0x007,
                         data=struct.pack('<I', 1), is_extended_id=False))
    return pos_rad


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
        print("  Invalid — enter a number from the list.")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--leg',   choices=['right', 'left'], help='Leg selection')
    parser.add_argument('--joint', help='Joint name (hip_pitch, hip_roll, hip_yaw, knee, ankle)')
    parser.add_argument('--enable-offset', action='store_true',
                        help='Write use_index_offset=True and save to flash')
    args = parser.parse_args()

    leg_name = args.leg or pick("Select leg:", ['right', 'left'])
    leg = LEGS[leg_name]

    joint_names = list(leg['joints'].keys())
    joint_name = args.joint if args.joint in leg['joints'] else pick("Select joint:", joint_names)
    node_id = leg['joints'][joint_name]
    can_iface = leg['can']

    print(f"\n{'─'*50}")
    print(f"  Leg   : {leg_name}")
    print(f"  Joint : {joint_name}  (node {node_id}  on  {can_iface})")
    print(f"{'─'*50}\n")

    bus = can.interface.Bus(channel=can_iface, interface='socketcan')
    try:
        # Read SDO endpoints
        use_offset = sdo_read_bool(bus, node_id, EP_USE_INDEX_OFFSET)
        index_offset = sdo_read_float(bus, node_id, EP_INDEX_OFFSET)

        if use_offset is None:
            print("  use_index_offset : no SDO reply (GIM firmware may not ack reads)")
        else:
            print(f"  use_index_offset : {use_offset}")

        if index_offset is None:
            print("  index_offset     : no SDO reply")
        else:
            offset_rad = index_offset * 2 * math.pi / GEAR_RATIO
            print(f"  index_offset     : {index_offset:.6f} rotor turns  ({offset_rad:.6f} rad output shaft)")

        # Optionally enable use_index_offset and save
        if args.enable_offset:
            print(f"\n  Writing use_index_offset = True (ep {EP_USE_INDEX_OFFSET})...")
            acked = sdo_write_bool(bus, node_id, EP_USE_INDEX_OFFSET, True)
            print(f"  {'Acked' if acked else 'No ack (expected for GIM firmware) — sent anyway'}")
            print("  Saving configuration to flash...")
            payload = struct.pack('<BHB4x', 1, EP_SAVE_CONFIG, 0)
            bus.send(can.Message(arbitration_id=(node_id << 5) | 0x004,
                                 data=payload, is_extended_id=False))
            time.sleep(0.5)
            print("  Rebooting motor to apply...")
            bus.send(can.Message(arbitration_id=(node_id << 5) | 0x016,
                                 data=bytes([0]), is_extended_id=False))
            time.sleep(4.0)
            print("  Done.")
            print("\n  Reading position after reboot...")
            pos_rad = read_position(bus, node_id)
            if pos_rad is not None:
                print(f"  Current position : {pos_rad:+.6f} rad (output shaft)")
            else:
                print("  Current position : no encoder data received")

        # Read live position
        if not args.enable_offset:
            print(f"\n  Reading current position (closed-loop flash)...")
            pos_rad = read_position(bus, node_id)
            if pos_rad is not None:
                print(f"  Current position : {pos_rad:+.6f} rad (output shaft)")
            else:
                print("  Current position : no encoder data received")

    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()
