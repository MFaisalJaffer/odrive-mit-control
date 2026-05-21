#!/usr/bin/env python3
"""
Live joint position monitor using MIT feedback frames.

Sends MIT commands with kp=0, kd=0, torque=0 so no force is applied.
Parses the MIT response frames (same CMD_MIT=0x008 arbitration ID) to
display live position, velocity, and torque for each joint.

Usage:
  python3 mit_monitor.py              # prompts for leg selection
  python3 mit_monitor.py --leg right
  python3 mit_monitor.py --leg left
  python3 mit_monitor.py --leg both

Press ENTER or Ctrl+C to stop and return all joints to IDLE.
"""

import argparse
import can
import math
import select
import struct
import sys
import time

GEAR_RATIO = 8.0
DT         = 0.02  # 50 Hz send rate

LEGS = {
    'right': {'can': 'can0', 'joints': {3: 'hip_pitch', 4: 'hip_roll', 5: 'hip_yaw', 6: 'knee', 7: 'ankle'}},
    'left':  {'can': 'can1', 'joints': {13: 'hip_pitch', 14: 'hip_roll', 15: 'hip_yaw', 16: 'knee', 17: 'ankle'}},
}

CMD_MIT       = 0x008
CMD_SET_STATE = 0x007

AXIS_STATE_IDLE        = 1
AXIS_STATE_CLOSED_LOOP = 8

# MIT command encoding ranges
MIT_P_MIN,  MIT_P_MAX  = -12.5, 12.5
MIT_V_MIN,  MIT_V_MAX  = -45.0, 45.0
MIT_KP_MIN, MIT_KP_MAX =   0.0, 500.0
MIT_KD_MIN, MIT_KD_MAX =   0.0, 5.0
MIT_T_MIN,  MIT_T_MAX  = -18.0, 18.0

# MIT feedback decoding ranges (differ from command ranges)
FB_P_MIN,  FB_P_MAX  = -12.5, 12.5
FB_V_MIN,  FB_V_MAX  = -65.0, 65.0
FB_T_MIN,  FB_T_MAX  = -50.0, 50.0


def can_id(node_id, cmd):
    return (node_id << 5) | cmd


def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    return int((x - x_min) / (x_max - x_min) * ((1 << bits) - 1))


def uint_to_float(x_int, x_min, x_max, bits):
    return x_int * (x_max - x_min) / ((1 << bits) - 1) + x_min


def wrap_to_pi(x):
    """Wrap angle to [-π, +π] so multi-turn drift doesn't show up in display."""
    return ((x + math.pi) % (2 * math.pi)) - math.pi


def send_mit_zero(bus, node_id):
    """Send MIT with zero kp/kd/torque — motor is free, feedback is returned."""
    p   = float_to_uint(0.0, MIT_P_MIN, MIT_P_MAX, 16)
    v   = float_to_uint(0.0, MIT_V_MIN, MIT_V_MAX, 12)
    kp_ = float_to_uint(0.0, MIT_KP_MIN, MIT_KP_MAX, 12)
    kd_ = float_to_uint(0.0, MIT_KD_MIN, MIT_KD_MAX, 12)
    t   = float_to_uint(0.0, MIT_T_MIN,  MIT_T_MAX,  12)
    data = bytes([
        (p >> 8) & 0xFF, p & 0xFF,
        (v >> 4) & 0xFF,
        ((v & 0xF) << 4) | ((kp_ >> 8) & 0xF),
        kp_ & 0xFF,
        (kd_ >> 4) & 0xFF,
        ((kd_ & 0xF) << 4) | ((t >> 8) & 0xF),
        t & 0xFF,
    ])
    bus.send(can.Message(arbitration_id=can_id(node_id, CMD_MIT),
                         data=data, is_extended_id=False))


def parse_mit_feedback(data):
    """
    Decode MIT feedback frame (8 bytes):
      Byte 0:       node_id echo
      Bytes 1-2:    position  16-bit  [-12.5, +12.5] rad
      Bytes 3, 4hi: velocity  12-bit  [-65,   +65]   rad/s
      Bytes 4lo, 5: torque    12-bit  [-50,   +50]   Nm
    Returns (pos_rad, vel_rads, torque_nm) or None if data too short.
    """
    if len(data) < 6:
        return None
    pos_raw  = (data[1] << 8) | data[2]
    vel_raw  = (data[3] << 4) | (data[4] >> 4)
    torq_raw = ((data[4] & 0xF) << 8) | data[5]
    pos_rad   = uint_to_float(pos_raw,  FB_P_MIN, FB_P_MAX, 16)
    vel_rads  = uint_to_float(vel_raw,  FB_V_MIN, FB_V_MAX, 12)
    torque_nm = uint_to_float(torq_raw, FB_T_MIN, FB_T_MAX, 12)
    return pos_rad, vel_rads, torque_nm


def set_state(bus, node_id, state):
    bus.send(can.Message(arbitration_id=can_id(node_id, CMD_SET_STATE),
                         data=struct.pack('<I', state), is_extended_id=False))


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
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--leg', choices=['right', 'left', 'both'], help='Leg selection')
    args = parser.parse_args()

    leg_name = args.leg or pick("Select leg:", ['right', 'left', 'both'])

    if leg_name == 'both':
        selected = list(LEGS.items())
    else:
        selected = [(leg_name, LEGS[leg_name])]

    # Open buses
    buses = {}
    for name, cfg in selected:
        buses[name] = can.interface.Bus(channel=cfg['can'], interface='socketcan')

    # Build lookup: arbitration_id → (leg_name, node_id)
    mit_ids = {}
    for name, cfg in selected:
        for nid in cfg['joints']:
            mit_ids[can_id(nid, CMD_MIT)] = (name, nid)

    print(f"\nMonitoring: {', '.join(n for n, _ in selected)}")
    print("Press ENTER to stop.\n")

    # Enter closed-loop
    for name, cfg in selected:
        bus = buses[name]
        for nid in cfg['joints']:
            set_state(bus, nid, AXIS_STATE_CLOSED_LOOP)
    time.sleep(0.5)

    # State: latest feedback per node
    latest = {}  # node_id → (pos, vel, torque)

    try:
        last_send = 0.0
        while True:
            # Check for ENTER
            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                break

            now = time.time()

            # Send zero-effort MIT to all nodes at DT rate
            if now - last_send >= DT:
                for name, cfg in selected:
                    bus = buses[name]
                    for nid in cfg['joints']:
                        send_mit_zero(bus, nid)
                last_send = now

            # Drain incoming frames from all buses
            for name, cfg in selected:
                bus = buses[name]
                r = bus.recv(timeout=0.005)
                if not r:
                    continue
                if r.arbitration_id in mit_ids:
                    _, nid = mit_ids[r.arbitration_id]
                    fb = parse_mit_feedback(bytes(r.data))
                    if fb:
                        latest[nid] = fb

            # Print live table
            if latest:
                lines = []
                for name, cfg in selected:
                    for nid in cfg['joints']:
                        label = cfg['joints'][nid]
                        if nid in latest:
                            pos, vel, torq = latest[nid]
                            pos = wrap_to_pi(pos)
                            lines.append(
                                f"  {label:12s} (node {nid:2d})  "
                                f"pos={pos:+7.4f} rad  "
                                f"vel={vel:+7.3f} rad/s  "
                                f"torq={torq:+6.3f} Nm"
                            )
                        else:
                            lines.append(f"  {label:12s} (node {nid:2d})  waiting...")
                # Move cursor up and overwrite
                output = '\n'.join(lines)
                print(f"\033[{len(lines)}A{output}", end='\n', flush=True)

    except KeyboardInterrupt:
        print("\n\nAborted.")

    finally:
        print("\nSetting all joints to IDLE...")
        for name, cfg in selected:
            bus = buses[name]
            for nid in cfg['joints']:
                set_state(bus, nid, AXIS_STATE_IDLE)
            bus.shutdown()
        print("Done.")


if __name__ == '__main__':
    main()
