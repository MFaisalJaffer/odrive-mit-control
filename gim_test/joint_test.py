#!/usr/bin/env python3
"""
Single-joint motion test — pose capture then smooth playback for one joint.

Same workflow as leg_test.py but operates on exactly one selected joint
on either leg. Only that joint enters CLOSED_LOOP; all others stay
untouched (they will remain in whatever state they were in — IDLE
unless something else is driving them).

Workflow:
  1. Enter CLOSED_LOOP in passive (kp=0) on the chosen joint.
  2. Read current position and check it against URDF limits.
  3. User poses the joint to the desired TARGET → ENTER → recorded.
  4. User moves the joint to its HANGING (or "rest") position → ENTER → recorded.
  5. Three smooth 3s moves at the chosen kp:
        Hanging → Zero → Target → Zero
  6. Joint returns to IDLE.

Usage:
  python3 joint_test.py --leg right --joint knee --urdf .../robot.urdf
  python3 joint_test.py --leg left  --joint hip_pitch --urdf .../robot.urdf --kp 150
  python3 joint_test.py --leg right --joint hip_yaw --urdf .../robot.urdf --csv /tmp/run.csv

  Joints: hip_pitch, hip_roll, hip_yaw, knee, ankle
"""

import argparse
import can
import csv
import math
import os
import select
import struct
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime

# ── Motion params ─────────────────────────────────────────────────────────────
GEAR_RATIO     = 8.0
MOVE_TIME      = 3.0
DWELL_TIME     = 1.0
DT             = 0.01
DEFAULT_KP     = 100.0
DEFAULT_KD     = 2.0
PASSIVE_KP     = 0.0
PASSIVE_KD     = 0.0
SAFETY_MARGIN  = 0.05

# ── Leg layout (mirrors leg_test.py / both_legs_test.py) ──────────────────────
LEGS = {
    'right': {
        'can': 'can0',
        'joints': {
            'hip_pitch': (3,  'dof_right_hip_pitch_04'),
            'hip_roll':  (4,  'dof_right_hip_roll_03'),
            'hip_yaw':   (5,  'dof_right_hip_yaw_03'),
            'knee':      (6,  'dof_right_knee_04'),
            'ankle':     (7,  'dof_right_ankle_02'),
        },
    },
    'left': {
        'can': 'can1',
        'joints': {
            'hip_pitch': (13, 'dof_left_hip_pitch_04'),
            'hip_roll':  (14, 'dof_left_hip_roll_03'),
            'hip_yaw':   (15, 'dof_left_hip_yaw_03'),
            'knee':      (16, 'dof_left_knee_04'),
            'ankle':     (17, 'dof_left_ankle_02'),
        },
    },
}

# ── ODrive CAN Simple constants ───────────────────────────────────────────────
CMD_ENC_EST   = 0x009
CMD_SET_STATE = 0x007
CMD_MIT       = 0x008

AXIS_STATE_IDLE        = 1
AXIS_STATE_CLOSED_LOOP = 8

MIT_P_MIN,  MIT_P_MAX  = -12.5, 12.5
MIT_V_MIN,  MIT_V_MAX  = -45.0, 45.0
MIT_KP_MIN, MIT_KP_MAX =   0.0, 500.0
MIT_KD_MIN, MIT_KD_MAX =   0.0, 5.0
MIT_T_MIN,  MIT_T_MAX  = -18.0, 18.0

FB_P_MIN, FB_P_MAX = -12.5, 12.5
FB_V_MIN, FB_V_MAX = -65.0, 65.0
FB_T_MIN, FB_T_MAX = -50.0, 50.0


# ── Helpers ───────────────────────────────────────────────────────────────────
def can_id(node_id, cmd):
    return (node_id << 5) | cmd


def send_raw(bus, node_id, cmd, data):
    bus.send(can.Message(arbitration_id=can_id(node_id, cmd),
                         data=data, is_extended_id=False))


def set_state(bus, node_id, state):
    send_raw(bus, node_id, CMD_SET_STATE, struct.pack('<I', state))


def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    return int((x - x_min) / (x_max - x_min) * ((1 << bits) - 1))


def uint_to_float(x_int, x_min, x_max, bits):
    return x_int * (x_max - x_min) / ((1 << bits) - 1) + x_min


def parse_mit_feedback(data):
    if len(data) < 6:
        return None
    pos_raw  = (data[1] << 8) | data[2]
    vel_raw  = (data[3] << 4) | (data[4] >> 4)
    torq_raw = ((data[4] & 0xF) << 8) | data[5]
    return (uint_to_float(pos_raw,  FB_P_MIN, FB_P_MAX, 16),
            uint_to_float(vel_raw,  FB_V_MIN, FB_V_MAX, 12),
            uint_to_float(torq_raw, FB_T_MIN, FB_T_MAX, 12))


def send_mit(bus, node_id, pos_rad, vel=0.0, kp=DEFAULT_KP, kd=DEFAULT_KD, torque=0.0):
    p   = float_to_uint(pos_rad, MIT_P_MIN, MIT_P_MAX, 16)
    v   = float_to_uint(vel,     MIT_V_MIN, MIT_V_MAX, 12)
    kp_ = float_to_uint(kp,      MIT_KP_MIN, MIT_KP_MAX, 12)
    kd_ = float_to_uint(kd,      MIT_KD_MIN, MIT_KD_MAX, 12)
    t   = float_to_uint(torque,  MIT_T_MIN,  MIT_T_MAX,  12)
    data = bytes([
        (p >> 8) & 0xFF, p & 0xFF,
        (v >> 4) & 0xFF,
        ((v & 0xF) << 4) | ((kp_ >> 8) & 0xF),
        kp_ & 0xFF,
        (kd_ >> 4) & 0xFF,
        ((kd_ & 0xF) << 4) | ((t >> 8) & 0xF),
        t & 0xFF,
    ])
    send_raw(bus, node_id, CMD_MIT, data)


def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def wrap_to_pi(x):
    if x is None:
        return None
    return ((x + math.pi) % (2 * math.pi)) - math.pi


def parse_urdf_limits(urdf_path, joint_name):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    for j in root.findall('joint'):
        if j.get('name') == joint_name:
            lim = j.find('limit')
            if lim is None:
                raise ValueError(f"Joint '{joint_name}' has no <limit> in URDF")
            return (float(lim.get('lower', '0')), float(lim.get('upper', '0')))
    raise ValueError(f"Joint '{joint_name}' not found in URDF")


def read_position(bus, node_id, duration=0.5, wrap=True):
    """Listen for an encoder broadcast and return the latest position."""
    enc_id = can_id(node_id, CMD_ENC_EST)
    pos = None
    deadline = time.time() + duration
    while time.time() < deadline:
        r = bus.recv(timeout=0.02)
        if r and r.arbitration_id == enc_id and len(r.data) >= 4:
            pos_rev = struct.unpack_from('<f', bytes(r.data), 0)[0]
            pos = pos_rev * 2 * math.pi / GEAR_RATIO
    if pos is None:
        return None
    return wrap_to_pi(pos) if wrap else pos


def passive_burst(bus, node_id, frames=20):
    for _ in range(frames):
        send_mit(bus, node_id, 0.0, kp=PASSIVE_KP, kd=PASSIVE_KD)
        time.sleep(DT)


def capture_pose(bus, node_id, joint_name, lo, hi, prompt):
    """Show the live position of one joint; ENTER returns the latest wrapped value."""
    enc_id  = can_id(node_id, CMD_ENC_EST)
    live    = None
    last_mit, last_print = 0.0, 0.0
    print(f"  (move the joint by hand; press ENTER to {prompt})")
    print()
    while True:
        now = time.time()
        if now - last_mit > 0.02:
            send_mit(bus, node_id, 0.0, kp=PASSIVE_KP, kd=PASSIVE_KD)
            last_mit = now
        r = bus.recv(timeout=0.02)
        if r and r.arbitration_id == enc_id and len(r.data) >= 4:
            pos_rev = struct.unpack_from('<f', bytes(r.data), 0)[0]
            live = wrap_to_pi(pos_rev * 2 * math.pi / GEAR_RATIO)
        if live is not None and (now - last_print > 0.05):
            in_range = (lo - SAFETY_MARGIN) <= live <= (hi + SAFETY_MARGIN)
            mark = ' ' if in_range else '!'
            print(f"  {joint_name}={live:+.4f} rad{mark}  "
                  f"limits [{lo:+.3f}, {hi:+.3f}]", end='\r')
            last_print = now
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            break
    print()
    return live if live is not None else 0.0


# ── Smooth move ───────────────────────────────────────────────────────────────
def smooth_move(bus, node_id, joint_name, start_pos, end_pos, turn_offset,
                kp, kd, move_time=MOVE_TIME, dwell_time=DWELL_TIME,
                label='', csv_writer=None, move_num=None):
    """One smooth move on a single joint. Logs per-tick rows if csv_writer given."""
    enc_id = can_id(node_id, CMD_ENC_EST)
    mit_id = can_id(node_id, CMD_MIT)

    print(f"\n  ▶ {label}")
    print(f"      {joint_name} (node {node_id}):  {start_pos:+.4f} → {end_pos:+.4f} rad   "
          f"kp={kp:.0f} kd={kd:.1f}")

    last_pos  = start_pos
    mit_pos   = float('nan')
    mit_vel   = float('nan')
    last_torq = float('nan')
    max_err   = 0.0
    max_torq  = 0.0

    def pump():
        nonlocal last_pos, mit_pos, mit_vel, last_torq, max_torq
        r = bus.recv(timeout=0.0)
        while r is not None:
            if r.arbitration_id == enc_id:
                pos_rev = struct.unpack_from('<f', bytes(r.data), 0)[0]
                last_pos = wrap_to_pi(pos_rev * 2 * math.pi / GEAR_RATIO)
            elif r.arbitration_id == mit_id:
                fb = parse_mit_feedback(bytes(r.data))
                if fb is not None:
                    p, v, torque = fb
                    mit_pos   = wrap_to_pi(p)
                    mit_vel   = v
                    last_torq = torque
                    if abs(torque) > max_torq:
                        max_torq = abs(torque)
            r = bus.recv(timeout=0.0)

    def log_row(elapsed, phase, cmd_wrapped):
        if csv_writer is None:
            return
        csv_writer.writerow({
            't':           f"{elapsed:.4f}",
            'move_num':    move_num,
            'phase':       phase,
            'joint':       joint_name,
            'node_id':     node_id,
            'target':      f"{cmd_wrapped:+.6f}",
            'enc_pos':     f"{last_pos:+.6f}",
            'mit_pos':     f"{mit_pos:+.6f}" if not math.isnan(mit_pos) else '',
            'mit_vel':     f"{mit_vel:+.6f}" if not math.isnan(mit_vel) else '',
            'mit_torque':  f"{last_torq:+.6f}" if not math.isnan(last_torq) else '',
            'kp':          kp,
            'kd':          kd,
        })

    move_start = time.time()
    while True:
        elapsed = time.time() - move_start
        if elapsed >= move_time:
            break
        alpha = smoothstep(elapsed / move_time)
        pump()
        cmd_wrapped = start_pos + (end_pos - start_pos) * alpha
        send_mit(bus, node_id, cmd_wrapped + turn_offset, kp=kp, kd=kd)
        err = abs(wrap_to_pi(last_pos - cmd_wrapped))
        if err > max_err:
            max_err = err
        log_row(elapsed, 'move', cmd_wrapped)
        time.sleep(DT)

    dwell_end = time.time() + dwell_time
    while time.time() < dwell_end:
        pump()
        elapsed = time.time() - move_start
        send_mit(bus, node_id, end_pos + turn_offset, kp=kp, kd=kd)
        log_row(elapsed, 'dwell', end_pos)
        time.sleep(DT)

    elapsed_total = time.time() - move_start
    print(f"      reached in {elapsed_total:.2f}s")

    actual = read_position(bus, node_id, duration=0.2)
    ferr = wrap_to_pi(actual - end_pos) if actual is not None else float('nan')
    ltq_s = f"{last_torq:>+7.2f}N" if not math.isnan(last_torq) else "   n/a "
    print(f"      target={end_pos:+.4f}  actual={actual if actual is not None else float('nan'):+.4f}  "
          f"final_err={ferr:+.4f}  max_err={max_err:.4f}  hold_τ={ltq_s}  max_τ={max_torq:.2f}N")


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--leg', required=True, choices=['right', 'left'])
    parser.add_argument('--joint', required=True,
                        choices=['hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle'])
    parser.add_argument('--urdf', required=True, help='Path to URDF')
    parser.add_argument('--can', default=None,
                        help='Override CAN interface (default per leg)')
    parser.add_argument('--kp', type=float, default=DEFAULT_KP,
                        help=f'kp (default {DEFAULT_KP})')
    parser.add_argument('--kd', type=float, default=DEFAULT_KD,
                        help=f'kd (default {DEFAULT_KD})')
    parser.add_argument('--csv', default=None,
                        help='Path to time-series CSV (default: auto-generated)')
    args = parser.parse_args()

    leg_cfg   = LEGS[args.leg]
    can_iface = args.can or leg_cfg['can']
    if args.joint not in leg_cfg['joints']:
        print(f"ERROR: joint '{args.joint}' not in {args.leg} leg")
        sys.exit(1)
    node_id, urdf_name = leg_cfg['joints'][args.joint]

    # URDF limits
    try:
        lo, hi = parse_urdf_limits(args.urdf, urdf_name)
    except Exception as e:
        print(f"ERROR parsing URDF: {e}")
        sys.exit(1)

    # Banner
    print('=' * 64)
    print(f"        SINGLE JOINT TEST — {args.leg} {args.joint}")
    print('=' * 64)
    print(f"CAN interface : {can_iface}")
    print(f"Node ID       : {node_id}")
    print(f"URDF joint    : {urdf_name}")
    print(f"URDF limits   : [{lo:+.3f}, {hi:+.3f}] rad   (safety margin ±{SAFETY_MARGIN})")
    print(f"Gains         : kp={args.kp}  kd={args.kd}")
    print()

    bus = can.interface.Bus(channel=can_iface, interface='socketcan')

    def go_idle():
        set_state(bus, node_id, AXIS_STATE_IDLE)

    try:
        # ── Step 1 ────────────────────────────────────────────────────────────
        print("Step 1: Entering CLOSED_LOOP passive (kp=0) and reading position...")
        set_state(bus, node_id, AXIS_STATE_CLOSED_LOOP)
        time.sleep(0.8)
        passive_burst(bus, node_id, frames=30)
        raw_initial = read_position(bus, node_id, duration=0.5, wrap=False)
        if raw_initial is None:
            print("  >>> ABORTING: no encoder data received.")
            go_idle()
            return
        turn_offset = raw_initial - wrap_to_pi(raw_initial)
        initial = wrap_to_pi(raw_initial)
        if abs(turn_offset) > 1e-6:
            print(f"  Turn offset (added to MIT commands): {turn_offset:+.4f} rad  "
                  f"({turn_offset / (2 * math.pi):+.2f} × 2π)")

        # ── Step 2: Range check ──────────────────────────────────────────────
        print(f"\nStep 2: Range check  →  initial = {initial:+.4f} rad")
        if not (lo - SAFETY_MARGIN <= initial <= hi + SAFETY_MARGIN):
            print(f"  >>> ABORTING: initial {initial:+.4f} outside "
                  f"[{lo:+.3f}, {hi:+.3f}] rad.")
            go_idle()
            return
        print("  OK — inside URDF limits.")

        # ── Step 3 ────────────────────────────────────────────────────────────
        print(f"\nStep 3: Pose '{args.joint}' to the DESIRED TARGET.")
        target_pos = capture_pose(bus, node_id, args.joint, lo, hi, 'record target')
        print(f"  Recorded target: {target_pos:+.4f} rad")
        if not (lo - SAFETY_MARGIN <= target_pos <= hi + SAFETY_MARGIN):
            print(f"  >>> ABORTING: target outside URDF limits.")
            go_idle()
            return

        # ── Step 4 ────────────────────────────────────────────────────────────
        print(f"\nStep 4: Move '{args.joint}' to its HANGING / REST position.")
        hanging_pos = capture_pose(bus, node_id, args.joint, lo, hi, 'record hanging')
        print(f"  Recorded hanging: {hanging_pos:+.4f} rad")
        if not (lo - SAFETY_MARGIN <= hanging_pos <= hi + SAFETY_MARGIN):
            print(f"  >>> ABORTING: hanging outside URDF limits.")
            go_idle()
            return

        # ── Step 5: zero check ───────────────────────────────────────────────
        zero_pos = 0.0
        if not (lo - SAFETY_MARGIN <= zero_pos <= hi + SAFETY_MARGIN):
            print(f"\n  >>> ABORTING: zero pose outside URDF limits "
                  f"[{lo:+.3f}, {hi:+.3f}].")
            go_idle()
            return

        # ── Step 6: Motion sequence ──────────────────────────────────────────
        # Default save location: gim_test/test_data/ next to this script
        default_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_data')
        os.makedirs(default_dir, exist_ok=True)
        csv_path = args.csv or os.path.join(
            default_dir,
            f"joint_test_{args.leg}_{args.joint}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
        )
        csv_fields = ['t', 'move_num', 'phase', 'joint', 'node_id',
                      'target', 'enc_pos', 'mit_pos', 'mit_vel', 'mit_torque',
                      'kp', 'kd']
        csv_file = open(csv_path, 'w', newline='')
        csv_writer = csv.DictWriter(csv_file, fieldnames=csv_fields)
        csv_writer.writeheader()
        print(f"\nStep 5: Motion sequence  (CSV → {csv_path})")

        try:
            smooth_move(bus, node_id, args.joint, hanging_pos, zero_pos, turn_offset,
                        kp=args.kp, kd=args.kd,
                        label=f"Move 1/3 — Hanging → Zero  ({MOVE_TIME:.0f}s)",
                        csv_writer=csv_writer, move_num=1)
            smooth_move(bus, node_id, args.joint, zero_pos, target_pos, turn_offset,
                        kp=args.kp, kd=args.kd,
                        label=f"Move 2/3 — Zero → Target   ({MOVE_TIME:.0f}s)",
                        csv_writer=csv_writer, move_num=2)
            smooth_move(bus, node_id, args.joint, target_pos, zero_pos, turn_offset,
                        kp=args.kp, kd=args.kd,
                        label=f"Move 3/3 — Target → Zero   ({MOVE_TIME:.0f}s)",
                        csv_writer=csv_writer, move_num=3)
        finally:
            csv_file.close()
            print(f"\nCSV written: {csv_path}")

        print("\nStep 6: Motion complete. Setting joint to IDLE.")
        go_idle()
        print()
        print('=' * 64)
        print("  Test complete.")
        print('=' * 64)

    except KeyboardInterrupt:
        print("\n\nAborted — setting joint to IDLE.")
        go_idle()
    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()
