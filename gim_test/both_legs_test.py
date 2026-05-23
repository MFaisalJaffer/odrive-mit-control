#!/usr/bin/env python3
"""
Both-legs motion test — same workflow as leg_test.py, but commands all
10 joints across can0 (right leg) and can1 (left leg) simultaneously.

Workflow:
  1. Enter CLOSED_LOOP in passive (kp=0) mode on both legs so encoders
     are sampled while you can move the joints by hand.
  2. Read each joint's current position and check it against URDF limits.
     Abort if any joint is out of range or unreadable.
  3. User poses BOTH legs to the desired TARGET → ENTER → recorded.
  4. User moves BOTH legs to HANGING (straight down) → ENTER → recorded.
  5. Single 4-move sequence at kp=100 (override with --kp):
        Hanging → Zero → Target → Zero → Hanging
     Per-tick CSV is written for sim comparison.
  6. All joints return to IDLE.

Usage:
  python3 both_legs_test.py --urdf /path/to/robot.urdf
  python3 both_legs_test.py --urdf .../robot.urdf --kp 150
  python3 both_legs_test.py --urdf .../robot.urdf --csv /tmp/run.csv

  Per-joint kp/kd via JSON (disables sweep):
    {"right_hip_pitch": {"kp": 150, "kd": 3}, "left_knee": {"kp": 200}}
  Joint key format: "<leg>_<friendly_name>"
"""

import argparse
import can
import csv
import json
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

# No kp sweep for the both-legs run — single pass at args.kp (default 100)

# ── Joint layout: (node_id, friendly_name, urdf_joint_name, leg) ──────────────
JOINTS = [
    # Right leg (can0, nodes 3-7)
    (3,  'hip_pitch', 'dof_right_hip_pitch_04', 'right'),
    (4,  'hip_roll',  'dof_right_hip_roll_03',  'right'),
    (5,  'hip_yaw',   'dof_right_hip_yaw_03',   'right'),
    (6,  'knee',      'dof_right_knee_04',      'right'),
    (7,  'ankle',     'dof_right_ankle_02',     'right'),
    # Left leg (can1, nodes 13-17)
    (13, 'hip_pitch', 'dof_left_hip_pitch_04',  'left'),
    (14, 'hip_roll',  'dof_left_hip_roll_03',   'left'),
    (15, 'hip_yaw',   'dof_left_hip_yaw_03',    'left'),
    (16, 'knee',      'dof_left_knee_04',       'left'),
    (17, 'ankle',     'dof_left_ankle_02',      'left'),
]

LEG_TO_CAN = {'right': 'can0', 'left': 'can1'}

# ── ODrive CAN Simple constants ───────────────────────────────────────────────
CMD_ENC_EST   = 0x009
CMD_SET_STATE = 0x007
CMD_MIT       = 0x008
CMD_GET_IQ    = 0x014   # Iq_setpoint (float32 A) + Iq_measured (float32 A)

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


# ── CAN helpers ───────────────────────────────────────────────────────────────
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


def compute_turn_offsets(raw_positions):
    return {nid: (raw - wrap_to_pi(raw)) for nid, raw in raw_positions.items()
            if raw is not None}


def passive_burst(bus_by_node, node_ids, frames=20):
    """Send `frames` rounds of kp=0 MIT commands across both buses."""
    for _ in range(frames):
        for nid in node_ids:
            send_mit(bus_by_node[nid], nid, 0.0, kp=PASSIVE_KP, kd=PASSIVE_KD)
        time.sleep(DT)


def drain_positions(buses, node_ids, leg_by_node, duration=0.5, wrap=True):
    """Listen on BOTH buses for encoder estimates. Returns {nid: rad}."""
    enc_id_to_nid = {}  # (leg, arb_id) → nid
    for nid in node_ids:
        enc_id_to_nid[(leg_by_node[nid], can_id(nid, CMD_ENC_EST))] = nid
    positions = {}
    deadline = time.time() + duration
    while time.time() < deadline:
        for leg, bus in buses.items():
            r = bus.recv(timeout=0.005)
            if r:
                key = (leg, r.arbitration_id)
                if key in enc_id_to_nid:
                    nid = enc_id_to_nid[key]
                    pos_rev = struct.unpack_from('<f', bytes(r.data), 0)[0]
                    pos_rad = pos_rev * 2 * math.pi / GEAR_RATIO
                    positions[nid] = wrap_to_pi(pos_rad) if wrap else pos_rad
    return positions


# ── URDF parsing ──────────────────────────────────────────────────────────────
def parse_urdf_limits(urdf_path, urdf_joint_names):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    found = {}
    for j in root.findall('joint'):
        name = j.get('name')
        if name in urdf_joint_names:
            lim = j.find('limit')
            if lim is None:
                raise ValueError(f"Joint '{name}' has no <limit> in URDF")
            found[name] = (float(lim.get('lower', '0')), float(lim.get('upper', '0')))
    missing = set(urdf_joint_names) - set(found.keys())
    if missing:
        raise ValueError(f"Joint(s) not found in URDF: {sorted(missing)}")
    return found


def check_within_limits(positions, limits_by_node, name_by_node, leg_by_node,
                        label='positions'):
    any_bad = False
    print(f"  {label}:")
    print(f"  {'leg':<6} {'node':<5} {'joint':<11} {'pos':>10}  {'lower':>10}  {'upper':>10}   status")
    print('  ' + '-' * 72)
    for nid in sorted(limits_by_node.keys()):
        lo, hi = limits_by_node[nid]
        pos = positions.get(nid)
        if pos is None:
            print(f"  {leg_by_node[nid]:<6} {nid:<5} {name_by_node[nid]:<11} "
                  f"{'?':>10}  {lo:>+10.4f}  {hi:>+10.4f}   NO READING")
            any_bad = True
            continue
        in_range = (lo - SAFETY_MARGIN) <= pos <= (hi + SAFETY_MARGIN)
        status = 'OK' if in_range else 'OUT OF RANGE'
        if not in_range:
            any_bad = True
        print(f"  {leg_by_node[nid]:<6} {nid:<5} {name_by_node[nid]:<11} "
              f"{pos:>+10.4f}  {lo:>+10.4f}  {hi:>+10.4f}   {status}")
    return not any_bad


# ── Pose capture (live, CLOSED_LOOP passive) ─────────────────────────────────
def capture_pose(buses, bus_by_node, node_ids, name_by_node, leg_by_node,
                 limits_by_node, prompt):
    """Continuously send kp=0 MIT on both buses, show live positions,
    return wrapped readings when the user hits ENTER."""
    enc_id_to_nid = {}
    for nid in node_ids:
        enc_id_to_nid[(leg_by_node[nid], can_id(nid, CMD_ENC_EST))] = nid
    live_pos = {}
    last_mit = 0.0
    last_print = 0.0
    print(f"  (move both legs by hand; press ENTER to {prompt})")
    print()

    while True:
        now = time.time()
        if now - last_mit > 0.02:  # 50 Hz passive
            for nid in node_ids:
                send_mit(bus_by_node[nid], nid, 0.0, kp=PASSIVE_KP, kd=PASSIVE_KD)
            last_mit = now

        # Drain both buses
        for leg, bus in buses.items():
            r = bus.recv(timeout=0.005)
            if r:
                key = (leg, r.arbitration_id)
                if key in enc_id_to_nid:
                    nid = enc_id_to_nid[key]
                    pos_rev = struct.unpack_from('<f', bytes(r.data), 0)[0]
                    live_pos[nid] = wrap_to_pi(pos_rev * 2 * math.pi / GEAR_RATIO)

        if live_pos and (now - last_print > 0.1):  # 10 Hz display
            # Two rows (right, then left) — too many joints for one line
            for leg in ('right', 'left'):
                line = f"  {leg:<6} "
                for nid in node_ids:
                    if leg_by_node[nid] != leg:
                        continue
                    rad = live_pos.get(nid)
                    lo, hi = limits_by_node[nid]
                    if rad is None:
                        cell = f"{name_by_node[nid]}=?     "
                    else:
                        out = '!' if not ((lo - SAFETY_MARGIN) <= rad <= (hi + SAFETY_MARGIN)) else ' '
                        cell = f"{name_by_node[nid]}={rad:+.3f}{out} "
                    line += cell
                print(line)
            # Move cursor back up two lines to overwrite next iteration
            print('\033[2A', end='')
            last_print = now

        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            break

    print()
    print()
    return {nid: live_pos.get(nid, 0.0) for nid in node_ids}


# ── Smooth move ───────────────────────────────────────────────────────────────
def smooth_move(buses, bus_by_node, node_ids, name_by_node, leg_by_node,
                start_pos, end_pos, gains_by_node, turn_offsets,
                move_time=MOVE_TIME, dwell_time=DWELL_TIME,
                label='', csv_writer=None, kp_run=None, move_num=None):
    """Coordinate one smooth move across both legs simultaneously."""
    enc_id_to_nid = {(leg_by_node[nid], can_id(nid, CMD_ENC_EST)): nid for nid in node_ids}
    mit_id_to_nid = {(leg_by_node[nid], can_id(nid, CMD_MIT)):     nid for nid in node_ids}
    iq_id_to_nid  = {(leg_by_node[nid], can_id(nid, CMD_GET_IQ)):  nid for nid in node_ids}

    print(f"\n  ▶ {label}")
    for nid in node_ids:
        s = start_pos.get(nid, 0.0)
        e = end_pos.get(nid, 0.0)
        kp, kd = gains_by_node[nid]
        print(f"      {leg_by_node[nid]:<6} {name_by_node[nid]:<11} (node {nid:>2}):  "
              f"{s:+.4f} → {e:+.4f} rad   kp={kp:.0f} kd={kd:.1f}")

    max_err     = {nid: 0.0 for nid in node_ids}
    last_pos    = {nid: start_pos.get(nid, 0.0) for nid in node_ids}
    mit_pos     = {nid: float('nan') for nid in node_ids}
    mit_vel     = {nid: float('nan') for nid in node_ids}
    last_torq   = {nid: float('nan') for nid in node_ids}
    max_torq    = {nid: 0.0 for nid in node_ids}
    iq_set      = {nid: float('nan') for nid in node_ids}
    iq_meas     = {nid: float('nan') for nid in node_ids}
    max_iq_meas = {nid: 0.0 for nid in node_ids}

    def pump_can():
        for leg, bus in buses.items():
            r = bus.recv(timeout=0.0)
            while r is not None:
                key = (leg, r.arbitration_id)
                if key in enc_id_to_nid:
                    nid = enc_id_to_nid[key]
                    pos_rev = struct.unpack_from('<f', bytes(r.data), 0)[0]
                    last_pos[nid] = wrap_to_pi(pos_rev * 2 * math.pi / GEAR_RATIO)
                elif key in mit_id_to_nid:
                    nid = mit_id_to_nid[key]
                    fb = parse_mit_feedback(bytes(r.data))
                    if fb is not None:
                        p, v, torque = fb
                        mit_pos[nid]   = wrap_to_pi(p)
                        mit_vel[nid]   = v
                        last_torq[nid] = torque
                        if abs(torque) > max_torq[nid]:
                            max_torq[nid] = abs(torque)
                elif key in iq_id_to_nid and len(r.data) >= 8:
                    nid = iq_id_to_nid[key]
                    iq_set[nid]  = struct.unpack_from('<f', bytes(r.data), 0)[0]
                    iq_meas[nid] = struct.unpack_from('<f', bytes(r.data), 4)[0]
                    if abs(iq_meas[nid]) > max_iq_meas[nid]:
                        max_iq_meas[nid] = abs(iq_meas[nid])
                r = bus.recv(timeout=0.0)

    def log_row(elapsed, phase, nid, cmd_wrapped):
        if csv_writer is None:
            return
        kp, kd = gains_by_node[nid]
        csv_writer.writerow({
            't':           f"{elapsed:.4f}",
            'kp_run':      kp_run,
            'move_num':    move_num,
            'phase':       phase,
            'leg':         leg_by_node[nid],
            'joint':       name_by_node[nid],
            'node_id':     nid,
            'target':      f"{cmd_wrapped:+.6f}",
            'enc_pos':     f"{last_pos[nid]:+.6f}",
            'mit_pos':     f"{mit_pos[nid]:+.6f}" if not math.isnan(mit_pos[nid]) else '',
            'mit_vel':     f"{mit_vel[nid]:+.6f}" if not math.isnan(mit_vel[nid]) else '',
            'mit_torque':  f"{last_torq[nid]:+.6f}" if not math.isnan(last_torq[nid]) else '',
            'iq_set':      f"{iq_set[nid]:+.6f}"  if not math.isnan(iq_set[nid])  else '',
            'iq_meas':     f"{iq_meas[nid]:+.6f}" if not math.isnan(iq_meas[nid]) else '',
            'kp':          kp,
            'kd':          kd,
        })

    move_start = time.time()
    while True:
        elapsed = time.time() - move_start
        if elapsed >= move_time:
            break
        alpha = smoothstep(elapsed / move_time)
        pump_can()

        for nid in node_ids:
            s = start_pos.get(nid, 0.0)
            e = end_pos.get(nid, 0.0)
            cmd_wrapped = s + (e - s) * alpha
            cmd_raw = cmd_wrapped + turn_offsets.get(nid, 0.0)
            kp, kd = gains_by_node[nid]
            send_mit(bus_by_node[nid], nid, cmd_raw, kp=kp, kd=kd)
            err = abs(wrap_to_pi(last_pos[nid] - cmd_wrapped))
            if err > max_err[nid]:
                max_err[nid] = err
            log_row(elapsed, 'move', nid, cmd_wrapped)
        time.sleep(DT)

    dwell_end = time.time() + dwell_time
    while time.time() < dwell_end:
        pump_can()
        elapsed = time.time() - move_start
        for nid in node_ids:
            e = end_pos.get(nid, 0.0)
            cmd_raw = e + turn_offsets.get(nid, 0.0)
            kp, kd = gains_by_node[nid]
            send_mit(bus_by_node[nid], nid, cmd_raw, kp=kp, kd=kd)
            log_row(elapsed, 'dwell', nid, e)
        time.sleep(DT)

    elapsed_total = time.time() - move_start
    print(f"      reached in {elapsed_total:.2f}s")

    actual = drain_positions({l: buses[l] for l in buses}, node_ids,
                              leg_by_node, duration=0.2)
    print(f"      {'leg':<6} {'joint':<11}  {'target':>9}  {'actual':>9}  "
          f"{'final_err':>10}  {'max_err':>8}  {'hold_τ':>8}  {'max_τ':>8}  "
          f"{'iq_meas':>8}  {'max_iq':>8}")
    print(f"      {'-' * 106}")
    for nid in node_ids:
        target = end_pos.get(nid, 0.0)
        act    = actual.get(nid, float('nan'))
        ferr   = wrap_to_pi(act - target) if not math.isnan(act) else float('nan')
        merr   = max_err[nid]
        ltq    = last_torq[nid]
        mtq    = max_torq[nid]
        liq    = iq_meas[nid]
        miq    = max_iq_meas[nid]
        flag   = '  <-- check' if (not math.isnan(ferr) and abs(ferr) > 0.1) or merr > 0.2 else ''
        ltq_s  = f"{ltq:>+7.2f}N" if not math.isnan(ltq) else "   n/a "
        mtq_s  = f"{mtq:>7.2f}N"
        liq_s  = f"{liq:>+7.2f}A" if not math.isnan(liq) else "   n/a "
        miq_s  = f"{miq:>7.2f}A"
        print(f"      {leg_by_node[nid]:<6} {name_by_node[nid]:<11}  "
              f"{target:>+9.4f}  {act:>+9.4f}  {ferr:>+10.4f}  "
              f"{merr:>8.4f}  {ltq_s}  {mtq_s}  {liq_s}  {miq_s}{flag}")


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--urdf', required=True,
                        help='Path to URDF for joint-limit safety check')
    parser.add_argument('--right-can', default=LEG_TO_CAN['right'],
                        help=f'Right-leg CAN interface (default: {LEG_TO_CAN["right"]})')
    parser.add_argument('--left-can',  default=LEG_TO_CAN['left'],
                        help=f'Left-leg CAN interface (default: {LEG_TO_CAN["left"]})')
    parser.add_argument('--kp', type=float, default=DEFAULT_KP,
                        help=f'Global kp (default {DEFAULT_KP})')
    parser.add_argument('--kd', type=float, default=DEFAULT_KD,
                        help=f'Global kd (default {DEFAULT_KD})')
    parser.add_argument('--gains', default=None,
                        help='Path to JSON file with per-joint kp/kd overrides. '
                             'Joint keys: "<leg>_<friendly>" e.g. "right_hip_pitch".')
    parser.add_argument('--csv', default=None,
                        help='Path to write time-series CSV (default: auto in CWD).')
    args = parser.parse_args()

    # Build lookups from JOINTS
    node_ids          = [j[0] for j in JOINTS]
    name_by_node      = {j[0]: j[1] for j in JOINTS}
    urdf_name_by_node = {j[0]: j[2] for j in JOINTS}
    leg_by_node       = {j[0]: j[3] for j in JOINTS}

    # URDF limits
    try:
        urdf_limits = parse_urdf_limits(args.urdf, list(urdf_name_by_node.values()))
    except Exception as e:
        print(f"ERROR parsing URDF: {e}")
        sys.exit(1)
    limits_by_node = {nid: urdf_limits[urdf_name_by_node[nid]] for nid in node_ids}

    # Per-joint gains
    gains_by_node = {nid: (args.kp, args.kd) for nid in node_ids}
    if args.gains:
        try:
            with open(args.gains) as f:
                raw = json.load(f)
        except Exception as e:
            print(f"ERROR loading --gains: {e}")
            sys.exit(1)
        # Key format: "<leg>_<friendly>"
        key_to_node = {f"{leg_by_node[nid]}_{name_by_node[nid]}": nid for nid in node_ids}
        for k, vals in raw.items():
            if k not in key_to_node:
                print(f"  WARNING: --gains has unknown key '{k}' (ignored). "
                      f"Valid keys: {sorted(key_to_node)}")
                continue
            nid = key_to_node[k]
            gains_by_node[nid] = (float(vals.get('kp', args.kp)),
                                   float(vals.get('kd', args.kd)))

    # Banner
    print('=' * 76)
    print(f"        BOTH LEGS MOTION TEST")
    print('=' * 76)
    print(f"Right CAN     : {args.right_can}")
    print(f"Left  CAN     : {args.left_can}")
    print(f"URDF          : {args.urdf}")
    print(f"Gear ratio    : {GEAR_RATIO}")
    print(f"Safety margin : {SAFETY_MARGIN:+.3f} rad on each URDF limit")
    print()
    print("Joints (URDF limits + gains):")
    for nid in node_ids:
        lo, hi = limits_by_node[nid]
        kp, kd = gains_by_node[nid]
        print(f"  {leg_by_node[nid]:<6} node {nid:>2}  {name_by_node[nid]:<11} "
              f"{urdf_name_by_node[nid]:<28} [{lo:+.3f}, {hi:+.3f}]   kp={kp:>5.1f}  kd={kd:>4.1f}")
    print()

    # Open both buses
    buses = {
        'right': can.interface.Bus(channel=args.right_can, interface='socketcan'),
        'left':  can.interface.Bus(channel=args.left_can,  interface='socketcan'),
    }
    bus_by_node = {nid: buses[leg_by_node[nid]] for nid in node_ids}

    def go_idle():
        for nid in node_ids:
            set_state(bus_by_node[nid], nid, AXIS_STATE_IDLE)

    try:
        # ── Step 1 ────────────────────────────────────────────────────────────
        print("Step 1: Entering CLOSED_LOOP on both legs and sampling encoders (kp=0 passive)...")
        for nid in node_ids:
            set_state(bus_by_node[nid], nid, AXIS_STATE_CLOSED_LOOP)
        time.sleep(0.8)
        passive_burst(bus_by_node, node_ids, frames=30)
        initial_raw = drain_positions(buses, node_ids, leg_by_node,
                                       duration=0.5, wrap=False)
        turn_offsets = compute_turn_offsets(initial_raw)
        initial = {nid: wrap_to_pi(p) for nid, p in initial_raw.items()}
        if turn_offsets:
            print("  Turn offsets (will be added to MIT commands):")
            for nid in node_ids:
                off = turn_offsets.get(nid)
                if off is not None and abs(off) > 1e-6:
                    print(f"    {leg_by_node[nid]:<6} {name_by_node[nid]:<11} (node {nid}): "
                          f"{off:+.4f} rad  ({off / (2 * math.pi):+.2f} × 2π)")

        # ── Step 2 ────────────────────────────────────────────────────────────
        print("\nStep 2: Initial position range check")
        if not check_within_limits(initial, limits_by_node, name_by_node,
                                    leg_by_node, label='initial positions'):
            print("\n  >>> ABORTING: one or more joints out of URDF limits.")
            go_idle()
            return

        # ── Step 3 ────────────────────────────────────────────────────────────
        print("\nStep 3: Pose BOTH legs to the DESIRED TARGET.")
        target_pos = capture_pose(buses, bus_by_node, node_ids, name_by_node,
                                   leg_by_node, limits_by_node, prompt='record target')
        print("  Recorded target positions:")
        for nid in node_ids:
            print(f"    {leg_by_node[nid]:<6} {name_by_node[nid]:<11} (node {nid}): "
                  f"{target_pos[nid]:+.4f} rad")
        if not check_within_limits(target_pos, limits_by_node, name_by_node,
                                    leg_by_node, label='target positions'):
            print("  >>> ABORTING: target outside URDF limits.")
            go_idle()
            return

        # ── Step 4 ────────────────────────────────────────────────────────────
        print("\nStep 4: Move BOTH legs to HANGING (straight down).")
        hanging_pos = capture_pose(buses, bus_by_node, node_ids, name_by_node,
                                    leg_by_node, limits_by_node, prompt='record hanging')
        print("  Recorded hanging positions:")
        for nid in node_ids:
            print(f"    {leg_by_node[nid]:<6} {name_by_node[nid]:<11} (node {nid}): "
                  f"{hanging_pos[nid]:+.4f} rad")
        if not check_within_limits(hanging_pos, limits_by_node, name_by_node,
                                    leg_by_node, label='hanging positions'):
            print("  >>> ABORTING: hanging position outside URDF limits.")
            go_idle()
            return

        # ── Step 5: zero pose ────────────────────────────────────────────────
        zero_pos = {nid: 0.0 for nid in node_ids}
        for nid in node_ids:
            lo, hi = limits_by_node[nid]
            if not (lo - SAFETY_MARGIN <= 0.0 <= hi + SAFETY_MARGIN):
                print(f"\n  >>> ABORTING: zero pose outside limits for "
                      f"{leg_by_node[nid]} {name_by_node[nid]}: [{lo:+.3f}, {hi:+.3f}]")
                go_idle()
                return

        # ── Step 6: Motion sequence (single run) ─────────────────────────────
        # Default save location: gim_test/test_data/ next to this script
        default_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_data')
        os.makedirs(default_dir, exist_ok=True)
        csv_path = args.csv or os.path.join(
            default_dir,
            f"both_legs_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
        )
        csv_fields = ['t', 'kp_run', 'move_num', 'phase', 'leg', 'joint',
                      'node_id', 'target', 'enc_pos', 'mit_pos', 'mit_vel',
                      'mit_torque', 'iq_set', 'iq_meas', 'kp', 'kd']
        csv_file = open(csv_path, 'w', newline='')
        csv_writer = csv.DictWriter(csv_file, fieldnames=csv_fields)
        csv_writer.writeheader()

        # Use the global --kp/--kd as the kp_run label in the CSV (single value)
        kp_run = args.kp
        print(f"\nStep 5: Motion sequence  (CSV → {csv_path})")
        print(f"  Single run at kp={args.kp}, kd={args.kd}")

        try:
            smooth_move(buses, bus_by_node, node_ids, name_by_node, leg_by_node,
                        hanging_pos, zero_pos, gains_by_node, turn_offsets,
                        label=f"Move 1/4 — Hanging → Zero    ({MOVE_TIME:.0f}s)",
                        csv_writer=csv_writer, kp_run=kp_run, move_num=1)
            smooth_move(buses, bus_by_node, node_ids, name_by_node, leg_by_node,
                        zero_pos, target_pos, gains_by_node, turn_offsets,
                        label=f"Move 2/4 — Zero → Target     ({MOVE_TIME:.0f}s)",
                        csv_writer=csv_writer, kp_run=kp_run, move_num=2)
            smooth_move(buses, bus_by_node, node_ids, name_by_node, leg_by_node,
                        target_pos, zero_pos, gains_by_node, turn_offsets,
                        label=f"Move 3/4 — Target → Zero     ({MOVE_TIME:.0f}s)",
                        csv_writer=csv_writer, kp_run=kp_run, move_num=3)
            smooth_move(buses, bus_by_node, node_ids, name_by_node, leg_by_node,
                        zero_pos, hanging_pos, gains_by_node, turn_offsets,
                        label=f"Move 4/4 — Zero → Hanging    ({MOVE_TIME:.0f}s)",
                        csv_writer=csv_writer, kp_run=kp_run, move_num=4)
        finally:
            csv_file.close()
            print(f"\nCSV written: {csv_path}")

        # ── Done ──────────────────────────────────────────────────────────────
        print("\nStep 6: Motion complete. Setting all joints to IDLE.")
        go_idle()
        print()
        print('=' * 76)
        print("  Test complete.")
        print('=' * 76)

    except KeyboardInterrupt:
        print("\n\nAborted — setting all joints to IDLE.")
        go_idle()
    finally:
        for bus in buses.values():
            bus.shutdown()


if __name__ == '__main__':
    main()
