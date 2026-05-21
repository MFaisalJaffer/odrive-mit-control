#!/usr/bin/env python3
"""
Leg motion test — pose capture then smooth playback.

Workflow:
  1. Enter CLOSED_LOOP in passive (kp=0) mode so encoders are actually sampled
     while the leg is free to move by hand.
  2. Read each joint's current position and check it against URDF joint limits.
     Abort if any joint is out of range (or unreadable).
  3. User poses the leg to the desired TARGET position → ENTER → recorded.
  4. User moves the leg to its HANGING position (straight down) → ENTER → recorded.
  5. Three smooth 3s moves under kp=EFFORT:
        Hanging → Zero → Target → Zero
  6. All joints return to IDLE.

Usage:
  # Global gains (default kp=50, kd=2.0 — intentionally low for first run):
  python3 leg_test.py --leg right --urdf /path/to/robot.urdf
  python3 leg_test.py --leg right --urdf .../robot.urdf --kp 120 --kd 2.5

  # Per-joint override via JSON file (any joints missing from the file fall
  # back to the global --kp/--kd):
  python3 leg_test.py --leg right --urdf .../robot.urdf --gains gains.json

  gains.json format:
    {
      "hip_pitch": {"kp": 100, "kd": 2.5},
      "knee":      {"kp": 250, "kd": 3.5}
    }

Tuning tips:
  - Start with the defaults (kp=50, kd=2). Run the three-move sequence and
    look at the per-joint 'error' column in each summary table.
  - For joints that overshoot or oscillate, raise kd first.
  - For joints with too much steady-state error, raise kp. Watch for
    oscillation as you raise kp; if it starts, back off and raise kd.
  - These actuators saturate at ~18 Nm so very high kp won't help once you're
    torque-limited — at that point you'd need feedforward (see docs).
"""

import argparse
import can
import json
import math
import select
import struct
import sys
import time
import xml.etree.ElementTree as ET

# ── Motion params ─────────────────────────────────────────────────────────────
GEAR_RATIO     = 8.0
MOVE_TIME      = 3.0     # seconds per move
DWELL_TIME     = 1.0     # seconds to hold at each waypoint
DT             = 0.01    # 100 Hz loop
DEFAULT_KP     = 50.0    # Default kp — start low, tune up
DEFAULT_KD     = 2.0     # Default kd
PASSIVE_KP     = 0.0     # kp for pose-capture passive mode
PASSIVE_KD     = 0.0     # kd for pose-capture passive mode
SAFETY_MARGIN  = 0.05    # rad — added to URDF limits for the initial range check

# ── Leg layout ────────────────────────────────────────────────────────────────
# Each joint: (node_id, friendly_name, urdf_joint_name)
LEGS = {
    'right': {
        'can': 'can0',
        'joints': [
            (3, 'hip_pitch', 'dof_right_hip_pitch_04'),
            (4, 'hip_roll',  'dof_right_hip_roll_03'),
            (5, 'hip_yaw',   'dof_right_hip_yaw_03'),
            (6, 'knee',      'dof_right_knee_04'),
            (7, 'ankle',     'dof_right_ankle_02'),
        ],
    },
    'left': {
        'can': 'can1',
        'joints': [
            (13, 'hip_pitch', 'dof_left_hip_pitch_04'),
            (14, 'hip_roll',  'dof_left_hip_roll_03'),
            (15, 'hip_yaw',   'dof_left_hip_yaw_03'),
            (16, 'knee',      'dof_left_knee_04'),
            (17, 'ankle',     'dof_left_ankle_02'),
        ],
    },
}

# ── ODrive CAN Simple constants ───────────────────────────────────────────────
CMD_ENC_EST   = 0x009
CMD_SET_STATE = 0x007
CMD_MIT       = 0x008

AXIS_STATE_IDLE        = 1
AXIS_STATE_CLOSED_LOOP = 8

# MIT bitfield ranges (manual section 4.1.3)
MIT_P_MIN,  MIT_P_MAX  = -12.5, 12.5    # rad (output shaft)
MIT_V_MIN,  MIT_V_MAX  = -45.0, 45.0    # rad/s
MIT_KP_MIN, MIT_KP_MAX =   0.0, 500.0
MIT_KD_MIN, MIT_KD_MAX =   0.0, 5.0
MIT_T_MIN,  MIT_T_MAX  = -18.0, 18.0    # Nm


# ── CAN helpers ───────────────────────────────────────────────────────────────
def can_id(node_id, cmd):
    return (node_id << 5) | cmd


def send_raw(bus, node_id, cmd, data):
    bus.send(can.Message(
        arbitration_id=can_id(node_id, cmd),
        data=data,
        is_extended_id=False,
    ))


def set_state(bus, node_id, state):
    send_raw(bus, node_id, CMD_SET_STATE, struct.pack('<I', state))


def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    return int((x - x_min) / (x_max - x_min) * ((1 << bits) - 1))


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


def passive_burst(bus, node_ids, frames=20):
    """Send `frames` rounds of kp=0 MIT commands to all joints (~DT spacing).

    Keeps the watchdog happy and prevents any holding torque while still
    leaving the axis in CLOSED_LOOP so the encoder is sampled."""
    for _ in range(frames):
        for nid in node_ids:
            send_mit(bus, nid, 0.0, kp=PASSIVE_KP, kd=PASSIVE_KD)
        time.sleep(DT)


def drain_positions(bus, node_ids, duration=0.5, wrap=True):
    """Listen for `duration` seconds; return latest pos per node (rad).

    If wrap=True (default), positions are normalized to [-π, +π] so multi-turn
    firmware drift doesn't pollute the reading. Set wrap=False to get the raw
    multi-turn value (only needed when computing turn offsets)."""
    enc_ids = {can_id(nid, CMD_ENC_EST): nid for nid in node_ids}
    positions = {}
    deadline = time.time() + duration
    while time.time() < deadline:
        r = bus.recv(timeout=0.02)
        if r and r.arbitration_id in enc_ids:
            nid = enc_ids[r.arbitration_id]
            pos_rev = struct.unpack_from('<f', bytes(r.data), 0)[0]
            pos_rad = pos_rev * 2 * math.pi / GEAR_RATIO
            positions[nid] = wrap_to_pi(pos_rad) if wrap else pos_rad
    return positions


def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def wrap_to_pi(x):
    """Wrap angle to [-π, +π]. None passes through."""
    if x is None:
        return None
    return ((x + math.pi) % (2 * math.pi)) - math.pi


def compute_turn_offsets(raw_positions):
    """Per-joint integer-multiple-of-2π offset: raw - wrap_to_pi(raw).

    Applied to commands so MIT targets stay in the firmware's current
    multi-turn coordinate even though we plan motion in wrapped [-π, +π] space."""
    return {nid: (raw - wrap_to_pi(raw)) for nid, raw in raw_positions.items()
            if raw is not None}


# ── URDF parsing ──────────────────────────────────────────────────────────────
def parse_urdf_limits(urdf_path, urdf_joint_names):
    """Return {urdf_joint_name: (lower_rad, upper_rad)}. Raises if any missing.

    Only looks at top-level <joint> elements (the kinematic tree), not the
    duplicate <joint> entries inside <ros2_control> which lack <limit>."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    found = {}
    for j in root.findall('joint'):
        name = j.get('name')
        if name in urdf_joint_names:
            lim = j.find('limit')
            if lim is None:
                raise ValueError(f"Joint '{name}' has no <limit> element in URDF")
            found[name] = (float(lim.get('lower', '0')), float(lim.get('upper', '0')))
    missing = set(urdf_joint_names) - set(found.keys())
    if missing:
        raise ValueError(f"Joint(s) not found in URDF: {sorted(missing)}")
    return found


def check_within_limits(positions, limits_by_node, name_by_node, label='positions'):
    """Print a table of positions vs URDF limits. Return True if all in range."""
    any_bad = False
    print(f"  {label}:")
    print(f"  {'node':<5} {'joint':<12} {'pos':>10}  {'lower':>10}  {'upper':>10}   status")
    print('  ' + '-' * 64)
    for nid, (lo, hi) in sorted(limits_by_node.items()):
        pos = positions.get(nid)
        joint_name = name_by_node[nid]
        if pos is None:
            print(f"  {nid:<5} {joint_name:<12} {'?':>10}  {lo:>+10.4f}  {hi:>+10.4f}   NO READING")
            any_bad = True
            continue
        in_range = (lo - SAFETY_MARGIN) <= pos <= (hi + SAFETY_MARGIN)
        status = 'OK' if in_range else 'OUT OF RANGE'
        if not in_range:
            any_bad = True
        print(f"  {nid:<5} {joint_name:<12} {pos:>+10.4f}  {lo:>+10.4f}  {hi:>+10.4f}   {status}")
    return not any_bad


# ── Pose capture (live, in CLOSED_LOOP passive mode) ──────────────────────────
def capture_pose(bus, node_ids, name_by_node, limits_by_node, prompt):
    """Continuously send kp=0 MIT (so encoder is sampled), display live
    positions, and return the last reading when the user presses ENTER."""
    enc_ids = {can_id(nid, CMD_ENC_EST): nid for nid in node_ids}
    live_pos = {}

    last_mit = 0.0
    last_print = 0.0
    print(f"  (move the leg by hand; press ENTER to {prompt})")
    print()

    while True:
        now = time.time()
        if now - last_mit > 0.02:        # 50 Hz passive MIT
            for nid in node_ids:
                send_mit(bus, nid, 0.0, kp=PASSIVE_KP, kd=PASSIVE_KD)
            last_mit = now

        r = bus.recv(timeout=0.02)
        if r and r.arbitration_id in enc_ids:
            nid = enc_ids[r.arbitration_id]
            pos_rev = struct.unpack_from('<f', bytes(r.data), 0)[0]
            live_pos[nid] = wrap_to_pi(pos_rev * 2 * math.pi / GEAR_RATIO)

        if live_pos and (now - last_print > 0.05):  # 20 Hz display
            line = '  '
            for nid in node_ids:
                rad = live_pos.get(nid)
                lo, hi = limits_by_node[nid]
                if rad is None:
                    cell = f"{name_by_node[nid]}=?     "
                else:
                    out = '!' if not ((lo - SAFETY_MARGIN) <= rad <= (hi + SAFETY_MARGIN)) else ' '
                    cell = f"{name_by_node[nid]}={rad:+.4f}{out} "
                line += cell
            print(line, end='\r')
            last_print = now

        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            break

    print()
    return {nid: live_pos.get(nid, 0.0) for nid in node_ids}


# ── Smooth move ───────────────────────────────────────────────────────────────
def smooth_move(bus, node_ids, name_by_node, start_pos, end_pos,
                gains_by_node, turn_offsets,
                move_time=MOVE_TIME, dwell_time=DWELL_TIME,
                label=''):
    """Smooth-interpolate from start_pos to end_pos, then dwell at end_pos.

    start_pos, end_pos: positions in wrapped [-π, +π] space.
    gains_by_node: {node_id: (kp, kd)} per-joint gains.
    turn_offsets[nid]: added to every MIT command so the firmware sees a target
        in its multi-turn coordinate (handles GIM multi-turn drift across boots).

    Tracks max |error| seen during the move + steady-state error after dwell."""
    enc_ids = {can_id(nid, CMD_ENC_EST): nid for nid in node_ids}

    print(f"\n  ▶ {label}")
    for nid in node_ids:
        s = start_pos.get(nid, 0.0)
        e = end_pos.get(nid, 0.0)
        kp, kd = gains_by_node[nid]
        print(f"      {name_by_node[nid]:12s} (node {nid}):  {s:+.4f} → {e:+.4f} rad   kp={kp:.0f} kd={kd:.1f}")

    max_err   = {nid: 0.0 for nid in node_ids}
    last_pos  = {nid: start_pos.get(nid, 0.0) for nid in node_ids}

    move_start = time.time()
    while True:
        elapsed = time.time() - move_start
        if elapsed >= move_time:
            break
        alpha = smoothstep(elapsed / move_time)

        # Pump any encoder frames in the queue so we can track error live.
        # Wrap to [-π, +π] to match start_pos/end_pos space.
        r = bus.recv(timeout=0.0)
        while r is not None:
            if r.arbitration_id in enc_ids:
                nid = enc_ids[r.arbitration_id]
                pos_rev = struct.unpack_from('<f', bytes(r.data), 0)[0]
                last_pos[nid] = wrap_to_pi(pos_rev * 2 * math.pi / GEAR_RATIO)
            r = bus.recv(timeout=0.0)

        for nid in node_ids:
            s = start_pos.get(nid, 0.0)
            e = end_pos.get(nid, 0.0)
            cmd_wrapped = s + (e - s) * alpha
            cmd_raw = cmd_wrapped + turn_offsets.get(nid, 0.0)
            kp, kd = gains_by_node[nid]
            send_mit(bus, nid, cmd_raw, kp=kp, kd=kd)
            # Error tracking in wrapped space (last_pos is already wrapped from drain)
            err = abs(last_pos[nid] - cmd_wrapped)
            if err > max_err[nid]:
                max_err[nid] = err
        time.sleep(DT)

    # Hold at end position
    dwell_end = time.time() + dwell_time
    while time.time() < dwell_end:
        for nid in node_ids:
            cmd_raw = end_pos.get(nid, 0.0) + turn_offsets.get(nid, 0.0)
            kp, kd = gains_by_node[nid]
            send_mit(bus, nid, cmd_raw, kp=kp, kd=kd)
        time.sleep(DT)

    elapsed_total = time.time() - move_start
    print(f"      reached in {elapsed_total:.2f}s")

    # Final steady-state read
    actual = drain_positions(bus, node_ids, duration=0.2)
    print(f"      {'joint':<12}  {'target':>10}  {'actual':>10}  {'final_err':>10}  {'max_err':>10}")
    print(f"      {'-' * 60}")
    for nid in node_ids:
        target = end_pos.get(nid, 0.0)
        act    = actual.get(nid, float('nan'))
        # Wrap the diff so a tiny motion crossing the ±π boundary doesn't
        # appear as a ~6.28 rad error.
        ferr   = wrap_to_pi(act - target) if not math.isnan(act) else float('nan')
        merr   = max_err[nid]
        flag   = '  <-- check' if (not math.isnan(ferr) and abs(ferr) > 0.1) or merr > 0.2 else ''
        print(f"      {name_by_node[nid]:<12}  {target:>+10.4f}  {act:>+10.4f}  "
              f"{ferr:>+10.4f}  {merr:>10.4f}{flag}")


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('--leg', required=True, choices=['right', 'left'],
                        help='Which leg to test')
    parser.add_argument('--urdf', required=True,
                        help='Path to URDF for joint-limit safety check')
    parser.add_argument('--can', default=None,
                        help='Override CAN interface (default: right=can0, left=can1)')
    parser.add_argument('--kp', type=float, default=DEFAULT_KP,
                        help=f'Global default kp (default {DEFAULT_KP})')
    parser.add_argument('--kd', type=float, default=DEFAULT_KD,
                        help=f'Global default kd (default {DEFAULT_KD})')
    parser.add_argument('--gains', default=None,
                        help='Path to JSON file with per-joint kp/kd overrides '
                             '({"joint_name": {"kp": N, "kd": M}, ...}). '
                             'Missing joints fall back to --kp/--kd.')
    args = parser.parse_args()

    leg_cfg     = LEGS[args.leg]
    can_iface   = args.can or leg_cfg['can']
    joints      = leg_cfg['joints']
    node_ids    = [j[0] for j in joints]
    name_by_node     = {j[0]: j[1] for j in joints}
    urdf_name_by_node = {j[0]: j[2] for j in joints}

    # URDF limits
    try:
        urdf_limits = parse_urdf_limits(args.urdf, list(urdf_name_by_node.values()))
    except Exception as e:
        print(f"ERROR parsing URDF: {e}")
        sys.exit(1)
    limits_by_node = {nid: urdf_limits[urdf_name_by_node[nid]] for nid in node_ids}

    # Per-joint gains: start with global defaults, override from --gains JSON
    gains_by_node = {nid: (args.kp, args.kd) for nid in node_ids}
    if args.gains:
        try:
            with open(args.gains) as f:
                raw = json.load(f)
        except Exception as e:
            print(f"ERROR loading --gains file: {e}")
            sys.exit(1)
        name_to_node = {name_by_node[nid]: nid for nid in node_ids}
        for jname, vals in raw.items():
            if jname not in name_to_node:
                print(f"  WARNING: --gains has unknown joint '{jname}' (ignored)")
                continue
            nid = name_to_node[jname]
            kp_ovr = float(vals.get('kp', args.kp))
            kd_ovr = float(vals.get('kd', args.kd))
            gains_by_node[nid] = (kp_ovr, kd_ovr)

    # Banner
    print('=' * 64)
    print(f"        {args.leg.upper()} LEG MOTION TEST")
    print('=' * 64)
    print(f"CAN interface : {can_iface}")
    print(f"URDF          : {args.urdf}")
    print(f"Gear ratio    : {GEAR_RATIO}")
    print(f"Safety margin : {SAFETY_MARGIN:+.3f} rad on each URDF limit")
    print()
    print("Joints (URDF limits + gains):")
    for nid in node_ids:
        lo, hi = limits_by_node[nid]
        kp, kd = gains_by_node[nid]
        print(f"  node {nid:>2}  {name_by_node[nid]:<12} {urdf_name_by_node[nid]:<28} "
              f"[{lo:+.3f}, {hi:+.3f}] rad   kp={kp:>5.1f}  kd={kd:>4.1f}")
    print()

    bus = can.interface.Bus(channel=can_iface, interface='socketcan')

    def go_idle():
        for nid in node_ids:
            set_state(bus, nid, AXIS_STATE_IDLE)

    try:
        # ── Step 1: Enter CLOSED_LOOP, send passive MIT, sample positions ────
        print("Step 1: Entering CLOSED_LOOP and sampling encoders (kp=0 passive)...")
        for nid in node_ids:
            set_state(bus, nid, AXIS_STATE_CLOSED_LOOP)
        time.sleep(0.8)
        passive_burst(bus, node_ids, frames=30)  # ~300ms of passive MIT
        # Read raw multi-turn positions to compute per-joint turn-offset.
        # All downstream logic uses wrapped [-π, +π] positions; commands add
        # the offset back so the firmware sees a sub-2π target.
        initial_raw = drain_positions(bus, node_ids, duration=0.5, wrap=False)
        turn_offsets = compute_turn_offsets(initial_raw)
        initial = {nid: wrap_to_pi(p) for nid, p in initial_raw.items()}
        if turn_offsets:
            print("  Turn offsets (will be added to MIT commands):")
            for nid in node_ids:
                off = turn_offsets.get(nid)
                if off is not None and abs(off) > 1e-6:
                    print(f"    {name_by_node[nid]:<12} (node {nid}): "
                          f"{off:+.4f} rad  ({off / (2 * math.pi):+.2f} × 2π)")

        # ── Step 2: Initial range check ──────────────────────────────────────
        print("\nStep 2: Initial position range check")
        ok = check_within_limits(initial, limits_by_node, name_by_node,
                                 label='initial positions')
        if not ok:
            print()
            print("  >>> ABORTING: one or more joints out of URDF limits "
                  "(or no encoder reading). Move them within limits and rerun.")
            go_idle()
            return

        # ── Step 3: Capture TARGET pose ──────────────────────────────────────
        print("\nStep 3: Pose the leg to the DESIRED TARGET position.")
        target_pos = capture_pose(bus, node_ids, name_by_node, limits_by_node,
                                  prompt='record target')

        print("  Recorded target positions:")
        for nid in node_ids:
            print(f"    {name_by_node[nid]:<12} (node {nid}): {target_pos[nid]:+.4f} rad")

        ok = check_within_limits(target_pos, limits_by_node, name_by_node,
                                 label='target positions')
        if not ok:
            print("  >>> ABORTING: target outside URDF limits.")
            go_idle()
            return

        # ── Step 4: Capture HANGING pose ─────────────────────────────────────
        print("\nStep 4: Move the leg to its HANGING position (straight down).")
        hanging_pos = capture_pose(bus, node_ids, name_by_node, limits_by_node,
                                   prompt='record hanging')

        print("  Recorded hanging positions:")
        for nid in node_ids:
            print(f"    {name_by_node[nid]:<12} (node {nid}): {hanging_pos[nid]:+.4f} rad")

        ok = check_within_limits(hanging_pos, limits_by_node, name_by_node,
                                 label='hanging positions')
        if not ok:
            print("  >>> ABORTING: hanging position outside URDF limits.")
            go_idle()
            return

        # ── Step 5: Range-check the zero pose too ────────────────────────────
        zero_pos = {nid: 0.0 for nid in node_ids}
        # (Zero is inside any joint limits that bracket 0, but warn if not.)
        for nid in node_ids:
            lo, hi = limits_by_node[nid]
            if not (lo - SAFETY_MARGIN <= 0.0 <= hi + SAFETY_MARGIN):
                print(f"\n  >>> ABORTING: zero pose is outside the limits for "
                      f"{name_by_node[nid]} (node {nid}): [{lo:+.3f}, {hi:+.3f}]")
                go_idle()
                return

        # ── Step 6: Motion sequence ──────────────────────────────────────────
        print("\nStep 5: Executing motion sequence...")
        smooth_move(bus, node_ids, name_by_node, hanging_pos, zero_pos,
                    gains_by_node, turn_offsets,
                    label=f"Move 1/3 — Hanging → Zero  ({MOVE_TIME:.0f}s)")
        smooth_move(bus, node_ids, name_by_node, zero_pos, target_pos,
                    gains_by_node, turn_offsets,
                    label=f"Move 2/3 — Zero → Target   ({MOVE_TIME:.0f}s)")
        smooth_move(bus, node_ids, name_by_node, target_pos, zero_pos,
                    gains_by_node, turn_offsets,
                    label=f"Move 3/3 — Target → Zero   ({MOVE_TIME:.0f}s)")

        # ── Done ─────────────────────────────────────────────────────────────
        print("\nStep 6: Motion complete. Setting all joints to IDLE.")
        go_idle()
        print()
        print('=' * 64)
        print("  Test complete.")
        print('=' * 64)

    except KeyboardInterrupt:
        print("\n\nAborted — setting all joints to IDLE.")
        go_idle()
    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()
