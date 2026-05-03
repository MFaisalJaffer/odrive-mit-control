#!/usr/bin/env python3
"""
Left leg motion test — pose capture then smooth playback.

Workflow:
  1. All joints go IDLE (free to move by hand).
  2. User poses the leg to the desired target position → ENTER → positions recorded.
  3. User moves leg to hanging position (straight down) → ENTER.
  4. Script enters closed-loop and executes three smooth 3-second moves:
       • Current (hanging) → Zero (0 rad on all joints)
       • Zero              → Desired target
       • Desired target    → Zero
  5. All joints return to IDLE.

Usage:
  python3 leg_test.py [can_interface]

  Default CAN interface: can0
  Default nodes: 13=hip_pitch, 14=hip_roll, 15=hip_yaw, 16=knee, 17=ankle
"""

import can
import math
import select
import struct
import sys
import time

# ── Config ────────────────────────────────────────────────────────────────────
CAN_IFACE  = sys.argv[1] if len(sys.argv) > 1 else 'can0'
GEAR_RATIO = 8.0
MOVE_TIME  = 3.0   # seconds per move
DWELL_TIME = 1.0   # seconds to hold at each waypoint
DT         = 0.01  # 100 Hz loop

# Left leg joints: node_id → label
JOINTS = {
    13: 'hip_pitch',
    14: 'hip_roll',
    15: 'hip_yaw',
    16: 'knee',
    17: 'ankle',
}
NODE_IDS = sorted(JOINTS.keys())

# ── ODrive CAN Simple constants ───────────────────────────────────────────────
CMD_ENC_EST   = 0x009
CMD_SET_STATE = 0x007
CMD_MIT       = 0x008

AXIS_STATE_IDLE        = 1
AXIS_STATE_CLOSED_LOOP = 8

# MIT bitfield ranges
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


def set_all_state(bus, state):
    for nid in NODE_IDS:
        set_state(bus, nid, state)


def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    return int((x - x_min) / (x_max - x_min) * ((1 << bits) - 1))


def send_mit(bus, node_id, pos_rad, vel=0.0, kp=150.0, kd=2.0, torque=0.0):
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


def drain_encoders(bus, duration=0.3):
    """Drain the bus for `duration` seconds, returning latest pos per node (rad)."""
    positions = {}
    deadline = time.time() + duration
    while time.time() < deadline:
        r = bus.recv(timeout=0.02)
        if not r:
            continue
        for nid in NODE_IDS:
            if r.arbitration_id == can_id(nid, CMD_ENC_EST) and len(r.data) >= 8:
                pos_rev = struct.unpack_from('<f', bytes(r.data), 0)[0]
                positions[nid] = pos_rev * 2 * math.pi / GEAR_RATIO
    return positions


def read_all_positions(bus, timeout=2.0):
    """Read one encoder sample for every joint. Returns dict node_id → rad."""
    positions = {}
    deadline = time.time() + timeout
    while time.time() < deadline and len(positions) < len(NODE_IDS):
        r = bus.recv(timeout=0.05)
        if not r:
            continue
        for nid in NODE_IDS:
            if nid not in positions and r.arbitration_id == can_id(nid, CMD_ENC_EST):
                pos_rev = struct.unpack_from('<f', bytes(r.data), 0)[0]
                positions[nid] = pos_rev * 2 * math.pi / GEAR_RATIO
    return positions


def smoothstep(t):
    """Smooth ease-in/ease-out: t ∈ [0, 1] → [0, 1]."""
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def smooth_move(bus, start_pos, end_pos, move_time=MOVE_TIME, dwell_time=DWELL_TIME,
                kp=150.0, kd=2.0, label=''):
    """
    Interpolate all joints from start_pos to end_pos over move_time seconds,
    then dwell at end_pos for dwell_time seconds.

    start_pos / end_pos: dict {node_id: rad}
    """
    print(f"\n  ▶ {label}")
    for nid in NODE_IDS:
        s = start_pos.get(nid, 0.0)
        e = end_pos.get(nid, 0.0)
        print(f"      {JOINTS[nid]:12s} (node {nid}):  {s:+.4f} → {e:+.4f} rad")

    move_start = time.time()
    while True:
        elapsed = time.time() - move_start
        if elapsed >= move_time:
            break
        alpha = smoothstep(elapsed / move_time)
        for nid in NODE_IDS:
            s = start_pos.get(nid, 0.0)
            e = end_pos.get(nid, 0.0)
            cmd = s + (e - s) * alpha
            send_mit(bus, nid, cmd, kp=kp, kd=kd)
        time.sleep(DT)

    # Hold at end position
    dwell_end = time.time() + dwell_time
    while time.time() < dwell_end:
        for nid in NODE_IDS:
            send_mit(bus, nid, end_pos.get(nid, 0.0), kp=kp, kd=kd)
        time.sleep(DT)

    print(f"      reached in {time.time() - move_start:.2f}s")


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    print("=" * 60)
    print("           LEFT LEG MOTION TEST")
    print("=" * 60)
    print(f"CAN interface : {CAN_IFACE}")
    print(f"Joints        : {', '.join(f'{JOINTS[n]} (node {n})' for n in NODE_IDS)}")
    print(f"Gear ratio    : {GEAR_RATIO}")
    print()

    bus = can.interface.Bus(channel=CAN_IFACE, interface='socketcan')

    try:
        # ── Step 1: IDLE all joints so user can pose the leg ─────────────────
        print("Step 1: Setting all joints to IDLE — leg is now free to move.")
        set_all_state(bus, AXIS_STATE_IDLE)
        time.sleep(0.5)

        print()
        print("  Pose the leg to the DESIRED TARGET position.")
        print("  Live joint positions (rad):\n")

        # Live display until ENTER
        live_pos = {}
        enc_ids = {can_id(nid, CMD_ENC_EST): nid for nid in NODE_IDS}

        while True:
            r = bus.recv(timeout=0.02)
            if r and r.arbitration_id in enc_ids:
                nid = enc_ids[r.arbitration_id]
                pos_rev = struct.unpack_from('<f', bytes(r.data), 0)[0]
                live_pos[nid] = pos_rev * 2 * math.pi / GEAR_RATIO

            if live_pos:
                line = '  '
                for nid in NODE_IDS:
                    rad = live_pos.get(nid, 0.0)
                    line += f"{JOINTS[nid]}={rad:+.4f}  "
                print(line + '  [ENTER to record]', end='\r')

            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                break

        print()
        print()

        # Record target positions — use latest live readings
        target_pos = {nid: live_pos.get(nid, 0.0) for nid in NODE_IDS}
        print("  Recorded target positions:")
        for nid in NODE_IDS:
            print(f"    {JOINTS[nid]:12s} (node {nid}): {target_pos[nid]:+.4f} rad")

        # ── Step 2: User moves leg to hanging position ────────────────────────
        print()
        print("Step 2: Move the leg to its HANGING position (straight down).")
        print("  Live joint positions:\n")

        live_pos = {}
        while True:
            r = bus.recv(timeout=0.02)
            if r and r.arbitration_id in enc_ids:
                nid = enc_ids[r.arbitration_id]
                pos_rev = struct.unpack_from('<f', bytes(r.data), 0)[0]
                live_pos[nid] = pos_rev * 2 * math.pi / GEAR_RATIO

            if live_pos:
                line = '  '
                for nid in NODE_IDS:
                    rad = live_pos.get(nid, 0.0)
                    line += f"{JOINTS[nid]}={rad:+.4f}  "
                print(line + '  [ENTER when ready]', end='\r')

            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                break

        print()
        print()

        # ── Step 3: Enter closed-loop and read current positions ──────────────
        print("Step 3: Entering closed-loop control...")
        set_all_state(bus, AXIS_STATE_CLOSED_LOOP)
        time.sleep(1.0)

        # Drain encoder frames to get fresh readings
        hanging_pos = drain_encoders(bus, duration=0.5)
        # Fill any missing with 0
        for nid in NODE_IDS:
            if nid not in hanging_pos:
                hanging_pos[nid] = 0.0

        print("  Hanging position readings:")
        for nid in NODE_IDS:
            print(f"    {JOINTS[nid]:12s} (node {nid}): {hanging_pos[nid]:+.4f} rad")

        zero_pos = {nid: 0.0 for nid in NODE_IDS}

        # ── Step 4: Three smooth moves ────────────────────────────────────────
        print()
        print("Step 4: Executing motion sequence...")

        # Move 1: Hanging → Zero
        smooth_move(bus, hanging_pos, zero_pos,
                    label=f'Move 1/3 — Hanging → Zero  ({MOVE_TIME:.0f}s)')

        # Move 2: Zero → Target
        smooth_move(bus, zero_pos, target_pos,
                    label=f'Move 2/3 — Zero → Target   ({MOVE_TIME:.0f}s)')

        # Move 3: Target → Zero
        smooth_move(bus, target_pos, zero_pos,
                    label=f'Move 3/3 — Target → Zero   ({MOVE_TIME:.0f}s)')

        # ── Done ──────────────────────────────────────────────────────────────
        print()
        print("Step 5: Motion complete. Setting all joints to IDLE.")
        set_all_state(bus, AXIS_STATE_IDLE)

        print()
        print("=" * 60)
        print("  Test complete.")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n\nAborted — setting all joints to IDLE.")
        set_all_state(bus, AXIS_STATE_IDLE)

    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()
