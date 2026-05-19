#!/usr/bin/env python3
"""
Scan all CAN interfaces for ODrive/GIM actuator nodes.

Listens for heartbeat frames on each interface and reports node IDs,
axis state, errors, and position estimate per interface.

After discovery, each node is briefly put into closed-loop control to
fetch a true position reading, then returned to IDLE.

Usage:
  python3 scan_all_can.py [duration_seconds] [can_interface ...]

  python3 scan_all_can.py              # scan can0 + can1 for 5s each
  python3 scan_all_can.py 3            # scan can0 + can1 for 3s each
  python3 scan_all_can.py 5 can0 can2  # scan specific interfaces
"""

import can
import math
import struct
import sys
import time

DURATION     = float(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].replace('.','').isdigit() else 5.0
IFACES       = sys.argv[2:] if len(sys.argv) > 2 else ['can0', 'can1']
GEAR_RATIO   = 8.0

CMD_HEARTBEAT = 0x001
CMD_SET_STATE = 0x007
CMD_ENC_EST   = 0x009

AXIS_STATE_IDLE        = 1
AXIS_STATE_CLOSED_LOOP = 8

AXIS_STATES = {
    0: 'UNDEFINED',
    1: 'IDLE',
    2: 'STARTUP_SEQUENCE',
    3: 'FULL_CALIBRATION_SEQUENCE',
    4: 'MOTOR_CALIBRATION',
    6: 'ENCODER_INDEX_SEARCH',
    7: 'ENCODER_OFFSET_CALIBRATION',
    8: 'CLOSED_LOOP_CONTROL',
    9: 'LOCKIN_SPIN',
    10: 'ENCODER_DIR_FIND',
    11: 'HOMING',
    12: 'ENCODER_HALL_POLARITY_CALIBRATION',
    13: 'ENCODER_HALL_PHASE_CALIBRATION',
}

def can_id(node_id, cmd): return (node_id << 5) | cmd
def parse_node_id(arb_id): return arb_id >> 5
def parse_cmd_id(arb_id):  return arb_id & 0x1F

def send(bus, node_id, cmd, data):
    bus.send(can.Message(arbitration_id=can_id(node_id, cmd), data=data, is_extended_id=False))

def fetch_true_position(bus, node_id):
    """Listen for the encoder broadcast in IDLE — do NOT enter closed-loop.

    Entering CLOSED_LOOP triggers the firmware lockin, which spins the rotor
    ~2.68 turns and offsets pos_estimate by ~2.11 rad output (gear 8). The
    encoder broadcast is emitted in IDLE on this firmware, so we can read
    pos_estimate passively without disturbing the actuator."""
    pos = None
    enc_id = can_id(node_id, CMD_ENC_EST)
    deadline = time.time() + 0.5
    while time.time() < deadline:
        r = bus.recv(timeout=0.05)
        if r and r.arbitration_id == enc_id and len(r.data) >= 8:
            pos = struct.unpack_from('<f', bytes(r.data), 0)[0]
            break
    return pos

def scan(iface, duration):
    try:
        bus = can.interface.Bus(channel=iface, interface='socketcan')
    except Exception as e:
        return None, str(e)

    nodes   = {}
    deadline = time.time() + duration

    try:
        while time.time() < deadline:
            remaining = deadline - time.time()
            r = bus.recv(timeout=min(0.1, remaining))
            if not r:
                continue
            node_id = parse_node_id(r.arbitration_id)
            cmd_id  = parse_cmd_id(r.arbitration_id)

            if cmd_id == CMD_HEARTBEAT and len(r.data) >= 8:
                axis_error = struct.unpack_from('<I', bytes(r.data), 0)[0]
                axis_state = r.data[4]
                if node_id not in nodes:
                    nodes[node_id] = {'state': axis_state, 'error': axis_error, 'count': 0}
                nodes[node_id]['count'] += 1
                nodes[node_id]['state'] = axis_state
                nodes[node_id]['error'] = axis_error

        # Fetch true positions for all discovered nodes
        true_pos = {}
        for node_id in sorted(nodes.keys()):
            pos_rev = fetch_true_position(bus, node_id)
            true_pos[node_id] = pos_rev
    finally:
        bus.shutdown()

    return nodes, true_pos


# ── Main ──────────────────────────────────────────────────────────────────────
print(f"Scanning {len(IFACES)} interface(s) for {DURATION:.0f}s each: {', '.join(IFACES)}\n")

results = {}
for iface in IFACES:
    print(f"  Scanning {iface}...", end=' ', flush=True)
    nodes, true_pos = scan(iface, DURATION)
    results[iface] = (nodes, true_pos)
    if nodes is None:
        print(f"ERROR — {true_pos}")
    else:
        print(f"found {len(nodes)} node(s)")

print()
print("═" * 60)
print(f"{'SCAN RESULTS':^60}")
print("═" * 60)

total = 0
for iface in IFACES:
    nodes, true_pos = results[iface]
    print(f"\n  ┌─ {iface} {'─' * (54 - len(iface))}")
    if nodes is None:
        print(f"  │  ERROR: {true_pos}")
    elif not nodes:
        print(f"  │  No nodes found.")
        print(f"  │  Check: motors powered on, CAN termination, baud rate")
    else:
        for node_id in sorted(nodes.keys()):
            info      = nodes[node_id]
            state_str = AXIS_STATES.get(info['state'], f"UNKNOWN({info['state']})")
            error_str = f"0x{info['error']:08X}" if info['error'] else "none"
            pos_rev   = true_pos.get(node_id)
            pos_str   = f"{pos_rev * 2 * math.pi / GEAR_RATIO:+.4f} rad" if pos_rev is not None else "no reading"
            print(f"  │  node {node_id:>2}  state={state_str:<22} error={error_str}  pos={pos_str}  hb={info['count']}/{DURATION:.0f}s")
            total += 1
    print(f"  └{'─' * 57}")

print(f"\n  Total nodes found: {total} across {len(IFACES)} interface(s)")
print("═" * 60)
