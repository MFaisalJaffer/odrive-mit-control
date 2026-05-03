#!/usr/bin/env python3
"""
Scan CAN bus for ODrive/GIM actuators by listening for heartbeat frames.

ODrive CAN Simple heartbeat: cmd_id=0x001, CAN ID = (node_id << 5) | 0x001
Listens for a configurable duration and reports all unique node IDs found.

Usage: python3 scan_can_nodes.py [can_interface] [duration_seconds]
"""

import can
import struct
import sys
import time
from collections import defaultdict

CAN_IFACE = sys.argv[1] if len(sys.argv) > 1 else 'can0'
DURATION  = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

CMD_HEARTBEAT = 0x001
CMD_ENC_EST   = 0x009

# Heartbeat axis states
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

def parse_node_id(arbitration_id):
    return arbitration_id >> 5

def parse_cmd_id(arbitration_id):
    return arbitration_id & 0x1F

def parse_heartbeat(data):
    if len(data) < 8:
        return None, None
    axis_error = struct.unpack_from('<I', bytes(data), 0)[0]
    axis_state = data[4]
    return axis_error, axis_state


print(f"Scanning {CAN_IFACE} for {DURATION}s...\n")

bus = can.interface.Bus(channel=CAN_IFACE, interface='socketcan')

nodes = {}       # node_id → {state, error, count, last_seen}
enc_nodes = {}   # node_id → last pos_estimate

deadline = time.time() + DURATION

try:
    while time.time() < deadline:
        remaining = deadline - time.time()
        r = bus.recv(timeout=min(0.1, remaining))
        if not r:
            continue

        node_id = parse_node_id(r.arbitration_id)
        cmd_id  = parse_cmd_id(r.arbitration_id)

        if cmd_id == CMD_HEARTBEAT:
            axis_error, axis_state = parse_heartbeat(r.data)
            if node_id not in nodes:
                nodes[node_id] = {'state': axis_state, 'error': axis_error, 'count': 0}
                state_str = AXIS_STATES.get(axis_state, f'UNKNOWN({axis_state})')
                print(f"  [+] Found node_id={node_id}  state={state_str}  error=0x{axis_error:08X}")
            nodes[node_id]['count'] += 1
            nodes[node_id]['state'] = axis_state
            nodes[node_id]['error'] = axis_error

        elif cmd_id == CMD_ENC_EST and len(r.data) >= 8:
            pos = struct.unpack_from('<f', bytes(r.data), 0)[0]
            enc_nodes[node_id] = pos

finally:
    bus.shutdown()

print(f"\n{'─'*60}")
print(f"{'SCAN RESULTS':^60}")
print(f"{'─'*60}")

if not nodes:
    print("  No ODrive/GIM nodes found on the bus.")
    print("  Check: motors powered on, CAN termination, baud rate (1Mbps)")
else:
    print(f"  Found {len(nodes)} node(s):\n")
    for node_id in sorted(nodes.keys()):
        info = nodes[node_id]
        state_str = AXIS_STATES.get(info['state'], f"UNKNOWN({info['state']})")
        error_str = f"0x{info['error']:08X}" if info['error'] else "none"
        pos_str = f"{enc_nodes[node_id]:+.4f} rev" if node_id in enc_nodes else "no encoder broadcast"
        print(f"  node_id = {node_id}")
        print(f"    state     : {state_str}")
        print(f"    errors    : {error_str}")
        print(f"    pos_est   : {pos_str}")
        print(f"    heartbeats: {info['count']} in {DURATION:.0f}s")
        print()

print(f"{'─'*60}")
