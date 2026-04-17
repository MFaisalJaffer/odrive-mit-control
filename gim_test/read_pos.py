#!/usr/bin/env python3
"""
Put actuator in closed-loop state and print pos_estimate continuously.

Usage: python3 read_pos.py [can_interface] [node_id] [gear_ratio]
"""

import can
import math
import struct
import sys
import time

CAN_IFACE  = sys.argv[1] if len(sys.argv) > 1 else 'can0'
NODE_ID    = int(sys.argv[2])   if len(sys.argv) > 2 else 1
GEAR_RATIO = float(sys.argv[3]) if len(sys.argv) > 3 else 8.0

CMD_SET_STATE = 0x007
CMD_ENC_EST   = 0x009
AXIS_STATE_CLOSED_LOOP = 8
AXIS_STATE_IDLE        = 1

def can_id(cmd): return (NODE_ID << 5) | cmd

bus = can.interface.Bus(channel=CAN_IFACE, interface='socketcan')

try:
    bus.send(can.Message(arbitration_id=can_id(CMD_SET_STATE),
                         data=struct.pack('<I', AXIS_STATE_IDLE),
                         is_extended_id=False))
    time.sleep(0.5)
    print(f"Opened {CAN_IFACE} node_id={NODE_ID} gear_ratio={GEAR_RATIO}")
    print("Press Ctrl+C to stop\n")

    while True:
        r = bus.recv(timeout=0.1)
        if r and r.arbitration_id == can_id(CMD_ENC_EST):
            pos = struct.unpack_from('<f', bytes(r.data), 0)[0]
            vel = struct.unpack_from('<f', bytes(r.data), 4)[0]
            out = pos * 2 * math.pi / GEAR_RATIO
            print(f"  pos_estimate = {pos:+.6f} rev   output = {out:+.6f} rad   vel = {vel:+.4f} rev/s",
                  end='\r')

except KeyboardInterrupt:
    print("\nDone.")
finally:
    bus.shutdown()
