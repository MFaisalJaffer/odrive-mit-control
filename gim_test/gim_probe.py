#!/usr/bin/env python3
"""
Probe for GIM protocol response by sending Retrieve Configuration (0x84)
to a few candidate CAN IDs and dumping ALL raw CAN traffic for 100ms after each.

This tells us:
  1. Whether the motor reacts at all to GIM config queries while in MIT mode
  2. What the motor's GIM CAN ID is (if it responds)

Usage: python3 gim_probe.py [can_interface]
"""

import can
import sys
import time

CAN_IFACE = sys.argv[1] if len(sys.argv) > 1 else 'can0'

# Candidates for the motor's GIM CAN ID
PROBE_IDS = [0x001, 0x002, 0x011, 0x061, 0x141, 0x7FF]

# GIM Retrieve Configuration for ConfID 0x1C (Protocol over CAN)
# If the motor is actually in GIM mode, this will tell us 0=GIM or 1=MIT
PAYLOAD_PROTOCOL  = bytes([0x84, 0x00, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00])
# Also try Zero Position (ConfID 0x14)
PAYLOAD_ZERO_POS  = bytes([0x84, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00])
# Also try Get Version (0xB1) — simplest possible command, no args
PAYLOAD_GET_VER   = bytes([0xB1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

LISTEN_SEC = 0.1  # 100ms listen window after each send

def probe(bus, can_id, payload, label):
    print(f"\n--- Sending {label} to CAN ID 0x{can_id:03X} ---")
    msg = can.Message(arbitration_id=can_id, data=payload, is_extended_id=False)
    bus.send(msg)
    print(f"  Sent: {payload.hex()}")
    print(f"  Listening for {int(LISTEN_SEC*1000)}ms...")

    deadline = time.time() + LISTEN_SEC
    got_any = False
    while time.time() < deadline:
        resp = bus.recv(timeout=LISTEN_SEC)
        if resp:
            got_any = True
            print(f"  RX  id=0x{resp.arbitration_id:03X}  data={bytes(resp.data).hex()}  len={resp.dlc}")
    if not got_any:
        print("  (no frames received)")

def main():
    bus = can.interface.Bus(channel=CAN_IFACE, interface='socketcan')
    print(f"Opened {CAN_IFACE}")
    print("Note: MIT heartbeat/encoder frames will also appear — look for non-MIT IDs responding\n")

    try:
        for can_id in PROBE_IDS:
            probe(bus, can_id, PAYLOAD_GET_VER,  "Get Version (0xB1)")
            probe(bus, can_id, PAYLOAD_PROTOCOL, "Retrieve Config ConfID=0x1C (Protocol)")
            probe(bus, can_id, PAYLOAD_ZERO_POS, "Retrieve Config ConfID=0x14 (Zero Pos)")
    finally:
        bus.shutdown()

if __name__ == '__main__':
    main()
