#!/usr/bin/env python3
"""
Diagnose why SDO (RxSdo/TxSdo) is not responding.

Steps:
  1. Check firmware version via Get_Version (if available)
  2. Send RxSdo READ for several well-known endpoints
  3. Dump ALL raw CAN frames after each request (not filtered by node_id)
     to find if TxSdo comes back on any CAN ID

Usage: python3 sdo_probe.py [can_interface] [node_id]
"""

import can
import struct
import sys
import time

CAN_IFACE = sys.argv[1] if len(sys.argv) > 1 else 'can0'
NODE_ID   = int(sys.argv[2]) if len(sys.argv) > 2 else 1

def can_id(cmd): return (NODE_ID << 5) | cmd

CMD_RXSDO    = 0x004
CMD_TXSDO    = 0x005
CMD_HEARTBEAT = 0x001
CMD_ENC_EST   = 0x009

LISTEN_S = 0.15

def send_raw(bus, arb_id, data):
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
    bus.send(msg)

def dump_after(bus, label, arb_id, payload, duration=LISTEN_S):
    """Send a frame and dump everything received for duration seconds."""
    print(f"\n--- {label} ---")
    print(f"  TX  id=0x{arb_id:03X}  data={payload.hex()}")
    send_raw(bus, arb_id, payload)
    deadline = time.time() + duration
    count = 0
    while time.time() < deadline:
        r = bus.recv(timeout=duration)
        if r:
            cmd = r.arbitration_id & 0x1F
            nid = r.arbitration_id >> 5
            print(f"  RX  id=0x{r.arbitration_id:03X}  (nid={nid} cmd=0x{cmd:02X})  data={bytes(r.data).hex()}")
            count += 1
    if count == 0:
        print("  (no frames received)")

def sdo_read(endpoint_id):
    return struct.pack('<BHB4x', 0, endpoint_id, 0)

def main():
    bus = can.interface.Bus(channel=CAN_IFACE, interface='socketcan')
    print(f"Opened {CAN_IFACE}, node_id={NODE_ID}")
    print(f"RxSdo CAN ID: 0x{can_id(CMD_RXSDO):03X}")
    print(f"TxSdo CAN ID: 0x{can_id(CMD_TXSDO):03X}\n")

    try:
        # 1. Show current heartbeat to confirm motor is alive and get state
        print("=== Heartbeat snapshot ===")
        deadline = time.time() + 0.3
        while time.time() < deadline:
            r = bus.recv(timeout=0.3)
            if r and (r.arbitration_id & 0x1F) == CMD_HEARTBEAT:
                nid = r.arbitration_id >> 5
                d = r.data
                ax_err = struct.unpack_from('<I', bytes(d), 0)[0]
                ax_state = d[4]
                flags = d[5]
                life = d[7] if len(d) > 7 else '?'
                print(f"  Heartbeat from nid={nid}: error=0x{ax_err:08X} state={ax_state} flags=0x{flags:02X} life={life}")
                break

        # 2. Show current encoder position
        print("\n=== Encoder position snapshot ===")
        deadline = time.time() + 0.3
        while time.time() < deadline:
            r = bus.recv(timeout=0.3)
            if r and (r.arbitration_id & 0x1F) == CMD_ENC_EST:
                nid = r.arbitration_id >> 5
                pos = struct.unpack_from('<f', bytes(r.data), 0)[0]
                vel = struct.unpack_from('<f', bytes(r.data), 4)[0]
                print(f"  Encoder from nid={nid}: pos={pos:.4f} rev  vel={vel:.4f} rev/s")
                break

        # 3. Try RxSdo READ for several endpoints with ALL frames dumped
        print("\n=== SDO read probes (dumping ALL frames) ===")

        # Endpoint 336: encoder.pos_estimate (float) — v0.5.13 and v0.5.14
        dump_after(bus, "READ endpoint 336 (encoder.pos_estimate, v0.5.14)",
                   can_id(CMD_RXSDO), sdo_read(336))

        # Endpoint 335: try nearby ID in case version differs
        dump_after(bus, "READ endpoint 335",
                   can_id(CMD_RXSDO), sdo_read(335))

        # Endpoint 1: usually fw_version or vbus
        dump_after(bus, "READ endpoint 1",
                   can_id(CMD_RXSDO), sdo_read(1))

        # Endpoint 0: root object
        dump_after(bus, "READ endpoint 0",
                   can_id(CMD_RXSDO), sdo_read(0))

        # Try broadcast CAN ID (0x000) for RxSdo — some ODrive-based firmware listens on 0
        dump_after(bus, "READ endpoint 336 via broadcast CAN ID 0x000",
                   0x000, sdo_read(336))

        # Try the TxSdo ID directly as a send to see if it echoes
        dump_after(bus, "READ endpoint 336 sent to TxSdo ID (0x025) — wrong direction test",
                   can_id(CMD_TXSDO), sdo_read(336))

        # 4. Try a WRITE to a safe read-only endpoint and watch for any response
        print("\n=== SDO write probe (endpoint 336 = pos_estimate, read-only — expect error response) ===")
        write_payload = struct.pack('<BHBf', 1, 336, 0, 0.0)
        dump_after(bus, "WRITE endpoint 336 (should fail with error code)",
                   can_id(CMD_RXSDO), write_payload)

    finally:
        bus.shutdown()

if __name__ == '__main__':
    main()
