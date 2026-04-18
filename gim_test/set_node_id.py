#!/usr/bin/env python3
"""
Change the CAN node_id of an ODrive/GIM actuator via SDO write.

The new node_id is saved and takes effect after reboot.

Usage:
  python3 set_node_id.py --can can0 --current-node 1 --new-node 3

Warning:
  If you set a node_id that conflicts with another device on the bus,
  both will become unreachable. Make sure only one actuator is on the
  bus when running this script.
"""

import argparse
import can
import struct
import time

# GIM firmware v0.5.14 endpoint
EP_NODE_ID   = 179   # can_node_id (uint32, rw)

CMD_RXSDO    = 0x004
CMD_TXSDO    = 0x005
CMD_SAVE_CFG = 0x01F
CMD_REBOOT   = 0x016


def can_id(node_id, cmd):
    return (node_id << 5) | cmd


def send(bus, node_id, cmd, data):
    bus.send(can.Message(
        arbitration_id=can_id(node_id, cmd),
        data=data,
        is_extended_id=False,
    ))


def wait_txsdo(bus, node_id, ep, timeout=0.5):
    txsdo_id = can_id(node_id, CMD_TXSDO)
    deadline = time.time() + timeout
    while time.time() < deadline:
        r = bus.recv(timeout=0.1)
        if r and r.arbitration_id == txsdo_id and len(r.data) >= 8:
            ep_echo = struct.unpack_from('<H', bytes(r.data), 1)[0]
            if ep_echo == ep:
                return bytes(r.data)
    return None


def sdo_write_uint32(bus, node_id, ep, value):
    payload = struct.pack('<BHBI', 1, ep, 0, value)
    send(bus, node_id, CMD_RXSDO, payload)
    reply = wait_txsdo(bus, node_id, ep)
    if reply:
        echoed = struct.unpack_from('<I', reply, 4)[0]
        ok = echoed == value
        print(f"    TxSdo: ep={ep} echoed={echoed} {'OK' if ok else 'MISMATCH'}")
    else:
        print("    No TxSdo reply (expected for GIM firmware)")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--can',          default='can0', help='CAN interface (default: can0)')
    parser.add_argument('--current-node', type=int, required=True, help='Current node_id of the actuator')
    parser.add_argument('--new-node',     type=int, required=True, help='New node_id to assign')
    args = parser.parse_args()

    if args.new_node < 0 or args.new_node > 63:
        print("ERROR: node_id must be between 0 and 63 (6-bit CAN node ID)")
        return

    if args.current_node == args.new_node:
        print("Current and new node_id are the same — nothing to do.")
        return

    print(f"Changing node_id: {args.current_node} → {args.new_node}")
    print(f"WARNING: Only one actuator should be connected to the bus.\n")

    bus = can.interface.Bus(channel=args.can, interface='socketcan')

    try:
        # Write new node_id
        print(f"Step 1: Writing node_id = {args.new_node} (endpoint {EP_NODE_ID})...")
        sdo_write_uint32(bus, args.current_node, EP_NODE_ID, args.new_node)
        print("  Sent.")

        # Save configuration
        print("\nStep 2: Saving configuration...")
        send(bus, args.current_node, CMD_SAVE_CFG, bytes(8))
        print("  Sent.")
        time.sleep(0.5)

        # Reboot
        print("\nStep 3: Rebooting actuator...")
        send(bus, args.current_node, CMD_REBOOT, bytes([0]))
        print("  Sent. Waiting 4 seconds for reboot...")
        time.sleep(4.0)

        # Verify by checking heartbeat on new node_id
        print(f"\nStep 4: Verifying — listening for heartbeat on new node_id={args.new_node}...")
        heartbeat_id = can_id(args.new_node, 0x001)  # Heartbeat cmd_id=0x001
        deadline = time.time() + 3.0
        found = False
        while time.time() < deadline:
            r = bus.recv(timeout=0.1)
            if r and r.arbitration_id == heartbeat_id:
                print(f"  SUCCESS: heartbeat received on node_id={args.new_node} (CAN ID 0x{heartbeat_id:03X})")
                found = True
                break

        if not found:
            print(f"  WARNING: no heartbeat seen on node_id={args.new_node}.")
            print(f"  The SDO write may not have taken effect — check firmware endpoint ID.")

    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()
