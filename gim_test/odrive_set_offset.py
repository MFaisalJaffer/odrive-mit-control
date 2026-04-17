#!/usr/bin/env python3
"""
Set encoder.config.index_offset via silent SDO write (no TxSdo response expected).
Reads current position from encoder broadcast (0x029), writes it as index_offset,
enables use_index_offset, then saves configuration.

After save the motor reboots. Re-run to confirm position reads ~0.0.

Usage: python3 odrive_set_offset.py [can_interface] [node_id]
"""

import can
import struct
import sys
import time

CAN_IFACE = sys.argv[1] if len(sys.argv) > 1 else 'can0'
NODE_ID   = int(sys.argv[2]) if len(sys.argv) > 2 else 1

GEAR_RATIO = 8.0

def can_id(cmd): return (NODE_ID << 5) | cmd

CMD_ENC_EST   = 0x009
CMD_RXSDO     = 0x004
CMD_TXSDO     = 0x005
CMD_SET_STATE = 0x007  # Set_Axis_State
CMD_REBOOT    = 0x016  # Reboot cmd: data[0]=0 reboot, 1=save+reboot, 2=erase+reboot
CMD_SAVE_CFG  = 0x01F  # Legacy save (v0.5.x)

AXIS_STATE_IDLE         = 1
AXIS_STATE_CLOSED_LOOP  = 8

# endpoints_0.5.14.json
EP_INDEX_OFFSET     = 362  # encoder.config.index_offset (float, rw)
EP_USE_INDEX_OFFSET = 363  # encoder.config.use_index_offset (bool, rw)


def send(bus, cmd, data):
    bus.send(can.Message(arbitration_id=can_id(cmd), data=data, is_extended_id=False))


def read_pos(bus, timeout=2.0):
    """Read encoder position, skipping zero frames."""
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        r = bus.recv(timeout=0.05)
        if r and r.arbitration_id == can_id(CMD_ENC_EST):
            pos = struct.unpack_from('<f', bytes(r.data), 0)[0]
            vel = struct.unpack_from('<f', bytes(r.data), 4)[0]
            last = (pos, vel)
            if abs(pos) > 1e-4:
                return pos, vel
    return last if last else (None, None)


def wait_txsdo(bus, ep, timeout=0.5):
    """Wait for TxSdo reply and return raw data bytes, or None if no response."""
    txsdo_id = can_id(CMD_TXSDO)
    deadline = time.time() + timeout
    while time.time() < deadline:
        r = bus.recv(timeout=timeout)
        if r and r.arbitration_id == txsdo_id and len(r.data) >= 8:
            ep_echo = struct.unpack_from('<H', bytes(r.data), 1)[0]
            if ep_echo == ep:
                return bytes(r.data)
    return None


def sdo_write_float(bus, ep, value):
    payload = struct.pack('<BHBf', 1, ep, 0, value)
    send(bus, CMD_RXSDO, payload)
    reply = wait_txsdo(bus, ep)
    if reply:
        echoed = struct.unpack_from('<f', reply, 4)[0]
        match = abs(echoed - value) < 1e-4
        print(f"    TxSdo reply: ep={struct.unpack_from('<H', reply, 1)[0]} value={echoed:.6f} {'OK' if match else 'MISMATCH (sent {:.6f})'.format(value)}")
    else:
        print("    No TxSdo reply (firmware may not support SDO ack)")


def sdo_write_bool(bus, ep, value):
    payload = struct.pack('<BHBB3x', 1, ep, 0, int(value))
    send(bus, CMD_RXSDO, payload)
    reply = wait_txsdo(bus, ep)
    if reply:
        echoed = bool(reply[4])
        match = echoed == bool(value)
        print(f"    TxSdo reply: ep={struct.unpack_from('<H', reply, 1)[0]} value={echoed} {'OK' if match else 'MISMATCH'}")
    else:
        print("    No TxSdo reply (firmware may not support SDO ack)")


def main():
    bus = can.interface.Bus(channel=CAN_IFACE, interface='socketcan')
    print(f"Opened {CAN_IFACE}, node_id={NODE_ID}\n")

    try:
        # Step 1: Clear any existing offset and reboot to get true MA732 absolute reading
        # Runtime pos_estimate accumulates from movement — only a fresh reboot gives the raw absolute
        print("Step 1: Clearing offset and rebooting to get true absolute encoder position...")
        sdo_write_float(bus, EP_INDEX_OFFSET, 0.0)
        sdo_write_bool(bus, EP_USE_INDEX_OFFSET, False)
        send(bus, CMD_SAVE_CFG, bytes(8))
        time.sleep(0.5)
        send(bus, CMD_REBOOT, bytes([0]))
        print("  Rebooting... waiting 4 seconds...")
        time.sleep(4.0)

        # Step 2: Enter closed-loop then back to idle to settle encoder
        print("\nStep 2: Entering closed-loop mode then idle to settle encoder...")
        send(bus, CMD_SET_STATE, struct.pack('<I', AXIS_STATE_CLOSED_LOOP))
        time.sleep(1.5)
        send(bus, CMD_SET_STATE, struct.pack('<I', AXIS_STATE_IDLE))
        time.sleep(0.5)

        # Step 3: Read TRUE absolute position (fresh after reboot, no accumulated drift)
        print("\nStep 3: Reading true absolute encoder position after fresh reboot...")
        pos, vel = read_pos(bus)
        if pos is None:
            print("ERROR: no encoder data received. Is the motor powered on?")
            return
        output_rad = pos * 2 * 3.14159265 / GEAR_RATIO
        print(f"  pos_estimate = {pos:.6f} rev  ({output_rad:.4f} rad output shaft)")
        print(f"  This is the true MA732 absolute reading at the current shaft position.")
        print(f"  Writing index_offset = {pos:.6f} to make this position = 0\n")

        # Step 4: Write index_offset = true absolute position
        print(f"Step 4: Writing encoder.config.index_offset = {pos:.6f} (endpoint {EP_INDEX_OFFSET})...")
        sdo_write_float(bus, EP_INDEX_OFFSET, pos)
        print("  Sent.")

        # Step 4: Enable use_index_offset
        print(f"Writing encoder.config.use_index_offset = True (endpoint {EP_USE_INDEX_OFFSET})...")
        sdo_write_bool(bus, EP_USE_INDEX_OFFSET, True)
        print("  Sent.")

        # Step 5: Enable use_index_offset -- already written above, just label
        # Step 6: Save configuration
        print("\nStep 5: Saving configuration...")
        send(bus, CMD_SAVE_CFG, bytes(8))
        print("  Sent.")
        time.sleep(0.5)

        # Step 6: Reboot to apply
        print("Step 6: Rebooting to apply zero offset...")
        send(bus, CMD_REBOOT, bytes([0]))
        print("  Sent. Waiting 4 seconds for reboot...")
        time.sleep(4.0)

        # Step 6: Read position after reboot
        print("\nReading position after reboot...")
        pos2, vel2 = read_pos(bus)
        if pos2 is not None:
            output_rad2 = pos2 * 2 * 3.14159265 / GEAR_RATIO
            print(f"  pos_estimate = {pos2:.6f} rev  ({output_rad2:.4f} rad output shaft)")
            if abs(pos2) < 0.05:
                print("  SUCCESS: position is ~0.0 — zero offset applied correctly.")
            else:
                print(f"  NOTE: position is {output_rad2:.4f} rad (not zero).")
                print("  This means the SDO write was silently ignored.")
                print("  Use software offset in gim_controllers.yaml instead:")
                print(f"    offset: {output_rad:.4f}")
        else:
            print("  No encoder data after reboot.")

    finally:
        bus.shutdown()

if __name__ == '__main__':
    main()
