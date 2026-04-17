#!/usr/bin/env python3
"""
Attempt to:
  1. Find the motor's GIM protocol CAN ID by scanning with Start Motor (0x91)
  2. Send Position Control (0x95) to move the motor slightly
  3. Stop Motor (0x92)
  4. Read Zero Position (ConfID 0x14) and Power-Off Position (ConfID 0x15)
  5. Optionally write a new Zero Position

This requires the ros2_control launch to be STOPPED first.

Usage: python3 gim_zero_set.py [can_interface]
"""

import can
import struct
import sys
import time

CAN_IFACE = sys.argv[1] if len(sys.argv) > 1 else 'can0'

# Candidate GIM CAN IDs to try
CANDIDATES = [0x001, 0x080, 0x081, 0x082, 0x089, 0x141, 0x002, 0x011]

LISTEN_S = 0.2  # 200ms listen window


def send_recv(bus, can_id, payload, expect_cmd, timeout=0.2):
    """Send a frame and return the first response matching expect_cmd, or None."""
    msg = can.Message(arbitration_id=can_id, data=payload, is_extended_id=False)
    bus.send(msg)
    deadline = time.time() + timeout
    while time.time() < deadline:
        r = bus.recv(timeout=timeout)
        if r and len(r.data) >= 2 and r.data[0] == expect_cmd:
            return r
    return None


def find_gim_id(bus):
    """Scan candidates for the motor's GIM CAN ID using Start Motor (0x91)."""
    print("Scanning for GIM CAN ID using Start Motor (0x91)...")
    payload = bytes([0x91, 0, 0, 0, 0, 0, 0, 0])
    for cid in CANDIDATES:
        print(f"  Trying 0x{cid:03X}...", end=' ', flush=True)
        r = send_recv(bus, cid, payload, expect_cmd=0x91)
        if r:
            res = r.data[1]
            print(f"RESPONSE! from 0x{r.arbitration_id:03X}, RES=0x{res:02X} ({'OK' if res==0 else 'FAIL'})")
            return cid
        else:
            print("no response")
    return None


def stop_motor(bus, cid):
    print(f"\nStopping motor (0x92) on CAN ID 0x{cid:03X}...")
    payload = bytes([0x92, 0, 0, 0, 0, 0, 0, 0])
    r = send_recv(bus, cid, payload, expect_cmd=0x92)
    if r:
        print(f"  Stop response: RES=0x{r.data[1]:02X}")
    else:
        print("  No response to stop")


def move_position(bus, cid, pos_rad, duration_ms=2000):
    """Send GIM Position Control (0x95) to pos_rad over duration_ms."""
    import struct
    pos_bytes = struct.pack('<f', pos_rad)
    dur_bytes = duration_ms.to_bytes(3, 'little')
    payload = bytes([0x95]) + pos_bytes + dur_bytes
    print(f"\nSending Position Control (0x95): {pos_rad:.3f} rad, {duration_ms}ms...")
    r = send_recv(bus, cid, payload, expect_cmd=0x95, timeout=0.5)
    if r:
        d = r.data
        res = d[1]
        if res == 0x00 and len(d) >= 4:
            pos_int = struct.unpack_from('<H', bytes(d[2:4]))[0]
            pos_float = pos_int * 25 / 65535 - 12.5
            print(f"  Response: RES=0x{res:02X}, current pos={pos_float:.4f} rad")
        else:
            print(f"  Response: RES=0x{res:02X}")
    else:
        print("  No response")


def read_config(bus, cid, conf_type, conf_id, label):
    payload = bytes([0x84, conf_type, conf_id, 0, 0, 0, 0, 0])
    r = send_recv(bus, cid, payload, expect_cmd=0x84, timeout=0.5)
    if r:
        d = r.data
        res = d[3]
        if res == 0x00 and len(d) >= 8:
            val = struct.unpack_from('<i', bytes(d[4:8]))[0]
            pos_rad = val * 2 * 3.14159265 / 65536
            print(f"  {label}: raw={val}, {pos_rad:.4f} rad")
        else:
            print(f"  {label}: RES=0x{res:02X} (failure)")
    else:
        print(f"  {label}: no response")


def write_zero_position(bus, cid, zero_rad):
    """Write ConfID 0x14 (Zero Position) in output-shaft RAD."""
    val = int(zero_rad * 65536 / (2 * 3.14159265))
    data = struct.pack('<i', val)
    payload = bytes([0x83, 0x00, 0x14, 0x00]) + data
    print(f"\nWriting Zero Position: {zero_rad:.4f} rad (raw={val})...")
    r = send_recv(bus, cid, payload, expect_cmd=0x83, timeout=0.5)
    if r:
        print(f"  Write response: RES=0x{r.data[2]:02X}")
    else:
        print("  No response to write")


def main():
    bus = can.interface.Bus(channel=CAN_IFACE, interface='socketcan')
    print(f"Opened {CAN_IFACE}\n")

    try:
        gim_id = find_gim_id(bus)

        if gim_id is None:
            print("\nNo GIM response found on any candidate ID.")
            print("Motor is likely ignoring GIM protocol while in MIT mode.")
            print("Options: use SteadyWin USB app, or use software offsets in gim_controllers.yaml")
            return

        print(f"\nGIM CAN ID found: 0x{gim_id:03X}")

        # Move to a small position to confirm it works
        move_position(bus, gim_id, pos_rad=1.0, duration_ms=2000)
        time.sleep(2.5)

        # Stop
        stop_motor(bus, gim_id)
        time.sleep(0.2)

        # Read config values
        print("\nReading configuration:")
        read_config(bus, gim_id, 0x00, 0x14, "Zero Position (0x14)")
        read_config(bus, gim_id, 0x00, 0x15, "Power-Off Position (0x15)")
        read_config(bus, gim_id, 0x00, 0x1C, "Protocol (0x1C): 0=GIM 1=MIT")
        read_config(bus, gim_id, 0x00, 0x12, "CAN ID (0x12)")

    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()
