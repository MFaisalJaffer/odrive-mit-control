#!/usr/bin/env python3
"""
Calibrate a joint's zero position by aligning the encoder's index_offset
with the URDF joint frame.

Workflow:
  1. Physically move the joint to a known position in the URDF frame
     (e.g. upper limit, lower limit, or zero)
  2. Run this script — it reads the raw encoder position and computes:
       index_offset = raw_pos_rotor - (reference_urdf_rad * gear_ratio / 2pi)
  3. Writes index_offset + use_index_offset=True, saves, reboots
  4. After reboot, /joint_states will read reference_urdf_rad at this physical position

Usage:
  python3 urdf_calibrate_joint.py \\
      --urdf   /path/to/robot.urdf \\
      --joint  dof_right_knee_04 \\
      --ref    upper              \\   # 'upper', 'lower', 'zero', or a float in rad
      --can    can0               \\
      --node   1

Arguments:
  --urdf   Path to URDF file
  --joint  Joint name as it appears in the URDF
  --ref    Current physical position expressed in URDF frame.
           Use 'upper', 'lower', 'zero', or a numeric value in radians.
  --can    CAN interface (default: can0)
  --node   CAN node ID of the actuator (default: 1)
"""

import argparse
import can
import math
import struct
import sys
import time
import xml.etree.ElementTree as ET

# ── ODrive CAN Simple constants ───────────────────────────────────────────────
CMD_ENC_EST   = 0x009
CMD_RXSDO     = 0x004
CMD_TXSDO     = 0x005
CMD_SET_STATE = 0x007
CMD_REBOOT    = 0x016
CMD_SAVE_CFG  = 0x01F

AXIS_STATE_IDLE        = 1
AXIS_STATE_CLOSED_LOOP = 8

# GIM firmware v0.5.14 endpoints
EP_INDEX_OFFSET     = 362
EP_USE_INDEX_OFFSET = 363


# ── URDF parsing ──────────────────────────────────────────────────────────────
def parse_urdf_joint(urdf_path, joint_name):
    """Return (gear_ratio, lower_limit, upper_limit) for the named joint."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    joint_el = None
    for j in root.iter('joint'):
        if j.get('name') == joint_name:
            joint_el = j
            break

    if joint_el is None:
        raise ValueError(f"Joint '{joint_name}' not found in {urdf_path}")

    limit_el = joint_el.find('limit')
    if limit_el is None:
        raise ValueError(f"Joint '{joint_name}' has no <limit> element")

    lower = float(limit_el.get('lower', '0'))
    upper = float(limit_el.get('upper', '0'))

    # gear_ratio may appear as a ros2_control joint param or a custom attribute.
    # Check ros2_control section first, then fall back to 1.0.
    gear_ratio = 1.0
    for rc in root.iter('ros2_control'):
        for jnt in rc.iter('joint'):
            if jnt.get('name') == joint_name:
                for param in jnt.iter('param'):
                    if param.get('name') == 'gear_ratio':
                        gear_ratio = float(param.text)

    return gear_ratio, lower, upper


def resolve_reference(ref_str, lower, upper):
    """Resolve --ref argument to a float in radians."""
    if ref_str == 'upper':
        return upper
    if ref_str == 'lower':
        return lower
    if ref_str == 'zero':
        return 0.0
    try:
        return float(ref_str)
    except ValueError:
        raise ValueError(f"--ref must be 'upper', 'lower', 'zero', or a number. Got: {ref_str}")


# ── CAN helpers (same approach as odrive_set_offset.py) ──────────────────────
def make_can_id(node_id, cmd):
    return (node_id << 5) | cmd


def send(bus, node_id, cmd, data):
    bus.send(can.Message(
        arbitration_id=make_can_id(node_id, cmd),
        data=data,
        is_extended_id=False,
    ))


def read_pos(bus, node_id, timeout=2.0):
    """Read encoder pos_estimate, skipping zero frames (motor broadcasts zeros when idle)."""
    enc_id = make_can_id(node_id, CMD_ENC_EST)
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        r = bus.recv(timeout=0.05)
        if r and r.arbitration_id == enc_id:
            pos = struct.unpack_from('<f', bytes(r.data), 0)[0]
            vel = struct.unpack_from('<f', bytes(r.data), 4)[0]
            last = (pos, vel)
            if abs(pos) > 1e-4:
                return pos, vel
    return last if last else (None, None)


def wait_txsdo(bus, node_id, ep, timeout=0.5):
    """Wait for TxSdo acknowledgment. GIM firmware typically does not respond."""
    txsdo_id = make_can_id(node_id, CMD_TXSDO)
    deadline = time.time() + timeout
    while time.time() < deadline:
        r = bus.recv(timeout=0.1)
        if r and r.arbitration_id == txsdo_id and len(r.data) >= 8:
            ep_echo = struct.unpack_from('<H', bytes(r.data), 1)[0]
            if ep_echo == ep:
                return bytes(r.data)
    return None


def sdo_write_float(bus, node_id, ep, value):
    payload = struct.pack('<BHBf', 1, ep, 0, value)
    send(bus, node_id, CMD_RXSDO, payload)
    reply = wait_txsdo(bus, node_id, ep)
    if reply:
        echoed = struct.unpack_from('<f', reply, 4)[0]
        ok = abs(echoed - value) < 1e-4
        print(f"    TxSdo: ep={ep} echoed={echoed:.6f} {'OK' if ok else 'MISMATCH'}")
    else:
        print("    No TxSdo reply (expected for GIM firmware)")


def sdo_write_bool(bus, node_id, ep, value):
    payload = struct.pack('<BHBB3x', 1, ep, 0, int(value))
    send(bus, node_id, CMD_RXSDO, payload)
    reply = wait_txsdo(bus, node_id, ep)
    if reply:
        echoed = bool(reply[4])
        ok = echoed == bool(value)
        print(f"    TxSdo: ep={ep} echoed={echoed} {'OK' if ok else 'MISMATCH'}")
    else:
        print("    No TxSdo reply (expected for GIM firmware)")


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--urdf',  required=True, help='Path to URDF file')
    parser.add_argument('--joint', required=True, help='Joint name in URDF')
    parser.add_argument('--ref',   required=True,
                        help="Current physical position in URDF frame: "
                             "'upper', 'lower', 'zero', or a float in rad")
    parser.add_argument('--can',   default='can0', help='CAN interface (default: can0)')
    parser.add_argument('--node',  type=int, default=1, help='CAN node ID (default: 1)')
    args = parser.parse_args()

    # ── Parse URDF ────────────────────────────────────────────────────────────
    print(f"Parsing URDF: {args.urdf}")
    gear_ratio, lower, upper = parse_urdf_joint(args.urdf, args.joint)
    ref_rad = resolve_reference(args.ref, lower, upper)

    print(f"  Joint:      {args.joint}")
    print(f"  Limits:     lower={lower:.6f} rad  upper={upper:.6f} rad")
    print(f"  Gear ratio: {gear_ratio}")
    print(f"  Reference:  {ref_rad:.6f} rad (--ref '{args.ref}')")

    if not (lower - 1e-6 <= ref_rad <= upper + 1e-6):
        print(f"ERROR: reference {ref_rad:.6f} is outside joint limits [{lower:.6f}, {upper:.6f}]")
        sys.exit(1)

    print()
    print(f">>> Move joint '{args.joint}' to {ref_rad:.6f} rad in URDF frame, then press ENTER.")
    input("    (Press ENTER when in position)")
    print()

    # ── CAN setup ─────────────────────────────────────────────────────────────
    bus = can.interface.Bus(channel=args.can, interface='socketcan')
    print(f"Opened {args.can}, node_id={args.node}\n")

    try:
        # Step 1: Clear existing offset and reboot for fresh MA732 absolute reading
        print("Step 1: Clearing offset and rebooting to get true absolute encoder reading...")
        sdo_write_float(bus, args.node, EP_INDEX_OFFSET, 0.0)
        sdo_write_bool(bus, args.node, EP_USE_INDEX_OFFSET, False)
        send(bus, args.node, CMD_SAVE_CFG, bytes(8))
        time.sleep(0.5)
        send(bus, args.node, CMD_REBOOT, bytes([0]))
        print("  Rebooting... waiting 4 seconds...")
        time.sleep(4.0)

        # Step 2: Enter closed-loop then idle to settle encoder
        print("\nStep 2: Entering closed-loop then idle to settle encoder...")
        send(bus, args.node, CMD_SET_STATE, struct.pack('<I', AXIS_STATE_CLOSED_LOOP))
        time.sleep(1.5)
        send(bus, args.node, CMD_SET_STATE, struct.pack('<I', AXIS_STATE_IDLE))
        time.sleep(0.5)

        # Step 3: Read raw absolute pos_estimate
        print("\nStep 3: Reading raw absolute encoder position...")
        raw_pos, _ = read_pos(bus, args.node)
        if raw_pos is None:
            print("ERROR: no encoder data received. Is the motor powered on?")
            sys.exit(1)

        raw_output_rad = raw_pos * 2 * math.pi / gear_ratio
        print(f"  raw pos_estimate = {raw_pos:.6f} rotor turns  ({raw_output_rad:.6f} rad output shaft)")

        # Step 4: Compute index_offset
        # We want: pos_after_offset = ref_rad in output shaft
        # pos_after_offset (rotor turns) = raw_pos - index_offset
        # ref_rad (rotor turns)          = ref_rad * gear_ratio / (2*pi)
        ref_rotor = ref_rad * gear_ratio / (2 * math.pi)
        index_offset = raw_pos - ref_rotor

        print(f"\n  Reference in rotor turns: {ref_rotor:.6f}")
        print(f"  index_offset = raw({raw_pos:.6f}) - ref({ref_rotor:.6f}) = {index_offset:.6f}")
        print(f"\n  After calibration, {ref_rad:.6f} rad (URDF) = this physical position.")

        # Step 5: Write index_offset
        print(f"\nStep 5: Writing encoder.config.index_offset = {index_offset:.6f} (ep {EP_INDEX_OFFSET})...")
        sdo_write_float(bus, args.node, EP_INDEX_OFFSET, index_offset)

        print(f"Writing encoder.config.use_index_offset = True (ep {EP_USE_INDEX_OFFSET})...")
        sdo_write_bool(bus, args.node, EP_USE_INDEX_OFFSET, True)

        # Step 6: Save and reboot
        print("\nStep 6: Saving configuration...")
        send(bus, args.node, CMD_SAVE_CFG, bytes(8))
        time.sleep(0.5)
        print("Rebooting to apply offset...")
        send(bus, args.node, CMD_REBOOT, bytes([0]))
        print("  Waiting 4 seconds for reboot...")
        time.sleep(4.0)

        # Step 7: Verify — enter closed-loop to get encoder broadcasting, then idle
        print("\nStep 7: Verifying...")
        send(bus, args.node, CMD_SET_STATE, struct.pack('<I', AXIS_STATE_CLOSED_LOOP))
        time.sleep(1.5)
        send(bus, args.node, CMD_SET_STATE, struct.pack('<I', AXIS_STATE_IDLE))
        time.sleep(0.5)

        pos2, _ = read_pos(bus, args.node)
        if pos2 is not None:
            output_rad2 = pos2 * 2 * math.pi / gear_ratio
            error_rad = abs(output_rad2 - ref_rad)
            print(f"  pos_estimate = {pos2:.6f} rotor turns  ({output_rad2:.6f} rad output shaft)")
            print(f"  Expected:      {ref_rotor:.6f} rotor turns  ({ref_rad:.6f} rad)")
            if error_rad < 0.05:
                print(f"  SUCCESS: reads {output_rad2:.4f} rad, expected {ref_rad:.4f} rad (error={error_rad:.4f} rad)")
            else:
                print(f"  WARNING: error = {error_rad:.4f} rad — offset may not have applied correctly.")
                print(f"  Try a full power cycle (motor off/on) if the error persists.")
        else:
            print("  No encoder data after reboot.")

    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()
