#!/usr/bin/env python3
"""
Set motor zero position via ODrive CAN Simple protocol.

Workflow:
  1. Clear errors
  2. Set position control mode (control_mode=3, input_mode=3)
  3. Enter closed-loop (axis_state=8)
  4. Move to target position (in output-shaft radians, converted to rotor turns)
  5. Go idle
  6. Read current encoder.pos_estimate
  7. Write it to encoder.config.index_offset  (endpoint 349)
  8. Enable encoder.config.use_index_offset   (endpoint 350)
  9. Save configuration

STOP the ros2_control launch before running this.

Usage: python3 odrive_zero_set.py [can_interface] [node_id] [target_rad]
  target_rad: desired output-shaft position to move to before zeroing (default 0.0)
              The motor will move here, then that position is declared zero.
"""

import can
import struct
import sys
import time

CAN_IFACE  = sys.argv[1] if len(sys.argv) > 1 else 'can0'
NODE_ID    = int(sys.argv[2]) if len(sys.argv) > 2 else 1
TARGET_RAD = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

GEAR_RATIO = 8.0  # output shaft gear ratio

# CAN IDs  (node_id << 5) | cmd_id
def can_id(cmd): return (NODE_ID << 5) | cmd

CMD_HEARTBEAT   = 0x001
CMD_SET_STATE   = 0x007
CMD_MIT         = 0x008
CMD_ENC_EST     = 0x009
CMD_SET_MODE    = 0x00B
CMD_SET_POS     = 0x00C
CMD_CLEAR_ERR      = 0x018
CMD_SET_LIN_COUNT  = 0x019  # Set_Linear_Count: sets encoder position to given int32
CMD_RXSDO          = 0x004
CMD_TXSDO          = 0x005
CMD_SAVE_CFG       = 0x01F

AXIS_STATE_IDLE        = 1
AXIS_STATE_CLOSED_LOOP = 8
CONTROL_MODE_POSITION  = 3
INPUT_MODE_POS_FILTER  = 3


def send(bus, cmd, data):
    msg = can.Message(arbitration_id=can_id(cmd), data=data, is_extended_id=False)
    bus.send(msg)


def recv_cmd(bus, expected_cmd, timeout=0.5):
    """Wait for a frame with the given cmd_id from our node.
    GIM firmware uses two CAN ID formulas:
      - (node_id << 5) | cmd_id  for command responses
      - (node_id << 7) | cmd_id  for periodic broadcasts
    Accept either.
    """
    alt_id_5 = (NODE_ID << 5) | expected_cmd
    alt_id_7 = (NODE_ID << 7) | expected_cmd
    deadline = time.time() + timeout
    while time.time() < deadline:
        r = bus.recv(timeout=timeout)
        if r and r.arbitration_id in (alt_id_5, alt_id_7):
            return r
    return None


def sdo_read_float(bus, endpoint_id):
    """Read a float endpoint via RxSdo → TxSdo.
    TxSdo response may be on (node_id<<5)|5 or (node_id<<7)|5.
    """
    payload = struct.pack('<BHB4x', 0, endpoint_id, 0)  # opcode=0 (read)
    send(bus, CMD_RXSDO, payload)
    # Accept TxSdo on either formula
    txsdo_5 = (NODE_ID << 5) | CMD_TXSDO
    txsdo_7 = (NODE_ID << 7) | CMD_TXSDO
    deadline = time.time() + 0.5
    while time.time() < deadline:
        r = bus.recv(timeout=0.5)
        if r and r.arbitration_id in (txsdo_5, txsdo_7) and len(r.data) >= 8:
            return struct.unpack_from('<f', bytes(r.data), 4)[0]
    return None


def sdo_write_float(bus, endpoint_id, value):
    """Write a float to an endpoint via RxSdo. Dump any response for diagnostics."""
    payload = struct.pack('<BHBf', 1, endpoint_id, 0, value)  # opcode=1 (write)
    send(bus, CMD_RXSDO, payload)
    deadline = time.time() + 0.1
    while time.time() < deadline:
        r = bus.recv(timeout=0.1)
        if r:
            print(f"    RX id=0x{r.arbitration_id:03X} data={bytes(r.data).hex()}")


def sdo_write_bool(bus, endpoint_id, value):
    """Write a bool (uint8) to an endpoint via RxSdo."""
    payload = struct.pack('<BHBB3x', 1, endpoint_id, 0, int(value))  # opcode=1
    send(bus, CMD_RXSDO, payload)
    deadline = time.time() + 0.1
    while time.time() < deadline:
        r = bus.recv(timeout=0.1)
        if r:
            print(f"    RX id=0x{r.arbitration_id:03X} data={bytes(r.data).hex()}")


def read_encoder(bus):
    """Read encoder estimates — skip zero frames (motor broadcasts zeros when IDLE)."""
    deadline = time.time() + 2.0
    while time.time() < deadline:
        r = recv_cmd(bus, CMD_ENC_EST, timeout=0.1)
        if r:
            pos = struct.unpack_from('<f', bytes(r.data), 0)[0]
            vel = struct.unpack_from('<f', bytes(r.data), 4)[0]
            if abs(pos) > 1e-6 or abs(vel) > 1e-6:
                return pos, vel
    # Motor may genuinely be at 0.0; return last received value
    r = recv_cmd(bus, CMD_ENC_EST, timeout=0.5)
    if r:
        pos = struct.unpack_from('<f', bytes(r.data), 0)[0]
        vel = struct.unpack_from('<f', bytes(r.data), 4)[0]
        return pos, vel
    return None, None


def set_axis_state(bus, state, label):
    print(f"  Setting axis state: {label} ({state})...")
    send(bus, CMD_SET_STATE, struct.pack('<I', state))
    time.sleep(0.3)


def set_controller_mode(bus, control_mode, input_mode):
    send(bus, CMD_SET_MODE, struct.pack('<II', control_mode, input_mode))
    time.sleep(0.1)


def main():
    print(f"Opening {CAN_IFACE}, node_id={NODE_ID}, gear_ratio={GEAR_RATIO}")
    print(f"Target output-shaft position: {TARGET_RAD:.4f} rad\n")
    bus = can.interface.Bus(channel=CAN_IFACE, interface='socketcan')

    try:
        # Step 1: Clear errors
        print("1. Clearing errors...")
        send(bus, CMD_CLEAR_ERR, bytes(8))
        time.sleep(0.2)

        # Step 2: Read current position (rotor turns, from periodic broadcast)
        print("2. Reading current encoder position...")
        pos_before, vel = read_encoder(bus)
        if pos_before is not None:
            output_rad_before = pos_before * 2 * 3.14159265 / GEAR_RATIO
            print(f"   pos_estimate={pos_before:.4f} turns (rotor)  →  {output_rad_before:.4f} rad (output shaft)")
        else:
            print("   Warning: no encoder data received")
            pos_before = 0.0

        # Step 3: Set position control mode
        print("3. Setting position control mode (control=3, input=3)...")
        set_controller_mode(bus, CONTROL_MODE_POSITION, INPUT_MODE_POS_FILTER)

        # Step 4: Enter closed-loop
        print("4. Entering closed-loop control...")
        set_axis_state(bus, AXIS_STATE_CLOSED_LOOP, "CLOSED_LOOP")
        time.sleep(0.5)

        # Step 5: Move to target position (convert output-shaft rad → rotor turns)
        target_turns = TARGET_RAD * GEAR_RATIO / (2 * 3.14159265)
        print(f"5. Moving to {TARGET_RAD:.4f} rad output shaft ({target_turns:.4f} rotor turns)...")
        # Set_Input_Pos: Input_Pos (float, turns), Vel_FF (int16, 0.001 rev/s), Torque_FF (int16, 0.001 Nm)
        send(bus, CMD_SET_POS, struct.pack('<fhh', target_turns, 0, 0))
        print("   Waiting 3 seconds for motor to reach position...")
        time.sleep(3.0)

        # Step 6: Read position after move
        print("6. Reading final encoder position...")
        pos_after, _ = read_encoder(bus)
        if pos_after is not None:
            output_rad_after = pos_after * 2 * 3.14159265 / GEAR_RATIO
            print(f"   pos_estimate={pos_after:.4f} turns (rotor)  →  {output_rad_after:.4f} rad (output shaft)")
        else:
            print("   Warning: no encoder data — using last known value")
            pos_after = pos_before

        # Step 7: Go idle
        print("7. Setting motor idle...")
        set_axis_state(bus, AXIS_STATE_IDLE, "IDLE")

        # Step 8: Zero encoder using Set_Linear_Count = 0
        # This tells the motor "you are now at position 0"
        # Unit: encoder counts. With 14-bit absolute encoder (16384 CPR) on output shaft,
        # but Set_Linear_Count operates on the raw encoder count (rotor side).
        # Setting to 0 declares current position as zero.
        print(f"\n8. Setting encoder position to 0 via Set_Linear_Count (cmd=0x019)...")
        send(bus, CMD_SET_LIN_COUNT, struct.pack('<i', 0))
        time.sleep(0.1)
        print("   Sent.")

        # Step 9: Save configuration
        print("9. Saving configuration (cmd_id=0x01F)...")
        send(bus, CMD_SAVE_CFG, bytes(8))
        time.sleep(1.5)  # motor reboots after save

        print("\nDone.")
        print(f"Motor zero set at {TARGET_RAD:.3f} rad output shaft.")
        print("Note: Set_Linear_Count is a runtime change. Save_Configuration persists it to flash.")
        print("After power cycle, verify with: ros2 topic echo /joint_states")

    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()
