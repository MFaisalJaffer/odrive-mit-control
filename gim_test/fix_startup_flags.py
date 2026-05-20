#!/usr/bin/env python3
"""
Disable startup auto-calibration flags and mark the encoder as pre-calibrated
so a previously-saved index_offset survives a cold power-cycle.

Run AFTER urdf_calibrate_joint.py has set index_offset on the actuator.
Then power-cycle the actuator and verify the offset is honored.

Endpoints (fw 3.11.1 / 0.6.0):
  144 axis0.config.startup_motor_calibration            -> False
  145 axis0.config.startup_encoder_index_search         -> False
  146 axis0.config.startup_encoder_offset_calibration   -> False
  147 axis0.config.startup_closed_loop_control          -> False
  370 axis0.encoder.config.pre_calibrated               -> True
  172 axis0.config.general_lockin.ramp_distance         -> 0.0
       (eliminates the ~2.68 rotor-turn spin on every IDLE→CLOSED_LOOP
        transition that was offsetting pos_estimate by ~2.11 rad output)

Usage:
  python3 fix_startup_flags.py --can can0 --node 1
"""

import argparse
import can
import struct
import time

CMD_RXSDO    = 0x004
CMD_REBOOT   = 0x016
CMD_SAVE_CFG = 0x01F


def make_can_id(node_id, cmd):
    return (node_id << 5) | cmd


def send(bus, node_id, cmd, data):
    bus.send(can.Message(
        arbitration_id=make_can_id(node_id, cmd),
        data=data,
        is_extended_id=False,
    ))


def sdo_write_bool(bus, node_id, ep, value):
    payload = struct.pack('<BHBB3x', 1, ep, 0, int(value))
    send(bus, node_id, CMD_RXSDO, payload)


def sdo_write_float(bus, node_id, ep, value):
    payload = struct.pack('<BHBf', 1, ep, 0, value)
    send(bus, node_id, CMD_RXSDO, payload)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--can',  default='can0')
    p.add_argument('--node', type=int, required=True)
    args = p.parse_args()

    bus = can.interface.Bus(channel=args.can, interface='socketcan')
    print(f"Opened {args.can}, node_id={args.node}")

    try:
        bool_writes = [
            (144, False, 'startup_motor_calibration'),
            (145, False, 'startup_encoder_index_search'),
            (146, False, 'startup_encoder_offset_calibration'),
            (147, False, 'startup_closed_loop_control'),
            (370, True,  'encoder.config.pre_calibrated'),
        ]
        for ep, val, name in bool_writes:
            print(f"  ep {ep:3d}  {name} = {val}")
            sdo_write_bool(bus, args.node, ep, val)
            time.sleep(0.05)

        # Zero ALL general_lockin fields. ramp_distance=0 alone was
        # observed to not suppress lockin; zeroing current as well means
        # the motor cannot apply torque to spin the rotor regardless of
        # what other lockin logic runs.
        float_writes = [
            (170, 0.0, 'general_lockin.current'),
            (171, 0.0, 'general_lockin.ramp_time'),
            (172, 0.0, 'general_lockin.ramp_distance'),
            (173, 0.0, 'general_lockin.accel'),
            (174, 0.0, 'general_lockin.vel'),
        ]
        for ep, val, name in float_writes:
            print(f"  ep {ep:3d}  {name} = {val}")
            sdo_write_float(bus, args.node, ep, val)
            time.sleep(0.05)

        print("\nSaving configuration...")
        send(bus, args.node, CMD_SAVE_CFG, bytes(8))
        time.sleep(2.5)   # give flash erase+write time to commit before reboot

        print("Rebooting...")
        send(bus, args.node, CMD_REBOOT, bytes([0]))
        print("\nDone. Now POWER-CYCLE the actuator (full power off, then on)")
        print("and check whether the previously-set index_offset is honored.")

    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()
