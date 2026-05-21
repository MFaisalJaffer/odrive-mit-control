#!/usr/bin/env python3
"""
Reboot a single ODrive/GIM actuator by node ID and CAN interface, and
compare CMD_ENC_EST vs MIT feedback positions before and after the reboot.

CMD_ENC_EST gives the firmware's internal multi-turn pos_estimate
(rotor turns → output shaft rad via gear ratio).
MIT feedback (response to a zero-effort MIT command) gives the
firmware's output-shaft position directly as packed in the MIT frame.

When these disagree we know the firmware's two reporting paths are out
of sync — useful for tracking down offset/turn issues.

Usage:
  python3 reboot_actuator.py --can can0 --node 5
  python3 reboot_actuator.py --can can1 --node 13
"""

import argparse
import can
import math
import struct
import time

GEAR_RATIO = 8.0

CMD_ENC_EST   = 0x009
CMD_SET_STATE = 0x007
CMD_MIT       = 0x008
CMD_REBOOT    = 0x016

AXIS_STATE_IDLE        = 1
AXIS_STATE_CLOSED_LOOP = 8

# MIT command encoding ranges
MIT_P_MIN,  MIT_P_MAX  = -12.5, 12.5
MIT_V_MIN,  MIT_V_MAX  = -45.0, 45.0
MIT_KP_MIN, MIT_KP_MAX =   0.0, 500.0
MIT_KD_MIN, MIT_KD_MAX =   0.0, 5.0
MIT_T_MIN,  MIT_T_MAX  = -18.0, 18.0

# MIT feedback decoding ranges
FB_P_MIN, FB_P_MAX = -12.5, 12.5
FB_V_MIN, FB_V_MAX = -65.0, 65.0
FB_T_MIN, FB_T_MAX = -50.0, 50.0


def can_id(node_id, cmd):
    return (node_id << 5) | cmd


def send(bus, node_id, cmd, data):
    bus.send(can.Message(arbitration_id=can_id(node_id, cmd),
                         data=data, is_extended_id=False))


def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    return int((x - x_min) / (x_max - x_min) * ((1 << bits) - 1))


def uint_to_float(x_int, x_min, x_max, bits):
    return x_int * (x_max - x_min) / ((1 << bits) - 1) + x_min


def send_mit_zero(bus, node_id):
    """Send MIT with zero kp/kd/torque — no force applied; feedback returned."""
    p   = float_to_uint(0.0, MIT_P_MIN, MIT_P_MAX, 16)
    v   = float_to_uint(0.0, MIT_V_MIN, MIT_V_MAX, 12)
    kp_ = float_to_uint(0.0, MIT_KP_MIN, MIT_KP_MAX, 12)
    kd_ = float_to_uint(0.0, MIT_KD_MIN, MIT_KD_MAX, 12)
    t   = float_to_uint(0.0, MIT_T_MIN,  MIT_T_MAX,  12)
    data = bytes([
        (p >> 8) & 0xFF, p & 0xFF,
        (v >> 4) & 0xFF,
        ((v & 0xF) << 4) | ((kp_ >> 8) & 0xF),
        kp_ & 0xFF,
        (kd_ >> 4) & 0xFF,
        ((kd_ & 0xF) << 4) | ((t >> 8) & 0xF),
        t & 0xFF,
    ])
    send(bus, node_id, CMD_MIT, data)


def read_enc_est(bus, node_id, timeout=1.5):
    """Read CMD_ENC_EST broadcast → output-shaft radians via gear_ratio."""
    enc_id = can_id(node_id, CMD_ENC_EST)
    deadline = time.time() + timeout
    while time.time() < deadline:
        r = bus.recv(timeout=0.1)
        if r and r.arbitration_id == enc_id and len(r.data) >= 4:
            pos_rev = struct.unpack_from('<f', bytes(r.data), 0)[0]
            return pos_rev * 2 * math.pi / GEAR_RATIO
    return None


def read_mit_feedback(bus, node_id, samples=10, timeout=1.5):
    """Send zero-effort MIT and decode response frames → output-shaft radians.

    Sends at 50 Hz; returns the last decoded position after `samples` responses
    or until `timeout` expires."""
    mit_id = can_id(node_id, CMD_MIT)
    deadline = time.time() + timeout
    last_pos = None
    count = 0
    last_send = 0.0
    while time.time() < deadline:
        now = time.time()
        if now - last_send > 0.02:
            send_mit_zero(bus, node_id)
            last_send = now
        r = bus.recv(timeout=0.01)
        if r and r.arbitration_id == mit_id and len(r.data) >= 6:
            d = r.data
            pos_raw = (d[1] << 8) | d[2]
            last_pos = uint_to_float(pos_raw, FB_P_MIN, FB_P_MAX, 16)
            count += 1
            if count >= samples:
                return last_pos
    return last_pos


def wrap_to_pi(x):
    """Wrap angle to [-π, +π]."""
    if x is None:
        return None
    return ((x + math.pi) % (2 * math.pi)) - math.pi


def fmt(v):
    return f"{v:+.6f}" if v is not None else "  (no data)"


def fmt_pair(raw):
    """Format (raw, wrapped) pair."""
    if raw is None:
        return "  (no data)"
    w = wrap_to_pi(raw)
    return f"{raw:+.6f} rad  (wrapped: {w:+.6f})"


def sample(bus, node_id):
    """Enter CLOSED_LOOP, read both encoder broadcast and MIT feedback,
    return to IDLE. Returns (enc_est_rad, mit_fb_rad)."""
    send(bus, node_id, CMD_SET_STATE, struct.pack('<I', AXIS_STATE_CLOSED_LOOP))
    time.sleep(1.0)
    enc = read_enc_est(bus, node_id)
    mit = read_mit_feedback(bus, node_id)
    send(bus, node_id, CMD_SET_STATE, struct.pack('<I', AXIS_STATE_IDLE))
    return enc, mit


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--can',  required=True, help='CAN interface (e.g. can0, can1)')
    p.add_argument('--node', type=int, required=True, help='Node ID')
    args = p.parse_args()

    bus = can.interface.Bus(channel=args.can, interface='socketcan')
    try:
        print(f"\n── Sampling BEFORE reboot ({args.can}, node {args.node}) ──")
        enc_before, mit_before = sample(bus, args.node)
        print(f"  CMD_ENC_EST  : {fmt_pair(enc_before)}")
        print(f"  MIT feedback : {fmt_pair(mit_before)}")
        if enc_before is not None and mit_before is not None:
            diff = wrap_to_pi(enc_before) - wrap_to_pi(mit_before)
            print(f"  diff (wrapped): {diff:+.6f} rad")

        print(f"\n── Sending reboot to node {args.node} ──")
        send(bus, args.node, CMD_REBOOT, bytes([0]))
        print("  Waiting 5 seconds for reboot...")
        time.sleep(5.0)

        print(f"\n── Sampling AFTER reboot ──")
        enc_after, mit_after = sample(bus, args.node)
        print(f"  CMD_ENC_EST  : {fmt_pair(enc_after)}")
        print(f"  MIT feedback : {fmt_pair(mit_after)}")
        if enc_after is not None and mit_after is not None:
            diff = wrap_to_pi(enc_after) - wrap_to_pi(mit_after)
            print(f"  diff (wrapped): {diff:+.6f} rad")

        # Compare before vs after — wrapped deltas show "true" motion (under 2π)
        print(f"\n── Delta across reboot (wrapped) ──")
        if enc_before is not None and enc_after is not None:
            d_raw     = enc_after - enc_before
            d_wrapped = wrap_to_pi(enc_after) - wrap_to_pi(enc_before)
            print(f"  CMD_ENC_EST  : raw={d_raw:+.6f}  wrapped={d_wrapped:+.6f} rad")
        if mit_before is not None and mit_after is not None:
            d_raw     = mit_after - mit_before
            d_wrapped = wrap_to_pi(mit_after) - wrap_to_pi(mit_before)
            print(f"  MIT feedback : raw={d_raw:+.6f}  wrapped={d_wrapped:+.6f} rad")
    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()
