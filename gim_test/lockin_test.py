#!/usr/bin/env python3
"""
Verify whether the IDLE -> CLOSED_LOOP transition triggers a lockin spin.

Procedure:
  1. Put the actuator in CLOSED_LOOP (this is where lockin would fire).
  2. Stream live pos_estimate from the broadcast (now sampled from the
     real encoder, since the firmware only samples in CLOSED_LOOP).
  3. The user moves the joint by hand and watches whether the reading
     tracks reality.
  4. ENTER or Ctrl+C returns the actuator to IDLE.

Interpretation:
  - If position jumps by ~2.11 rad at the instant CLOSED_LOOP is entered,
    lockin is still firing. Zero out more lockin fields (calibration_lockin
    eps 156-160) or investigate firmware behaviour.
  - If position stays stable and tracks hand motion, lockin is dead and
    the encoder is reading correctly. Use this for real calibration.

Usage:
  python3 lockin_test.py --can can0 --node 5
"""

import argparse
import can
import math
import struct
import sys
import select
import time

CMD_SET_STATE = 0x007
CMD_ENC_EST   = 0x009

AXIS_STATE_IDLE        = 1
AXIS_STATE_CLOSED_LOOP = 8

GEAR_RATIO = 8.0


def can_id(node_id, cmd):
    return (node_id << 5) | cmd


def send_state(bus, node_id, state):
    bus.send(can.Message(
        arbitration_id=can_id(node_id, CMD_SET_STATE),
        data=struct.pack('<I', state),
        is_extended_id=False,
    ))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--can',  default='can0')
    p.add_argument('--node', type=int, required=True)
    args = p.parse_args()

    bus = can.interface.Bus(channel=args.can, interface='socketcan')
    enc_arb = can_id(args.node, CMD_ENC_EST)

    print(f"Opened {args.can}, node_id={args.node}")
    print()

    # Snapshot the IDLE broadcast for a couple of frames so we know what was
    # being reported before the transition.
    print("Reading IDLE broadcast for 1s (should be stale on this firmware)...")
    idle_pos = None
    deadline = time.time() + 1.0
    while time.time() < deadline:
        r = bus.recv(timeout=0.05)
        if r and r.arbitration_id == enc_arb and len(r.data) >= 8:
            pos = struct.unpack_from('<f', bytes(r.data), 0)[0]
            idle_pos = pos
    idle_rad = (idle_pos * 2 * math.pi / GEAR_RATIO) if idle_pos is not None else None
    print(f"  IDLE last pos_estimate = {idle_pos} rev"
          + (f"  ({idle_rad:+.4f} rad output)" if idle_rad is not None else ""))
    print()

    try:
        print("Entering CLOSED_LOOP — watch the joint for a physical twitch.")
        print("If it kicks, lockin is alive. If it stays still, lockin is dead.")
        print()
        send_state(bus, args.node, AXIS_STATE_CLOSED_LOOP)
        cl_entry_time = time.time()

        # Capture the first few CLOSED_LOOP samples
        print("First CLOSED_LOOP samples (delta from IDLE = lockin offset):")
        first_samples = []
        deadline = time.time() + 1.5
        while time.time() < deadline and len(first_samples) < 8:
            r = bus.recv(timeout=0.05)
            if r and r.arbitration_id == enc_arb and len(r.data) >= 8:
                pos = struct.unpack_from('<f', bytes(r.data), 0)[0]
                first_samples.append((time.time() - cl_entry_time, pos))

        for dt, pos in first_samples:
            rad = pos * 2 * math.pi / GEAR_RATIO
            delta = (rad - idle_rad) if idle_rad is not None else float('nan')
            print(f"  t={dt*1000:6.0f}ms   pos={pos:+.6f} rev  ({rad:+.4f} rad)   "
                  f"delta-from-idle={delta:+.4f} rad")

        if idle_rad is not None and first_samples:
            final_rad = first_samples[-1][1] * 2 * math.pi / GEAR_RATIO
            jump = abs(final_rad - idle_rad)
            print()
            if jump > 1.0:
                print(f"  >>> LOCKIN STILL ACTIVE — position jumped {jump:.3f} rad on entry")
            elif jump > 0.05:
                print(f"  >>> Small offset of {jump:.3f} rad — could be lockin remnant or settle")
            else:
                print(f"  >>> No significant jump ({jump:.3f} rad) — lockin appears dead")

        print()
        print("Now move the joint by hand. Position should track:")
        print("(Press ENTER or Ctrl+C to stop and return to IDLE)")
        print()

        while True:
            r = bus.recv(timeout=0.1)
            if r and r.arbitration_id == enc_arb and len(r.data) >= 8:
                pos = struct.unpack_from('<f', bytes(r.data), 0)[0]
                vel = struct.unpack_from('<f', bytes(r.data), 4)[0]
                rad = pos * 2 * math.pi / GEAR_RATIO
                vel_rad = vel * 2 * math.pi / GEAR_RATIO
                print(f"    pos={rad:+.4f} rad  ({pos:+.6f} rev)   "
                      f"vel={vel_rad:+.3f} rad/s   ", end='\r')

            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                break

    except KeyboardInterrupt:
        pass
    finally:
        print()
        print("Returning to IDLE...")
        send_state(bus, args.node, AXIS_STATE_IDLE)
        time.sleep(0.2)
        bus.shutdown()


if __name__ == '__main__':
    main()
