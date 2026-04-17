#!/usr/bin/env python3
"""
Scan for GIM protocol CAN ID by sending Retrieve Configuration (0x84)
for Zero Position (ConfType=0x00, ConfID=0x14) to every CAN ID from 1 to 0x7FF.
Listens for any response to determine:
  1. Whether GIM config commands work while motor is in MIT mode
  2. What CAN ID the motor is listening on for GIM protocol

Usage: python3 gim_can_scan.py [can_interface]
"""

import can
import struct
import sys
import time

CAN_IFACE = sys.argv[1] if len(sys.argv) > 1 else 'can0'

# GIM Retrieve Configuration frame
# BYTE0=0x84, BYTE1=ConfType(0x00=int32), BYTE2=ConfID(0x14=ZeroPos), rest NULL
RETRIEVE_CONFIG_CMD = 0x84
CONF_TYPE_INT32 = 0x00
CONF_ID_ZERO_POS = 0x14
CONF_ID_PROTOCOL = 0x1C
CONF_ID_CAN_ID   = 0x12

def make_retrieve_config(conf_type, conf_id):
    return bytes([RETRIEVE_CONFIG_CMD, conf_type, conf_id, 0, 0, 0, 0, 0])

def scan(bus, start_id=1, end_id=0x7FF, delay=0.005):
    print(f"Scanning CAN IDs 0x{start_id:03X} to 0x{end_id:03X} for GIM protocol response...")
    payload = make_retrieve_config(CONF_TYPE_INT32, CONF_ID_ZERO_POS)

    for can_id in range(start_id, end_id + 1):
        msg = can.Message(arbitration_id=can_id, data=payload, is_extended_id=False)
        try:
            bus.send(msg)
        except Exception as e:
            print(f"  Send error at 0x{can_id:03X}: {e}")
            continue

        # Listen briefly for a response
        deadline = time.time() + delay
        while time.time() < deadline:
            resp = bus.recv(timeout=delay)
            if resp and resp.data and resp.data[0] == RETRIEVE_CONFIG_CMD:
                print(f"\n[HIT] Response to query on CAN ID 0x{can_id:03X}!")
                print(f"  Response from CAN ID: 0x{resp.arbitration_id:03X}")
                d = resp.data
                print(f"  Raw bytes: {d.hex()}")
                conf_type_r = d[1]
                conf_id_r   = d[2]
                res         = d[3]
                if res == 0x00 and len(d) >= 8:
                    val_int = struct.unpack_from('<i', bytes(d[4:8]))[0]
                    pos_rad = val_int * 2 * 3.14159265 / 65536
                    print(f"  ConfType=0x{conf_type_r:02X}, ConfID=0x{conf_id_r:02X}, RES=0x{res:02X}")
                    print(f"  Zero Position raw int: {val_int}  ({pos_rad:.4f} rad)")
                else:
                    print(f"  RES=0x{res:02X} (non-zero = failure)")
                return can_id

        if can_id % 0x80 == 0:
            print(f"  ... scanned up to 0x{can_id:03X}")

    print("\nNo GIM response found. Motor likely ignores GIM config in MIT mode.")
    return None

def read_config(bus, motor_can_id, conf_type, conf_id, label):
    payload = make_retrieve_config(conf_type, conf_id)
    msg = can.Message(arbitration_id=motor_can_id, data=payload, is_extended_id=False)
    bus.send(msg)
    resp = bus.recv(timeout=0.5)
    if resp and resp.data and resp.data[0] == RETRIEVE_CONFIG_CMD:
        d = resp.data
        res = d[3]
        if res == 0x00:
            val_int = struct.unpack_from('<i', bytes(d[4:8]))[0]
            print(f"  {label}: raw={val_int}  (as pos: {val_int * 2 * 3.14159265 / 65536:.4f} rad)")
        else:
            print(f"  {label}: RES=0x{res:02X} (failure)")
    else:
        print(f"  {label}: no response")

def main():
    bus = can.interface.Bus(channel=CAN_IFACE, interface='socketcan')
    print(f"Opened {CAN_IFACE}\n")

    try:
        motor_can_id = scan(bus)

        if motor_can_id is not None:
            print(f"\nMotor GIM CAN ID confirmed: 0x{motor_can_id:03X}")
            print("Reading additional config values...")
            read_config(bus, motor_can_id, 0x00, CONF_ID_CAN_ID,   "CAN ID (0x12)")
            read_config(bus, motor_can_id, 0x00, CONF_ID_PROTOCOL, "Protocol (0x1C): 0=GIM, 1=MIT")
            read_config(bus, motor_can_id, 0x00, 0x15,             "Power-Off Position (0x15)")
    finally:
        bus.shutdown()

if __name__ == '__main__':
    main()
