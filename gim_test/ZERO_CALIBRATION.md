# GIM8108-8 Zero Position Calibration via CAN

## Overview

The GIM8108-8 actuator runs a modified ODrive firmware (~v0.5.x) with a 14-bit MA732 absolute encoder.
This document explains how zero position calibration works, what was discovered during testing, and
the recommended approach.

---

## Protocol

The motor uses **ODrive CAN Simple protocol**. CAN ID formula:

```
arbitration_id = (node_id << 5) | cmd_id
```

Key command IDs:

| Command           | cmd_id | Notes |
|-------------------|--------|-------|
| Set_Axis_State    | 0x007  | data: uint32 LE |
| Encoder_Estimates | 0x009  | broadcast: float32 pos, float32 vel |
| RxSdo (write)     | 0x004  | host → motor parameter write |
| TxSdo (ack)       | 0x005  | motor → host (NOT supported on GIM firmware) |
| Save_Configuration| 0x01F  | 8 zero bytes, triggers save + reboot |
| Reboot            | 0x016  | data[0]: 0=reboot, 1=save+reboot, 2=erase |

---

## SDO Frame Format

Writes use RxSdo (cmd_id=0x004):

```python
struct.pack('<BHBf', opcode, endpoint_id, reserved, value)
# opcode:      1 = write
# endpoint_id: uint16 little-endian
# reserved:    0x00
# value:       float32 or bool (little-endian)
```

**Important**: This GIM firmware does **not** respond with TxSdo — writes are silent (no ack).
The motor receives the frame (confirmed via CAN dump) but never replies.

---

## encoder.config.index_offset

This is the mechanism for persistent zero position. Endpoint IDs vary by firmware version:

| Firmware version | index_offset ID | use_index_offset ID |
|-----------------|-----------------|---------------------|
| v0.5.14 (confirmed on this unit) | 362 | 363 |

### How it works

- `index_offset` (float, rotor turns): offset subtracted from raw MA732 reading
- `use_index_offset` (bool): enables the offset
- Applied at startup, shifts `pos_estimate` so the calibrated position reads as 0
- `pos_estimate` (and therefore `/joint_states`) **does** respect this offset
- **Position control commands (MIT mode, Set_Input_Pos) do NOT respect this offset** — they target raw encoder counts

### Formula (from SteadyWin manual)

```
index_offset = pos_estimate + old_index_offset
```

When starting fresh (old offset = 0, use_index_offset = False), this simplifies to:

```
index_offset = raw_absolute_pos_estimate
```

---

## Key Learnings

### 1. Runtime pos_estimate accumulates — use fresh reboot reading

After motor movement, `pos_estimate` may drift from the true MA732 absolute reading.
**Always reboot first** and read `pos_estimate` immediately after entering closed-loop.
Only a post-reboot reading gives the true absolute encoder position.

### 2. use_index_offset applies to both feedback AND control

In vanilla ODrive, `use_index_offset` applies via `enc_index_cb()` (index pulse interrupt).
The GIM firmware applies it differently for the MA732 absolute encoder. After `odrive_set_offset.py`
runs and saves the config, both `pos_estimate` (feedback) and position control commands (MIT/pos)
respect the offset. `position_des: [0.0]` correctly drives to the calibrated zero.

### 3. SDO writes work silently

The GIM firmware accepts SDO writes (RxSdo) but never sends TxSdo acknowledgments.
Writes take effect but there is no confirmation. Verified by observing `pos_estimate` shift
by exactly `-X` when writing `index_offset = X`.

### 4. Position units

- `pos_estimate` is in **rotor turns** (before gearbox)
- `index_offset` is in **rotor turns**
- `/joint_states` position = `pos_estimate × 2π / gear_ratio` (output shaft radians)
- Writing `index_offset` in output shaft turns (pos/8) causes 8x unit mismatch

### 5. Save_Configuration

cmd_id `0x01F` with 8 zero bytes triggers save and reboot (legacy ODrive v0.5 behavior).
The motor takes ~3-4 seconds to reboot. A separate explicit reboot via `0x016` can also be sent.

---

## odrive_set_offset.py — Usage

```bash
pip install python-can
python3 odrive_set_offset.py can0 1
```

### What it does

1. Clears existing offset (`index_offset=0`, `use_index_offset=False`) and reboots
2. Enters closed-loop then idle to settle the encoder
3. Reads true MA732 absolute position (post-reboot, no accumulated drift)
4. Writes `index_offset = pos_estimate` (rotor turns)
5. Enables `use_index_offset = True`
6. Saves configuration and reboots
7. Reads position after reboot to verify (~0.0 means success)

### Endpoint IDs

Update these constants at the top of the script to match your firmware version:

```python
EP_INDEX_OFFSET     = 362  # encoder.config.index_offset (float, rw)
EP_USE_INDEX_OFFSET = 363  # encoder.config.use_index_offset (bool, rw)
```

---

## Handling the Command Frame Mismatch

Since position control commands target raw encoder space (not offset-adjusted),
use the **software offset** in `gim_controllers.yaml` to align commands with the calibrated zero:

```yaml
leg_impedance_controller:
  ros__parameters:
    gim_joint:
      offset: <value>   # output shaft radians at raw encoder zero
      direction: 1.0
```

To find the correct software offset value after running `odrive_set_offset.py`:

```bash
# Read raw pos_estimate (disable use_index_offset temporarily, enter closed-loop, read, re-enable)
# offset = raw_pos_estimate × 2π / gear_ratio
```

---

## Recommended Calibration Procedure

1. Move motor to desired zero position manually or via position control
2. Run `odrive_set_offset.py can0 <node_id>` — this sets persistent `index_offset`
3. After reboot, `/joint_states` will read ~0.0 at the calibrated position
4. Both MIT control and position control commands will now target the offset-adjusted frame —
   `position_des: [0.0]` will go to the calibrated zero

> **Warning**: Do not write arbitrary endpoint IDs via SDO — this can corrupt other firmware
> settings. If the motor starts behaving unexpectedly after calibration attempts, perform a
> factory reset via Motor Wizard, then re-run `odrive_set_offset.py` with the correct
> endpoint IDs (362/363 for firmware v0.5.14).
