# MIT Control Mode — odrive_ros2_control

This document describes the MIT control mode added to the `odrive_ros2_control` hardware interface for use with the **SteadyWin GIM6010-8** and **GIM8108-8** motors.

## Overview

MIT control (also called "motion control" in the SteadyWin manual) implements the following control law on the motor firmware:

```
T_target = Kp × (P_target − P_current) + Kd × (V_target − V_current) + T_ff
```

All five parameters — position, velocity, Kp, Kd, and torque feedforward — are packed into a single 8-byte CAN frame sent every control cycle. This is the preferred mode for legged robots because **Kp and Kd can be changed every cycle**, enabling variable-impedance control (e.g., stiff during stance, compliant during swing).

## Unit Convention

**All position and velocity values in the MIT protocol are in output-shaft radians and rad/s.** The hardware interface passes them through directly — no gear-ratio scaling or 2π conversion is applied in the MIT command or feedback path.

This differs from the standard ODrive CAN protocol (Set_Input_Pos / Get_Encoder_Estimates), which reports motor-shaft turns. For those messages the hardware interface applies `× 2π / gear_ratio` to convert to output-shaft radians.

## CAN Protocol

CAN ID format (standard 11-bit): `(node_id << 5) | cmd_id`

### Command Frame (host → motor)

cmd_id: `0x008`, 8 bytes, **big-endian**

| Bytes | Bits | Field | Physical Range | Encoding |
|-------|------|-------|---------------|----------|
| 0–1 | [63:48] | Position | [−12.5, 12.5] rad (output shaft) | uint16 |
| 2, 3[7:4] | [47:36] | Velocity | [−45, 45] rad/s (output shaft) | uint12 |
| 3[3:0], 4 | [35:24] | Kp | [0, 500] | uint12 |
| 5, 6[7:4] | [23:12] | Kd | [0, 5] | uint12 |
| 6[3:0], 7 | [11:0] | Torque FF | [−18, 18] Nm | uint12 |

Encoding formula:
```
uint_val = (float_val − range_min) × ((1 << bits) − 1) / (range_max − range_min)
```

### Feedback Frame (motor → host)

cmd_id: `0x008`, 6 bytes, **big-endian**

| Bytes | Field | Decoding | Unit |
|-------|-------|----------|------|
| 0 | Motor ID | raw uint8 | — |
| 1–2 | Position | `raw × 0.000381 − 12.5` | rad (output shaft) |
| 3, 4[7:4] | Velocity | `raw × 0.03175 − 65.0` | rad/s (output shaft) |
| 4[3:0], 5 | Torque | `raw × 0.02442 − 50.0` | Nm |

The decoded values are already in output-shaft radians — no further scaling is applied.

### Motor Activation

No `Set_Controller_Mode` message is required. The firmware processes `0x008` frames independently. The motor must be in `AXIS_STATE_CLOSED_LOOP_CONTROL` (state 8).

## ros2_control Integration

### Command Interfaces (per joint)

| Interface | Type | Unit | Description |
|-----------|------|------|-------------|
| `position` | command | rad | Target position — output shaft (P_target) |
| `velocity` | command | rad/s | Target velocity — output shaft (V_target) |
| `effort` | command | Nm | Torque feedforward (T_ff) |
| `kp` | command | — | Position gain |
| `kd` | command | — | Damping gain |

MIT mode is activated when a controller claims **both** `kp` and `kd` interfaces. Standard position/velocity/torque modes are used otherwise.

### State Interfaces (per joint)

| Interface | Unit | Source |
|-----------|------|--------|
| `position` | rad (output shaft) | MIT feedback frame (0x008) or encoder estimates (0x009) |
| `velocity` | rad/s (output shaft) | MIT feedback frame (0x008) or encoder estimates (0x009) |
| `effort` | Nm | MIT feedback frame (0x008) or Get_Torques (0x01C) |

## URDF / xacro Configuration

Use plugin name `odrive_ros2_control_plugin/ODriveHardwareInterface`. The `gear_ratio` parameter is required for GIM motors so that standard encoder estimates (motor-shaft turns) are correctly converted to output-shaft radians. It does **not** affect the MIT command/feedback path.

```xml
<ros2_control name="bipedal_robot" type="system">
  <hardware>
    <plugin>odrive_ros2_control_plugin/ODriveHardwareInterface</plugin>
    <param name="can">can0</param>
  </hardware>

  <joint name="left_hip_pitch">
    <param name="node_id">1</param>
    <param name="gear_ratio">8.0</param>
    <command_interface name="position">
      <param name="min">-18.85</param>
      <param name="max">18.85</param>
    </command_interface>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
    <command_interface name="kp"/>
    <command_interface name="kd"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <joint name="right_hip_pitch">
    <param name="node_id">2</param>
    <param name="gear_ratio">8.0</param>
    <command_interface name="position">
      <param name="min">-18.85</param>
      <param name="max">18.85</param>
    </command_interface>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
    <command_interface name="kp"/>
    <command_interface name="kd"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <!-- repeat for all joints -->
</ros2_control>
```

> **Joint limits:** The MIT position range is ±12.5 rad (output shaft). Set URDF limits accordingly — ±18.85 rad (≈ ±6π) is a safe default for the GIM6010-8. Limits that are too narrow (e.g. the URDF default ±3.14 rad) will cause the controller to clamp commands before they reach the hardware interface.

## Controller Usage

A custom legged-robot controller writes to all five command interfaces each cycle:

```cpp
// Stance phase — high stiffness
joint_kp_cmd_[i] = 150.0;   // Nm/rad
joint_kd_cmd_[i] = 3.0;     // Nm·s/rad
joint_pos_cmd_[i] = desired_pos;
joint_vel_cmd_[i] = 0.0;
joint_eff_cmd_[i] = 0.0;

// Swing phase — low impedance
joint_kp_cmd_[i] = 10.0;
joint_kd_cmd_[i] = 0.5;
joint_pos_cmd_[i] = desired_pos;
joint_vel_cmd_[i] = desired_vel;
joint_eff_cmd_[i] = torque_ff;
```

## Motor Specifications

| Motor | Max Speed | Peak Torque | Gear Ratio |
|-------|-----------|-------------|------------|
| GIM6010-8 | ~45 rad/s (output) | ~18 Nm (output) | 8:1 |
| GIM8108-8 | refer to datasheet | refer to datasheet | 8:1 |

The MIT command ranges above (±45 rad/s, ±18 Nm) reflect GIM6010-8 physical limits. The CAN protocol supports up to ±65 rad/s and ±50 Nm; the firmware clips to the motor's actual limits.

## CAN Bus Setup

```bash
# Bring up CAN interface at 500 kbps (factory default)
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# For 1 Mbps (if configured)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

Each motor's `node_id` is readable and configurable via:
```
odrv0.axis0.config.can.node_id
```
