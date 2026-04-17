# Running the GIM Test Stack

Single-joint test setup for the GIM8108-8 actuator using ros2_control + LegImpedanceController.

---

## Prerequisites

- Docker container running with ROS2 Humble
- CAN interface `can0` up and running on the host
- Motor powered on and connected via CAN

### Verify CAN is up (on host, outside container)

```bash
ip link show can0
# Should show: can0: <...> mtu 16 qdisc pfifo_fast state UP
```

---

## 1. Build (first time or after code changes)

Inside the container:

```bash
cd /root/ros2_ws
colcon build --packages-select odrive_can my_robot_controllers
source install/setup.bash
```

> Note: `gim_test` is not a colcon package — it lives inside `odrive_can`.
> Always use the full path to the launch file (see below).

---

## 2. Launch

```bash
source /root/ros2_ws/install/setup.bash
ros2 launch /root/ros2_ws/src/odrive_can/gim_test/gim_test.launch.py
```

Expected output — you should see:
- `[ros2_control_node]` starting
- `[INFO] ... Configured and activated joint_state_broadcaster`
- `[INFO] ... Configured and activated leg_impedance_controller`
- No `SocketCAN` errors (means `can0` is up and `node_id=1` responded)

---

## 3. Read Joint Position

In a separate terminal (source first):

```bash
source /root/ros2_ws/install/setup.bash
ros2 topic echo /joint_states
```

Output example:
```
name: [gim_joint]
position: [1.027]    # output shaft radians
velocity: [-0.007]
effort:   [nan]
```

> Position is output shaft radians = `pos_estimate × 2π / gear_ratio` (gear_ratio=8.0).
> This value respects `encoder.config.index_offset` if set.

---

## 4. Send Commands

Commands go to `/leg_impedance_controller/command` as a `LegCmd` message.
All fields are **arrays** (one element per joint).

### Go to position 0.0 rad

```bash
ros2 topic pub --once /leg_impedance_controller/command odrive_mit_example/msg/LegCmd \
  "{position_des: [0.0], velocity_des: [0.0], feedforward_torque: [0.0], kp_scale: [20.0], kd_scale: [5.0]}"
```

### Go to a specific position (e.g. 1.57 rad)

```bash
ros2 topic pub --once /leg_impedance_controller/command odrive_mit_example/msg/LegCmd \
  "{position_des: [1.57], velocity_des: [0.0], feedforward_torque: [0.0], kp_scale: [20.0], kd_scale: [5.0]}"
```

### Gains

| Parameter       | Description                        | Typical value |
|-----------------|------------------------------------|---------------|
| `kp_scale`      | Position gain scale                | 20.0          |
| `kd_scale`      | Velocity damping scale             | 5.0           |
| `feedforward_torque` | Additional torque feedforward | 0.0           |

> **Note**: Commands target the **raw encoder frame**, not the `index_offset`-adjusted frame.
> If `/joint_states` reads 2.5 rad at your desired zero, commanding `position_des: [2.5]`
> will move to that physical position. Use the `offset:` in `gim_controllers.yaml` to
> remap the command frame. See `ZERO_CALIBRATION.md` for details.

---

## 5. Controller Configuration

Edit `/root/ros2_ws/src/odrive_can/gim_test/gim_controllers.yaml`:

```yaml
leg_impedance_controller:
  ros__parameters:
    joints:
      - gim_joint
    cmd_timeout_s: 0.5      # motor goes idle if no command received within this time
    gim_joint:
      offset: 0.0           # output shaft radians added to position command
      direction: 1.0        # 1.0 or -1.0 to flip direction
```

After editing, rebuild and relaunch.

---

## 6. URDF Key Parameters

File: `gim_test.urdf`

| Parameter       | Value  | Notes |
|-----------------|--------|-------|
| `node_id`       | 1      | CAN node ID of the motor |
| `gear_ratio`    | 8.0    | GIM8108-8 planetary gearbox |
| `use_mit_protocol` | true | MIT torque control mode |
| `can`           | can0   | CAN interface |
| `control_mode`  | 3      | Position control (ODrive) |
| `input_mode`    | 1      | Passthrough |

---

## 7. Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `package 'gim_test' not found` | Using package name instead of path | Use full path: `ros2 launch /root/ros2_ws/src/odrive_can/gim_test/gim_test.launch.py` |
| `Failed to initialize SocketCAN` | `can0` not in URDF or interface down | Check `<param name="can">can0</param>` in URDF |
| `command not found: ros2` | Workspace not sourced | Run `source /root/ros2_ws/install/setup.bash` |
| Motor doesn't move | `cmd_timeout_s` expired | Publish command again; check topic name is correct |
| `/joint_states` shows 0.0 constantly | Motor in IDLE, not publishing | Launch stack activates motor; if persists check CAN connection |
| `parameter 'cmd_timeout_s' already declared` | Controller reconfigured without restart | Restart the launch file |
