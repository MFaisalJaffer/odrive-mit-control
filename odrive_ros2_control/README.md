# ODrive ros2_control Plugin

This package serves as a hardware interface to control ODrives from [ros2_control](https://control.ros.org/master/index.html).

It assumes that the ODrive is already configured and calibrated (see [docs](https://docs.odriverobotics.com/v/latest/guides/getting-started.html) for details).

**This is a work in progress** (see **Features**).

## Usage

For a high level usage example, see the [BotWheel Explorer ROS2 Package](../odrive_botwheel_explorer/README.md).

For MIT control mode (GIM series actuators), see [MIT_CONTROL.md](MIT_CONTROL.md).

## Features

- Communicates over Linux SocketCAN
- Position Control (with optional velocity and torque feedforward)
- Velocity Control (with optional torque feedforward)
- Torque Control
- **MIT Control Mode** — single-frame impedance control (position + velocity + Kp + Kd + torque_ff); required for SteadyWin GIM6010-8 / GIM8108-8 actuators
- Automatic control mode selection (based on which Command Interfaces are claimed by the ros2_control Controller)
- Position, velocity and torque Feedback
- Multiple ODrives

**TODO:**

- Error feedback & error handling: If an ODrive disarms for some reason (e.g. undervoltage), the application that connects to ros2_control will currently not be notified.
- Other telemetry: Additional data like temperatures, DC voltage, etc. are currently not propagated through ros2_control up to the application.


## Parameters

Top level:

- `can`: Name of the CAN interface to run on (e.g. `can0`)

Per joint:

- `node_id`: `node_id` of the ODrive / GIM motor
- `gear_ratio`: output-to-motor gear ratio (e.g. `8.0` for GIM6010-8 and GIM8108-8). Used to convert standard encoder estimates (motor-shaft turns) to output-shaft radians. Defaults to `1.0`. Not applied to the MIT command/feedback path, which natively uses output-shaft radians.

## Plugin Name

```xml
<plugin>odrive_ros2_control_plugin/ODriveHardwareInterface</plugin>
```

## Command Interfaces

(from ros2_control Controller to ODrive)

- `position` — rad (output shaft)
- `velocity` — rad/s (output shaft)
- `effort` — Nm (torque)
- `kp` — position gain (MIT mode only)
- `kd` — damping gain (MIT mode only)

MIT mode is activated automatically when a controller claims both `kp` and `kd`.

## State Interfaces

(from ODrive to ros2_control Controller)

- `position` — rad (output shaft)
- `velocity` — rad/s (output shaft)
- `effort` — Nm (torque)
