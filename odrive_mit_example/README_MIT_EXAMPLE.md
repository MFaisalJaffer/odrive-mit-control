# ODrive MIT Example

This package demonstrates how to control an ODrive using MIT Control Mode with `ros2_control`. The setup has been updated to include necessary system dependencies and fixes for the custom controller.

## Prerequisites & Installation

Before running the example, ensure your environment is set up correctly.

1.  **System Dependencies**:
    The following system packages are required for `rsl` and `realtime_tools`. You can install them manually or use the `wake_up.sh` script provided in the root of the workspace.
    
    ```bash
    # Update package lists and fix potential GPG key issues
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    apt-get update
    
    # Install required libraries
    apt-get install -y libfmt-dev libcap-dev
    ```

    *Alternatively, run `./wake_up.sh` from the workspace root.*

2.  **ROS 2 Packages**:
    Ensure you have the standard `ros2_control` packages installed or present in your workspace.
    - `ros-humble-ros2-control`
    - `ros-humble-ros2-controllers`

## Building

You need to build both the example package and the custom controller package (`my_robot_controllers`).

```bash
cd /root/ros2_ws
colcon build --packages-up-to odrive_mit_example my_robot_controllers
```

If you encounter build errors, ensure you have sourced the ROS 2 installation (`source /opt/ros/humble/setup.bash`) and installed the system dependencies listed above.

## Running the Example

1.  **Source the Workspace**:
    This is critical. You must source the workspace **after** building to ensure ROS can find the new message types and controller plugins.
    
    ```bash
    source install/setup.bash
    ```

2.  **Launch the System**:
    Start the hardware interface and controllers.
    
    ```bash
    ros2 launch odrive_mit_example mit_example.launch.py
    ```

    **Foxglove Studio Visualization:**
    If you want to visualize the robot in Foxglove Studio and connect via Foxglove Bridge:
    ```bash
    ros2 launch odrive_mit_example foxglove_mit_example.launch.py
    ```
    This launches the bridge (port 8765) and the `robot_state_publisher` needed for the 3D view.

    **What happens on launch:**
    - The `ros2_control_node` starts and connects to the ODrive via CAN.
    - The `joint_state_broadcaster` starts publishing joint states.
    - The `leg_impedance_controller` is spawned and waits for commands.

    **Note:** Ensure your ODrive is configured for CAN (baud rate 250k, 500k, or 1M) and connected to `can0`. Update `description/urdf/mit_robot.urdf.xacro` if your Node IDs differ from the defaults (0, 1, 2).

## Sending Commands

The controller subscribes to `odrive_mit_example/msg/LegCmd` messages on the `~/command` topic (usually `/leg_impedance_controller/command`).

### Via Command Line

Send a single message to test the connection. Ensure you have sourced the workspace first!

```bash
ros2 topic pub --once /leg_impedance_controller/command odrive_mit_example/msg/LegCmd "{
  position_des: [0.0, 0.0, 0.0],
  velocity_des: [0.0, 0.0, 0.0],
  feedforward_torque: [0.0, 0.0, 0.0],
  kp_scale: [10.0, 10.0, 10.0],
  kd_scale: [0.5, 0.5, 0.5]
}"
```

### Via Python Script

For continuous control or more complex logic, use a Python script.

```python
import rclpy
from rclpy.node import Node
from odrive_mit_example.msg import LegCmd

class Commander(Node):
    def __init__(self):
        super().__init__('commander_node')
        self.publisher_ = self.create_publisher(LegCmd, '/leg_impedance_controller/command', 10)
        self.timer = self.create_timer(1.0, self.publish_command)

    def publish_command(self):
        msg = LegCmd()
        msg.position_des = [0.0, 0.0, 0.0]
        msg.velocity_des = [0.0, 0.0, 0.0]
        msg.feedforward_torque = [0.0, 0.0, 0.0]
        msg.kp_scale = [10.0, 10.0, 10.0]
        msg.kd_scale = [0.5, 0.5, 0.5]
        
        self.publisher_.publish(msg)
        self.get_logger().info('Sent command to ODrive')

def main(args=None):
    rclpy.init(args=args)
    node = Commander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Smooth Trajectory Generator (Cubic Splines)

If you find the direct command steps too abrupt (jerky), you can use the included trajectory generator script. This creates a smooth cubic spline path from the current position to the target position.

1.  **Launch the System** (Terminal 1):
    ```bash
    ros2 launch odrive_mit_example foxglove_mit_example.launch.py
    ```

2.  **Run the Trajectory Generator** (Terminal 2):
    ```bash
    source install/setup.bash
    python3 src/odrive_can/odrive_mit_example/scripts/trajectory_generator.py
    ```
    *This node subscribes to your high-level commands and publishes smooth `LegCmd` messages to the controller.*

3.  **Send a Target**:
    Publish to `/trajectory_target` with your desired joint positions (in radians).
    
    ```bash
    # Example: Move hip to 0.5 rad, others to 0.0
    ros2 topic pub --once /trajectory_target std_msgs/msg/Float64MultiArray "{data: [0.5, 0.0, 0.0]}"
    ```
    The robot will smoothly interpolate to this position over 2.0 seconds.

## Right Leg Control (Inverse Kinematics)

For the **KBot V2 Right Leg**, we provide a dedicated launch file and an Inverse Kinematics (IK) node that allows you to control the end-effector position in Cartesian space.

### 1. Launch the Leg Control System

This starts the hardware interface, the impedance controller, and the IK node.

```bash
ros2 launch odrive_mit_example leg_control.launch.py
```

### 2. Send a Target Position

The IK node subscribes to `/leg_target` (Geometry Point). Coordinates are in meters relative to the hip attachment point (base_link).

```bash
# Example: Move to 40cm below the hip (x=0, y=0, z=-0.4)
ros2 topic pub --once /leg_target geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: -0.4}"
```

### 3. Actuator Configuration

You can configure actuator offsets and directions in `src/odrive_can/odrive_mit_example/config/actuator_config.yaml`.

-   **Offsets**: Zero-position adjustment in radians. `actuator_pos = joint_pos + offset`.
-   **Directions**: Motor polarity (1.0 or -1.0). `command_to_hw = (joint_cmd + offset) * direction`.

```yaml
leg_impedance_controller:
  ros__parameters:
    actuator_offsets:
      dof_right_hip_pitch_04: 0.0
      # ...
    actuator_directions:
      dof_right_hip_pitch_04: 1.0  # Set to -1.0 to invert motor direction
      # ...
```

## Troubleshooting

- **"The passed message type is invalid"**: This means your current terminal doesn't know about the `LegCmd` message. Run `source install/setup.bash` again. Check visibility with `ros2 interface list | grep LegCmd`.
- **Controller fails to spawn**: Check the `ros2_control_node` logs. Usually indicates the plugin wasn't found (rebuild `my_robot_controllers`) or CAN connection failed.
