#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from odrive_mit_example.msg import LegCmd
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        
        # Parameters
        self.declare_parameter('move_duration', 0.7)  # Seconds to move
        self.declare_parameter('control_rate', 100.0) # Hz
        
        # Publisher for the robot command
        self.cmd_pub = self.create_publisher(LegCmd, '/leg_impedance_controller/command', 10)
        
        # Subscriber for current state (optional, used for initial position)
        self.state_sub = self.create_subscription(JointState, '/joint_states', self.state_callback, 10)
        
        # Subscriber for new targets
        # Message format: [q1, q2, q3] in radians
        self.target_sub = self.create_subscription(Float64MultiArray, '/trajectory_target', self.target_callback, 10)
        
        self.dt = 1.0 / self.get_parameter('control_rate').value
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        # State
        self.current_pos = np.zeros(3)
        self.current_vel = np.zeros(3)
        self.has_state = False
        
        # Trajectory state
        self.is_moving = False
        self.start_pos = np.zeros(3)
        self.target_pos = np.zeros(3)
        self.start_time = 0.0
        self.move_duration = self.get_parameter('move_duration').value
        
        # Last commanded setpoint (to ensure continuity if we receive a new command while moving)
        self.last_cmd_pos = np.zeros(3)
        
        self.get_logger().info('Trajectory Generator Started. Waiting for /joint_states...')

    def state_callback(self, msg):
        # Assumes joint_states order matches LegCmd order [hip, knee, ankle]
        # In a real app, check msg.name
        if len(msg.position) >= 3:
            # Simple mapping; better to map by name if possible
            # Assuming names are ['hip_joint', 'knee_joint', 'ankle_joint']
            try:
                # Find indices if names are present
                hip_idx = msg.name.index('hip_joint')
                knee_idx = msg.name.index('knee_joint')
                ankle_idx = msg.name.index('ankle_joint')
                self.current_pos = np.array([msg.position[hip_idx], msg.position[knee_idx], msg.position[ankle_idx]])
                self.current_vel = np.array([msg.velocity[hip_idx], msg.velocity[knee_idx], msg.velocity[ankle_idx]])
            except ValueError:
                # Fallback to direct index if names don't match exactly or just take first 3
                self.current_pos = np.array(msg.position[:3])
                self.current_vel = np.array(msg.velocity[:3])
            
            if not self.has_state:
                self.has_state = True
                self.last_cmd_pos = self.current_pos
                self.get_logger().info(f'Received initial state: {self.current_pos}')

    def target_callback(self, msg):
        if not self.has_state:
            self.get_logger().warn('Cannot set target: No joint state received yet.')
            return
            
        if len(msg.data) != 3:
            self.get_logger().error('Target must have 3 elements [hip, knee, ankle]')
            return

        self.start_pos = self.last_cmd_pos # Start from where we last commanded
        self.target_pos = np.array(msg.data)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.is_moving = True
        self.get_logger().info(f'New target received: {self.target_pos}')

    def get_cubic_spline_point(self, t, T, p0, pf):
        """
        Returns (position, velocity) at time t for a move from p0 to pf in duration T.
        Velocity at start and end is assumed 0.
        """
        if t >= T:
            return pf, 0.0
        if t <= 0:
            return p0, 0.0
            
        # Normalized time
        tau = t / T
        
        # Cubic Hermite spline basis functions for v0=0, vf=0
        # p(tau) = p0 * (2*tau^3 - 3*tau^2 + 1) + pf * (-2*tau^3 + 3*tau^2)
        # p(tau) = p0 + (pf - p0) * (3*tau^2 - 2*tau^3)
        
        # Let s = 3*tau^2 - 2*tau^3 (Smoothstep function)
        s = 3*(tau**2) - 2*(tau**3)
        pos = p0 + (pf - p0) * s
        
        # Velocity
        # v(t) = dp/dt = (dp/dtau) * (dtau/dt)
        # ds/dtau = 6*tau - 6*tau^2
        # v = (pf - p0) * (6*tau - 6*tau^2) * (1/T)
        
        ds = 6*tau - 6*(tau**2)
        vel = (pf - p0) * ds / T
        
        return pos, vel

    def control_loop(self):
        if not self.has_state:
            return

        msg = LegCmd()
        
        # Default gains (Matching README example)
        msg.kp_scale = [10.0, 50.0, 10.0]
        msg.kd_scale = [0.5, 0.5, 0.5]
        msg.feedforward_torque = [0.0, 0.0, 0.0]

        if self.is_moving:
            now = self.get_clock().now().nanoseconds / 1e9
            t = now - self.start_time
            
            # Generate trajectory for each joint
            pos_cmds = []
            vel_cmds = []
            
            for i in range(3):
                p, v = self.get_cubic_spline_point(t, self.move_duration, self.start_pos[i], self.target_pos[i])
                pos_cmds.append(p)
                vel_cmds.append(v)
            
            msg.position_des = pos_cmds
            msg.velocity_des = vel_cmds
            
            self.last_cmd_pos = np.array(pos_cmds)
            
            if t >= self.move_duration:
                self.is_moving = False
                self.get_logger().info('Target reached')
                
        else:
            # Hold last position
            msg.position_des = self.last_cmd_pos.tolist()
            msg.velocity_des = [0.0, 0.0, 0.0]

        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

