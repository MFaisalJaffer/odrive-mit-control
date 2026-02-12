#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from odrive_mit_example.msg import LegCmd
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import numpy as np
import math

class SimpleIKNode(Node):
    def __init__(self):
        super().__init__('simple_ik_node')
        
        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(LegCmd, '/leg_impedance_controller/command', 10)
        self.target_sub = self.create_subscription(Point, '/leg_target', self.target_callback, 10)
        self.state_sub = self.create_subscription(JointState, '/joint_states', self.state_callback, 10)
        
        # Parameters
        self.declare_parameter('control_rate', 50.0)
        self.dt = 1.0 / self.get_parameter('control_rate').value
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        # Robot State
        self.current_joints = np.zeros(5) # [hip_pitch, hip_roll, hip_yaw, knee, ankle]
        self.target_pos = np.array([0.0, 0.0, -0.5]) # Default home position (0.5m down)
        self.has_state = False
        
        # Joint Limits (approximate from URDF)
        self.limits_lower = np.array([-2.21, -2.26, -1.57, -2.70, -0.22])
        self.limits_upper = np.array([1.04, 0.20, 1.57, 0.0, 1.25])
        
        # Current Solution (start with 0)
        self.q_sol = np.zeros(5) 
        
        self.get_logger().info('Simple IK Node Started. Waiting for /joint_states and /leg_target...')

    def state_callback(self, msg):
        # Map joint states to internal order
        # Assuming names: 
        # dof_right_hip_pitch_04, dof_right_hip_roll_03, dof_right_hip_yaw_03, dof_right_knee_04, dof_right_ankle_02
        names = [
            'dof_right_hip_pitch_04',
            'dof_right_hip_roll_03',
            'dof_right_hip_yaw_03',
            'dof_right_knee_04',
            'dof_right_ankle_02'
        ]
        
        try:
            indices = [msg.name.index(n) for n in names]
            self.current_joints = np.array([msg.position[i] for i in indices])
            if not self.has_state:
                self.q_sol = self.current_joints.copy()
                self.has_state = True
        except ValueError:
            pass

    def target_callback(self, msg):
        self.target_pos = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f'New Target: {self.target_pos}')

    def get_fk(self, q):
        # Forward Kinematics based on URDF parameters
        # q: [hip_pitch, hip_roll, hip_yaw, knee, ankle]
        
        # Transforms (extracted from URDF)
        # 1. Base -> Hip Pitch (Joint 1)
        # Origin: 0 0 0 (relative to base_link), RPY: 1.57 0 1.57
        T01 = self.transform(0,0,0, 1.57, 0, 1.57) @ self.revolute(q[0], 'z') # Axis 0 0 1
        
        # 2. Hip Pitch -> Hip Roll (Joint 2)
        # Link: KC_D_102R... (Hip Yoke)
        # Origin: -0.028 -0.030 -0.071, RPY: 3.14 -1.57 0
        T12 = self.transform(-0.028, -0.030, -0.071, 3.14, -1.57, 0) @ self.revolute(q[1], 'z') # Axis 0 0 -1 (handled by negative sign in q usually, or axis vector)
        # Note: URDF axis 0 0 -1 means rotation is negative.
        # self.revolute(q, 'z') does +rotation. So we pass -q[1].
        
        # 3. Hip Roll -> Hip Yaw (Joint 3)
        # Link: RS03_4
        # Origin: 0 0.143 0.024, RPY: -1.57 0 0
        T23 = self.transform(0, 0.143, 0.024, -1.57, 0, 0) @ self.revolute(-q[2], 'z') # Axis 0 0 -1
        
        # 4. Hip Yaw -> Knee (Joint 4)
        # Link: KC_D_301R... (Femur)
        # Origin: 0.0205 -0.021 0.212, RPY: 1.57 0 -1.57
        T34 = self.transform(0.0205, -0.021, 0.212, 1.57, 0, -1.57) @ self.revolute(q[3], 'z') # Axis 0 0 1
        
        # 5. Knee -> Ankle (Joint 5)
        # Link: KC_D_401R... (Shin)
        # Origin: -0.030 0.290 0.035, RPY: 0 0 0
        T45 = self.transform(-0.030, 0.290, 0.035, 0, 0, 0) @ self.revolute(q[4], 'z') # Axis 0 0 1
        
        # 6. Ankle -> Foot End Effector
        # Link: KB_D_501R... (Foot)
        # No joint after this, but we need the tip position.
        # Let's assume tip is at some offset from Ankle frame.
        # URDF inertial origin is -0.014 0.027 -0.016.
        # Let's assume tip is 0 0 0 of the foot link for now.
        
        T_total = T01 @ T12 @ T23 @ T34 @ T45
        pos = T_total[0:3, 3]
        return pos

    def transform(self, x, y, z, r, p, yw):
        # Translation
        T = np.eye(4)
        T[0:3, 3] = [x, y, z]
        
        # Rotation (RPY extrinsic)
        # Rx
        Rx = np.eye(4)
        Rx[0:3, 0:3] = [[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]]
        # Ry
        Ry = np.eye(4)
        Ry[0:3, 0:3] = [[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]]
        # Rz
        Rz = np.eye(4)
        Rz[0:3, 0:3] = [[np.cos(yw), -np.sin(yw), 0], [np.sin(yw), np.cos(yw), 0], [0, 0, 1]]
        
        # R = Rz * Ry * Rx (for fixed axis RPY)
        # URDF uses: intrinsic? No, usually sxyz (static).
        # "rpy" attribute in URDF corresponds to fixed-axis roll, then pitch, then yaw.
        # This is equivalent to R = Rz(y) * Ry(p) * Rx(r)
        
        R = Rz @ Ry @ Rx
        return T @ R # Translation then Rotation? No, usually Rotation then Translation in local frame?
        # URDF origin tag: "The reference frame of the child link with respect to the reference frame of the parent link."
        # It specifies the transform from Parent to Child Joint Frame.
        # T_parent_child = Translate * Rotate
        
        # So we apply Rotation relative to parent, then Translation?
        # Standard: T = Translation * Rotation (if R is specified in parent frame).
        # URDF RPY is "fixed axis roll, pitch, yaw".
        # So yes, T_joint = Translate(xyz) * Rotate(rpy)
        
        T_rot = np.eye(4)
        T_rot[0:3, 0:3] = R[0:3, 0:3]
        
        return T @ T_rot

    def revolute(self, angle, axis='z'):
        T = np.eye(4)
        c = np.cos(angle)
        s = np.sin(angle)
        if axis == 'z':
            T[0:2, 0:2] = [[c, -s], [s, c]]
        return T

    def solve_ik(self, target, q_init):
        q = q_init.copy()
        alpha = 0.1 # Learning rate
        max_iter = 50
        tol = 0.001
        
        for i in range(max_iter):
            curr_pos = self.get_fk(q)
            err = target - curr_pos
            if np.linalg.norm(err) < tol:
                break
                
            # Jacobian (Numerical)
            J = np.zeros((3, 5))
            eps = 1e-4
            for j in range(5):
                q_pert = q.copy()
                q_pert[j] += eps
                pos_pert = self.get_fk(q_pert)
                J[:, j] = (pos_pert - curr_pos) / eps
            
            # Dampened Pseudo-Inverse: J^T * (J * J^T + lambda*I)^-1
            # Or just pinv for simple cases
            lam = 0.01
            try:
                # J_pinv = np.linalg.pinv(J)
                # Damped
                J_pinv = J.T @ np.linalg.inv(J @ J.T + lam * np.eye(3))
            except np.linalg.LinAlgError:
                J_pinv = np.zeros((5, 3))
                
            dq = J_pinv @ err
            q += alpha * dq
            
            # Clamp limits
            q = np.clip(q, self.limits_lower, self.limits_upper)
            
        return q

    def control_loop(self):
        if not self.has_state:
            return

        # Solve IK
        self.q_sol = self.solve_ik(self.target_pos, self.q_sol)
        
        # Publish Command
        msg = LegCmd()
        msg.position_des = self.q_sol.tolist()
        msg.velocity_des = [0.0] * 5
        msg.feedforward_torque = [0.0] * 5
        msg.kp_scale = [10.0] * 5
        msg.kd_scale = [0.5] * 5
        
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
