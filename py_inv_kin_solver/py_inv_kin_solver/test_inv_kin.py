#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tf_transformations import euler_from_quaternion
import numpy as np
import math
import time

from py_inv_kin_solver.slsqp_optimizer import solve_ik_with_constraint as solve_ik, forward_kinematics

class TestInvKin(Node):
    def __init__(self):
        super().__init__('test_inv_kin_node')

        self.declare_parameter('x_ee', 0.38)
        self.declare_parameter('y_ee', 0.0)
        self.declare_parameter('z_ee', 0.077)

        self.x_ee = self.get_parameter('x_ee').value
        self.y_ee = self.get_parameter('y_ee').value
        self.z_ee = self.get_parameter('z_ee').value

        self.q_prev = [0.0, 0.0, 0.0, 0.0]
        self.delta_t = 0.2

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.traj_pub = self.create_publisher(JointTrajectory, '/r1/arm_controller/joint_trajectory', 10)

        self.timer = self.create_timer(self.delta_t, self.timer_callback)

        self.base_pose = [0.0, 0.0, 0.0]  # x, y, theta
        self.base_cmd = Twist()

        self.get_logger().info('Python IK Node has started.')

    def timer_callback(self):
        self.get_logger().info(f"Solving IK for EE position: [{self.x_ee:.3f}, {self.y_ee:.3f}, {self.z_ee:.3f}]")
        start_time = time.time()

        try:
            joint_angles = solve_ik(self.x_ee, self.y_ee, self.z_ee, self.q_prev)
        except Exception as e:
            self.get_logger().error(f"Optimization failed: {e}")
            return

        self.q_prev = joint_angles

        self.send_joint_command(joint_angles)

        duration = time.time() - start_time
        joint_angles_fk = joint_angles
        joint_angles_fk[1] = -1*joint_angles[1]
        joint_angles_fk[2] = -1*joint_angles[2]
        joint_angles_fk[3] = -1*joint_angles[3]
        est_pos = forward_kinematics(joint_angles_fk)
        self.get_logger().info(f"Joint angles: {np.round(joint_angles, 6).tolist()}")
        self.get_logger().info(f"IK took {duration:.6f} seconds")
        self.get_logger().info(f"Estimated EE position: [{est_pos[0]:.3f}, {est_pos[1]:.3f}, {est_pos[2]:.3f}]")

    def send_joint_command(self, joint_angles):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['r1/joint1', 'r1/joint2', 'r1/joint3', 'r1/joint4']

        point = JointTrajectoryPoint()
        point.positions = joint_angles.tolist()


        traj_msg.points.append(point)
        self.traj_pub.publish(traj_msg)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.base_pose = [x, y, yaw]

    def cmd_vel_callback(self, msg):
        self.base_cmd = msg

def main(args=None):
    rclpy.init(args=args)
    node = TestInvKin()
    rclpy.spin(node)
    rclpy.shutdown()