import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from mm_interfaces.msg import TrajectoryDiff
from tf_transformations import euler_from_quaternion
import numpy as np
import math

from py_inv_kin_solver.slsqp_optimizer_with_dist_constraint import solve_ik_with_constraint as solve_ik, forward_kinematics

class ManipControlNode(Node):
    def __init__(self):
        super().__init__('manip_control_node')

        self.declare_parameter("robot_identity", "r2")
        self.robot_name = self.get_parameter("robot_identity").get_parameter_value().string_value

        self.delta_t = 0.2
        self.d_fixed = 0.5
        self.base_height = 0.1
        self.q_prev = np.array([0.0, 0.0, 0.0, 0.0])
        self.traj_received = False
        self.ee_trajectory = []

        self.current_base_position = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.base_cmd_vel = Twist()

        # Initialize r1 EE position with zeros
        self.r1_ee_position = np.array([0.0, 0.0, 0.0])

        self.create_subscriptions()
        self.create_publishers()

        self.timer = self.create_timer(self.delta_t, self.timer_callback)
        self.get_logger().info(f"Node {self.get_name()} has started")

    def create_subscriptions(self):
        self.create_subscription(TrajectoryDiff, f'/{self.robot_name}/ee_trajectory', self.read_trajectory_callback, 1)
        self.create_subscription(Odometry, f'/{self.robot_name}/odom', self.read_odometry_callback, 1)
        self.create_subscription(Twist, f'/{self.robot_name}/cmd_vel', self.read_cmd_vel_callback, 1)
        self.create_subscription(Point, '/r1/ee_pos', self.other_ee_pos_callback, 1)


    def create_publishers(self):
        self.traj_pub = self.create_publisher(JointTrajectory, f'/{self.robot_name}/arm_controller/joint_trajectory', 10)

    def read_trajectory_callback(self, msg):
        self.get_logger().info("Trajectory Received")
        self.traj_received = True
        self.ee_trajectory.clear()
        for goal in msg.trajectory:
            self.ee_trajectory.append(np.array([goal.x, goal.y, goal.z]))

    def read_odometry_callback(self, msg):
        self.current_base_position[0] = msg.pose.pose.position.x
        self.current_base_position[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)
        self.current_base_position[2] = yaw

    def read_cmd_vel_callback(self, msg):
        self.base_cmd_vel = msg

    def other_ee_pos_callback(self, msg):
        # Update fixed EE position from /r1/ee_pos topic
        self.r1_ee_position = np.array([msg.x, msg.y, msg.z])
        self.get_logger().debug(f"Updated fixed EE position: {self.r1_ee_position}")

    def timer_callback(self):
        next_base_pos = self.base_forward_kin_solver(self.current_base_position, self.base_cmd_vel)
        projected_base_pos = next_base_pos

        if self.traj_received and len(self.ee_trajectory) > 0:
            projected_traj_pos = self.closest_point_on_trajectory_to_vertical_line(self.ee_trajectory, projected_base_pos)

            x_ee = projected_traj_pos[0] - projected_base_pos[0]
            y_ee = projected_traj_pos[1] - projected_base_pos[1]
            z_ee = projected_traj_pos[2] - self.base_height

            # self.get_logger().info(f"Solving IK for EE position: [{x_ee:.3f}, {y_ee:.3f}, {z_ee:.3f}]")
            self.get_logger().info(f"Solving IK for EE position: [{projected_traj_pos[0]:.3f}, {projected_traj_pos[1]:.3f}, {projected_traj_pos[2]:.3f}]")

            joint_angles = solve_ik(projected_traj_pos[0], projected_traj_pos[1], projected_traj_pos[2], self.q_prev, self.r1_ee_position, self.d_fixed, projected_base_pos, self.base_height)
            # joint_angles = solve_ik(x_ee, y_ee, z_ee, self.q_prev, self.r1_ee_position, self.d_fixed, projected_base_pos, self.base_height)
            self.q_prev = joint_angles

            self.send_joint_command(joint_angles, duration_sec=self.delta_t)

            joint_angles_fk = joint_angles
            joint_angles_fk[1] = -1*joint_angles[1]
            joint_angles_fk[2] = -1*joint_angles[2]
            joint_angles_fk[3] = -1*joint_angles[3]

            est_ee_pos = forward_kinematics(joint_angles_fk)

            self.get_logger().info(f"Joint angles: {np.round(joint_angles, 6)}")
            self.get_logger().info(f"Estimated EE position: [{est_ee_pos[0]:.3f}, {est_ee_pos[1]:.3f}, {est_ee_pos[2]:.3f}]")

        self.current_base_position = next_base_pos

    def base_forward_kin_solver(self, current_pos, cmd_vel):
        dt = self.delta_t
        x, y, theta = current_pos
        v = cmd_vel.linear.x
        w = cmd_vel.angular.z

        new_x = x + v * math.cos(theta) * dt
        new_y = y + v * math.sin(theta) * dt
        new_theta = math.atan2(math.sin(theta + w * dt), math.cos(theta + w * dt))

        return np.array([new_x, new_y, new_theta])

    def closest_point_on_trajectory_to_vertical_line(self, trajectory, q):
        min_dist = float('inf')
        closest_point = np.zeros(3)

        for i in range(len(trajectory) - 1):
            p1 = trajectory[i]
            p2 = trajectory[i + 1]

            dx, dy, dz = p2 - p1
            t_num = np.dot(q - p1, p2 - p1)
            t_den = np.dot(p2 - p1, p2 - p1)
            t = t_num / t_den if t_den != 0 else 0.0
            t = max(0.0, min(1.0, t))

            proj = p1 + t * (p2 - p1)
            dist = np.linalg.norm(proj[:2] - q[:2])

            if dist < min_dist:
                min_dist = dist
                closest_point = proj

        self.get_logger().info(f"Closest point on trajectory: [{closest_point[0]:.3f}, {closest_point[1]:.3f}, {closest_point[2]:.3f}]")
        return closest_point

    def send_joint_command(self, joint_angles, duration_sec):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            f"{self.robot_name}/joint1",
            f"{self.robot_name}/joint2",
            f"{self.robot_name}/joint3",
            f"{self.robot_name}/joint4"
        ]

        # Adjust angles
        adjusted = []
        for i in range(4):
            adjusted.append(joint_angles[i])

        point = JointTrajectoryPoint()
        point.positions = adjusted
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)

        traj_msg.points.append(point)
        self.traj_pub.publish(traj_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ManipControlNode()
    rclpy.spin(node)
    rclpy.shutdown()
