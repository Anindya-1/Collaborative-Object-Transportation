#ifndef INV_KIN_NODE_HPP
#define INV_KIN_NODE_HPP

#include "rclcpp/rclcpp.hpp"

#include <chrono>  
#include <string>
#include <vector>
#include "optimizer_SQP.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "mm_interfaces/msg/trajectory_diff.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace inv_kin_node {

class InvKinNode : public rclcpp::Node
{
public:
    InvKinNode();
    ~InvKinNode();

protected:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<mm_interfaces::msg::TrajectoryDiff>::SharedPtr trajectory_subscription_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
private:
    void readOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void readCmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);
    void readTrajectoryCallback(const mm_interfaces::msg::TrajectoryDiff::SharedPtr traj);
    void trajectoryReader(const mm_interfaces::msg::TrajectoryDiff::SharedPtr traj);
    void timerCallback();

    tf2::Vector3 base_for_kin_solver(tf2::Vector3 current_pos, geometry_msgs::msg::Twist cmd_vel);
    geometry_msgs::msg::Point closest_point_on_trajectory_to_vertical_line(
        const std::vector<tf2::Vector3>& trajectory,
        const tf2::Vector3& q);

    void send_joint_command(const alglib::real_1d_array& joint_angles, double duration_sec=0.1);

    alglib::real_1d_array joint_angles;
    std::chrono::duration<double> duration;
    double x_ee_pos, y_ee_pos, z_ee_pos;
    std::vector<double> est_ee_pos;

    tf2::Vector3 current_base_position;
    tf2::Vector3 next_base_position;
    geometry_msgs::msg::Twist base_cmd_vel;
    tf2::Vector3 projected_base_pos;
    geometry_msgs::msg::Point projected_traj_pos;

    std::vector<tf2::Vector3> ee_trajectory_;

    trajectory_msgs::msg::JointTrajectory traj_msg;

    double delta_t;
    bool traj_received_;

    alglib::real_1d_array q_prev;

    std::string robot_name;
};

} // namespace inv_kin_node

#endif // INV_KIN_NODE_HPP