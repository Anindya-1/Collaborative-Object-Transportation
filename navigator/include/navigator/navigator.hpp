#ifndef NAVIGATOR__NAVIGATOR_HPP_
#define NAVIGATOR__NAVIGATOR_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "mm_interfaces/msg/trajectory_diff.hpp"

namespace navigator {

class Navigator : public rclcpp::Node {
public:
    Navigator();
    ~Navigator();
    void setNavConst(double gain);
    void setGoalDistanceTolerance(double tolerance);

protected:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<mm_interfaces::msg::TrajectoryDiff>::SharedPtr trajectory_subscription_;
    rclcpp::Publisher<mm_interfaces::msg::TrajectoryDiff>::SharedPtr ee_trajectory_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

private:
    void readOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void readTrajectoryCallback(const mm_interfaces::msg::TrajectoryDiff::SharedPtr traj);
    void trajectoryReader(const mm_interfaces::msg::TrajectoryDiff::SharedPtr traj);
    void timerCallback();

    void publish_ee_trajectory(std::vector<tf2::Vector3> ee_traj);

    geometry_msgs::msg::Twist cmd_vel_;
    tf2::Vector3 goal_;
    tf2::Vector3 robot_position;
    std::vector<tf2::Vector3> trajectory_;
    std::vector<tf2::Vector3> ee_trajectory_;   
    bool traj_loaded;
    double nav_const_;
    double tolerance_;
    double theta_LOS_rate;
    double previous_theta_LOS;
    double delta_t;
    int trajectory_point_count;
    int count;
    double lin_speed;

    double ee_traj_level;
};

} // namespace navigator

#endif  // NAVIGATOR__NAVIGATOR_HPP_