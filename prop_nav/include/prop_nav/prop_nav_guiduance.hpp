#ifndef PROP_NAV__PROP_NAV_GUIDANCE_HPP_
#define PROP_NAV__PROP_NAV_GUIDANCE_HPP_

#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "mm_interfaces/msg/trajectory_diff.hpp"

namespace prop_nav {

class PropNavGuide : public rclcpp::Node {
public:
    PropNavGuide();
    ~PropNavGuide();
    void setNavConst(double gain);
    void setGoalDistanceTolerance(double tolerance);
    void setDriveConfiguration(int config);

protected:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<mm_interfaces::msg::TrajectoryDiff>::SharedPtr trajectory_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

private:
    void readOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void readTrajectoryCallback(const mm_interfaces::msg::TrajectoryDiff::SharedPtr traj);

    geometry_msgs::msg::Twist cmd_vel_;
    tf2::Vector3 goal_;
    std::vector<tf2::Vector3> trajectory_;
    bool no_goal_received_;
    double nav_const_;
    double tolerance_;
    double theta_LOS;
    double theta_LOS_rate;
    double previous_theta_LOS;
    double delta_t;
    int trajectory_point_count;
    int count;
    int config;
    double lin_speed;
};

} // namespace prop_nav

#endif  // PROP_NAV__PROP_NAV_GUIDANCE_HPP_