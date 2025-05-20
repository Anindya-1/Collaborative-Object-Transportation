#ifndef NAVIGATOR__NAVIGATOR_HPP_
#define NAVIGATOR__NAVIGATOR_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "mm_interfaces/msg/trajectory_diff.hpp"
#include "mm_interfaces/action/trajectory2_d.hpp"

using Trajectory2D = mm_interfaces::action::Trajectory2D;
using Trajectory2DGoalHandle = rclcpp_action::ServerGoalHandle<Trajectory2D>;

namespace navigator {

class Navigator : public rclcpp::Node {
public:
    Navigator();
    ~Navigator();
    void setNavConst(double gain);
    void setGoalDistanceTolerance(double tolerance);

protected:
    rclcpp_action::Server<Trajectory2D>::SharedPtr trajectory_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    // rclcpp::Subscription<mm_interfaces::msg::TrajectoryDiff>::SharedPtr trajectory_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    //rclcpp::TimerBase::SharedPtr timer_;

private:
    void readOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    // void readTrajectoryCallback(const mm_interfaces::msg::TrajectoryDiff::SharedPtr traj);
    // void trajectoryReader(const mm_interfaces::msg::TrajectoryDiff::SharedPtr traj);
    //void timerCallback();

    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Trajectory2D::Goal> goal);
    rclcpp_action::CancelResponse cancel_callback(std::shared_ptr<Trajectory2DGoalHandle> goal_handle);
    void handle_accepted_callback(std::shared_ptr<Trajectory2DGoalHandle> goal_handle);
    void execute_goal(std::shared_ptr<Trajectory2DGoalHandle> goal_handle);

    geometry_msgs::msg::Twist cmd_vel_;
    tf2::Vector3 goal_;
    tf2::Vector3 robot_position;
    std::vector<tf2::Vector3> trajectory_;
    bool traj_loaded;
    double nav_const_;
    double tolerance_;
    double theta_LOS_rate;
    double previous_theta_LOS;
    double delta_t;
    int trajectory_point_count;
    int count;
    double lin_speed;
};

} // namespace navigator

#endif  // NAVIGATOR__NAVIGATOR_HPP_