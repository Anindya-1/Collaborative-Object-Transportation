#ifndef OMNI_CONTROL__REACH_GOAL_HPP_
#define OMNI_CONTROL__REACH_GOAL_HPP_

#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "mm_interfaces/msg/trajectory2_d.hpp"

namespace omni_control_1 {

class ReachGoal : public rclcpp::Node {
 public:
  ReachGoal();
  ~ReachGoal();
  void setAngularGain(double gain);
  void setLinearXGain(double gain);
  void setLinearYGain(double gain);
  void setGoalDistanceTolerance(double tolerance);

 protected:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr goal_subscription_;
  rclcpp::Subscription<mm_interfaces::msg::Trajectory2D>::SharedPtr trajectory_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

 private:
  void readTrajectoryCallback(const mm_interfaces::msg::Trajectory2D::SharedPtr msg);
  void readOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void readGoalPointCallback(const geometry_msgs::msg::Vector3::SharedPtr goal);  

  std::vector<tf2::Vector3> trajectory;
  geometry_msgs::msg::Twist cmd_vel_;
  tf2::Vector3 goal_;
  bool no_goal_received_;
  double angular_gain_;
  double linear_x_gain_;
  double linear_y_gain_;
  double tolerance_;
  double theta_desired;
  int trajectory_point_count;
  int cnt;
};

}  // namespace omni_control_1

#endif  // OMNI_CONTROL__REACH_GOAL_HPP_