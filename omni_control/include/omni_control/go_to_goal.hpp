#ifndef OMNI_CONTROL__GO_TO_GOAL_HPP_
#define OMNI_CONTROL__GO_TO_GOAL_HPP_

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Vector3.h"

namespace omni_control {

class GoToGoal : public rclcpp::Node {
 public:
  GoToGoal();
  ~GoToGoal();
  void setAngularGain(double gain);
  void setLinearGain(double gain);
  void setGoalDistanceTolerance(double tolerance);

 protected:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr goal_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

 private:
  void readOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void readGoalPointCallback(const geometry_msgs::msg::Vector3::SharedPtr goal);

  geometry_msgs::msg::Twist cmd_vel_;
  tf2::Vector3 goal_;
  bool no_goal_received_;
  double angular_gain_;
  double linear_gain_;
  double tolerance_;
};

}  // namespace omni_control

#endif  // OMNI_CONTROL__GO_TO_GOAL_HPP_