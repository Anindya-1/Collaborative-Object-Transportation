#ifndef OMNI_CONTROL__REACH_GOAL_HPP_
#define OMNI_CONTROL__REACH_GOAL_HPP_

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Vector3.h"

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
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

 private:
  void readOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void readGoalPointCallback(const geometry_msgs::msg::Vector3::SharedPtr goal);

  geometry_msgs::msg::Twist cmd_vel_;
  tf2::Vector3 goal_;
  bool no_goal_received_;
  double angular_gain_;
  double linear_x_gain_;
  double linear_y_gain_;
  double tolerance_;
  double theta_desired;
};

}  // namespace omni_control_1

#endif  // OMNI_CONTROL__REACH_GOAL_HPP_