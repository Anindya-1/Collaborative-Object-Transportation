#include "prop_nav/prop_nav_guiduance.hpp"

#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mm_interfaces/msg/trajectory_diff.hpp"

#include "tf2/utils.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

namespace prop_nav {

PropNavGuide::PropNavGuide() : Node("prop_nav_guidance")
    , no_goal_received_(false), nav_const_(1.0), tolerance_(1.0), trajectory_point_count(0), count(0), 
    previous_theta_LOS(0.0), delta_t(0.02), theta_LOS_rate(0.0), lin_speed(0.1)
{
    this->declare_parameter("config", 2);

    config = this->get_parameter("config").as_int();

    goal_.setZero();

    trajectory_subscription_ = this->create_subscription<mm_interfaces::msg::TrajectoryDiff>(
    "/trajectory", 1, std::bind(&PropNavGuide::readTrajectoryCallback, this, _1));
    
    if (config == 3){ 
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/gazebo_ground_truth/odom", 1, std::bind(&PropNavGuide::readOdometryCallback, this, _1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/omnidirectional_controller/cmd_vel_unstamped", 10);
    }
    else if(config == 2){
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/odom", 1, std::bind(&PropNavGuide::readOdometryCallback, this, _1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Incorrect Configuration");
    }
    
    RCLCPP_INFO(this->get_logger(), "Node %s has started", this->get_logger().get_name());
}

void PropNavGuide::setNavConst(double gain) {
    this->nav_const_ = gain;
    RCLCPP_INFO(this->get_logger(), "Navigation constant set to %.2f", this->nav_const_);
}

void PropNavGuide::setGoalDistanceTolerance(double tolerance) {
  this->tolerance_ = tolerance;
  RCLCPP_INFO(this->get_logger(), "Distance to goal tolerance set to %.2f", this->tolerance_);
}

void PropNavGuide::readTrajectoryCallback(const mm_interfaces::msg::TrajectoryDiff::SharedPtr traj){
    if(no_goal_received_ == true){
        for(auto goal:traj->trajectory){
            tf2::Vector3 waypoint;
            waypoint.setX(goal.x);
            waypoint.setY(goal.y);
            waypoint.setZ(goal.z);

            trajectory_.push_back(waypoint);
            trajectory_point_count++;
        }
        RCLCPP_INFO(this->get_logger(), "Trajectory has %d points", trajectory_point_count);
        if(trajectory_point_count > 0 && !trajectory_.empty()){
            no_goal_received_ = true;
            tf2::Vector3 initial_waypoint{trajectory_[0]};
            goal_.setX(initial_waypoint.getX());
            goal_.setY(initial_waypoint.getY());
            goal_.setY(initial_waypoint.getZ());
        }   
    }
}

void PropNavGuide::readOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom){
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.angular.z = 0.0;

    if (!no_goal_received_){
        cmd_vel_publisher_->publish(cmd_vel_);
        return;
    }

    // RCLCPP_INFO(this->get_logger(), "x: %.2f, y: %.2f, theta: %.2f",odom->pose.pose.position.x, odom->pose.pose.position.y, (odom->pose.pose.orientation.z * 57.295));

    tf2::Vector3 robot_position;
    tf2::Vector3 goal_position;

    double robot_yaw;
    robot_position.setX(odom->pose.pose.position.x);
    robot_position.setY(odom->pose.pose.position.y);
    robot_position.setZ(0);
    //robot_yaw = odom->pose.pose.orientation.z;
    // tf2::Quaternion q(
    //     odom->pose.pose.orientation.x,
    //     odom->pose.pose.orientation.y,
    //     odom->pose.pose.orientation.z,
    //     odom->pose.pose.orientation.w
    // );
    // double roll, pitch, yaw;
    // tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    // robot_yaw = yaw; // Use the yaw component



    // double goal_yaw;
    goal_position.setX(goal_.getX());
    goal_position.setY(goal_.getY());
    goal_position.setZ(0.0);
    // goal_yaw = goal_.getZ();    
    
    double distance_to_goal = goal_position.distance(robot_position);

    if (distance_to_goal < tolerance_) {
        if(count >= trajectory_point_count){
            RCLCPP_INFO(this->get_logger(), "Robot has reached its final position");
            no_goal_received_ = false;
            cmd_vel_publisher_->publish(cmd_vel_);
        }
        else{
            goal_ = trajectory_.at(count);
            RCLCPP_INFO(this->get_logger(), "Receiveid goal point: (%.2f, %.2f)",
            goal_.getX(), goal_.getY());
            count++;
            }
        return;
    }

    theta_LOS = atan2(goal_position.getY() - robot_position.getY(), goal_position.getX() - robot_position.getX());
    theta_LOS_rate = (theta_LOS - previous_theta_LOS) / delta_t;
    previous_theta_LOS = theta_LOS;
    // double yaw_error = goal_yaw - robot_yaw;

    // cmd_vel_.linear.x = nav_const_*(distance_to_goal * cos(theta_desired));
    // cmd_vel_.linear.y = nav_const_*(distance_to_goal * sin(theta_desired));
    // cmd_vel_.angular.z = nav_const_* (yaw_error + (theta_desired - robot_yaw));

    double ang_vel(0.0);
    // if(nav_const_* theta_LOS_rate > 0.5){
    //     ang_vel = 0.25;
    // }
    // else{
    //     ang_vel = nav_const_* theta_LOS_rate;
    // }
    ang_vel = nav_const_* theta_LOS_rate;

    if (config == 2){
        cmd_vel_.linear.x = lin_speed;
        cmd_vel_.angular.z = ang_vel;
    }
    else if(config == 3){
        cmd_vel_.linear.x = lin_speed * cos(theta_LOS);
        cmd_vel_.linear.y = lin_speed * sin(theta_LOS);
        cmd_vel_.angular.z = ang_vel;
    }
    else{
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.linear.y = 0.0;
        cmd_vel_.angular.z = 0.0;
    }

    cmd_vel_publisher_->publish(cmd_vel_);
}

PropNavGuide::~PropNavGuide() {};

}   // namespace prop_nav