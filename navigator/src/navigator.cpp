#include "navigator/navigator.hpp"

#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mm_interfaces/msg/trajectory_diff.hpp"

#include "tf2/utils.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace navigator {

Navigator::Navigator() : Node("navigator")
    , traj_loaded(false), nav_const_(1.0), tolerance_(1.0), 
    theta_LOS_rate(0.0), previous_theta_LOS(0.0), delta_t(0.02), trajectory_point_count(0), count(0), lin_speed(0.1)
{
    goal_.setZero();

    this->declare_parameter("robot_identity", "robot1");
    std::string robot_name = this->get_parameter("robot_identity").as_string();

    std::string trajectory_topic = "/" + robot_name + "/trajectory";

    trajectory_subscription_ = this->create_subscription<mm_interfaces::msg::TrajectoryDiff>(
    trajectory_topic, 1, std::bind(&Navigator::readTrajectoryCallback, this, std::placeholders::_1));

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 1, std::bind(&Navigator::readOdometryCallback, this, std::placeholders::_1));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(delta_t*1000)),
        std::bind(&Navigator::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "Node %s has started", this->get_logger().get_name());
}

void Navigator::setNavConst(double gain) {
    this->nav_const_ = gain;
    RCLCPP_INFO(this->get_logger(), "Navigation constant set to %.2f", this->nav_const_);
}

void Navigator::setGoalDistanceTolerance(double tolerance) {
  this->tolerance_ = tolerance;
  RCLCPP_INFO(this->get_logger(), "Distance to goal tolerance set to %.2f", this->tolerance_);
}

void Navigator::readTrajectoryCallback(const mm_interfaces::msg::TrajectoryDiff::SharedPtr traj){
    RCLCPP_INFO(this->get_logger(), "Trajectory Received");
    if(traj_loaded == false){
        trajectory_.clear();
        count = 0;
        trajectory_point_count = 0;
        trajectoryReader(traj);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Trajectory already loaded");
    }
}

void Navigator::trajectoryReader(const mm_interfaces::msg::TrajectoryDiff::SharedPtr traj){
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
        traj_loaded = true;
        tf2::Vector3 initial_waypoint{trajectory_[0]};
        goal_.setX(initial_waypoint.getX());
        goal_.setY(initial_waypoint.getY());
        goal_.setZ(initial_waypoint.getZ());
    }   
}

void Navigator::readOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom){

    robot_position.setX(odom->pose.pose.position.x);
    robot_position.setY(odom->pose.pose.position.y);
    robot_position.setZ(0);
    
}

void Navigator::timerCallback() {
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.angular.z = 0.0;

    if (traj_loaded == false) {
        // RCLCPP_INFO(this->get_logger(), "Trajectory not loaded yet");
        cmd_vel_publisher_->publish(cmd_vel_);
        return;
    }

    tf2::Vector3 goal_position;

    goal_position.setX(goal_.getX());
    goal_position.setY(goal_.getY());
    goal_position.setZ(0.0);
    
    double distance_to_goal = goal_position.distance(robot_position);

    if (distance_to_goal < tolerance_) {
        if(count >= trajectory_point_count){
            RCLCPP_INFO(this->get_logger(), "Robot has reached its final position");
            traj_loaded = false;
            cmd_vel_publisher_->publish(cmd_vel_);
        }
        else{
            goal_ = trajectory_.at(count);
            RCLCPP_INFO(this->get_logger(), "Received goal point: (%.2f, %.2f)",
                        goal_.getX(), goal_.getY());
            count++;    
        }
        return;
    }

    double theta_LOS;

    theta_LOS = atan2(goal_position.getY() - robot_position.getY(), goal_position.getX() - robot_position.getX());
    theta_LOS_rate = (theta_LOS - previous_theta_LOS) / delta_t;
    previous_theta_LOS = theta_LOS;

    double ang_vel(0.0);

    ang_vel = nav_const_* theta_LOS_rate;

    cmd_vel_.linear.x = lin_speed;
    cmd_vel_.angular.z = ang_vel;

    cmd_vel_publisher_->publish(cmd_vel_);
}

Navigator::~Navigator() {};

}   // namespace navigator


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    std::shared_ptr<navigator::Navigator> navigator_node = std::make_shared<navigator::Navigator>();
    navigator_node->setNavConst(5.0);
    navigator_node->setGoalDistanceTolerance(0.1);
    rclcpp::spin(navigator_node);
    rclcpp::shutdown();
    return 0;
}



/*
ros2 topic pub --once /trajectory mm_interfaces/msg/TrajectoryDiff "{trajectory: [
  {x: 0.0, y: 0.0, z: 0.0},
  {x: 0.25, y: 0.25, z: 0.0},
  {x: 0.5, y: 0.5, z: 0.0},
  {x: 0.75, y: 0.0, z: 0.0},
  {x: 0.5, y: -0.5, z: 0.0},
  {x: 0.0, y: -0.75, z: 0.0},
  {x: -0.5, y: -0.5, z: 0.0},
  {x: -0.75, y: 0.0, z: 0.0},
  {x: -0.5, y: 0.5, z: 0.0},
  {x: 0.0, y: 0.0, z: 0.0}
]}"
*/