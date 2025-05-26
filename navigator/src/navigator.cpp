#include "navigator/navigator.hpp"

#include <cmath>
#include <memory>

#include "tf2/utils.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace navigator {

Navigator::Navigator() : Node("navigator")
    , traj_loaded(false), nav_const_(1.0), tolerance_(1.0), 
    theta_LOS_rate(0.0), previous_theta_LOS(0.0), delta_t(0.2), trajectory_point_count(0), count(0), lin_speed(0.06), max_ang_vel_(1.0), ee_traj_level(0.325)
{
    goal_.setZero();

    this->declare_parameter("robot_identity", "r1");
    std::string robot_name = this->get_parameter("robot_identity").as_string();

    std::string trajectory_topic = "/" + robot_name + "/trajectory";
    std::string ee_trajectory_topic = "/" + robot_name + "/ee_trajectory";
    std::string base_cmd_topic = "/" + robot_name + "/cmd_vel";
    std::string base_odom_topic = "/" + robot_name + "/odom";
    std::string move_ahead_topic = "/" + robot_name + "/move_ahead";
    std::string reached_waypoint_topic = "/" + robot_name + "/reached_waypoint";

    // std::string trajectory_topic =  "/trajectory";
    // std::string ee_trajectory_topic =   "/ee_trajectory";
    // std::string base_cmd_topic = "/cmd_vel";
    // std::string base_odom_topic = "/odom";
    // std::string move_ahead_topic = "/move_ahead";
    // std::string reached_waypoint_topic =  "/reached_waypoint";

    trajectory_subscription_ = this->create_subscription<mm_interfaces::msg::TrajectoryDiff>(
    trajectory_topic, 1, std::bind(&Navigator::readTrajectoryCallback, this, std::placeholders::_1));

    ee_trajectory_publisher_ = this->create_publisher<mm_interfaces::msg::TrajectoryDiff>(ee_trajectory_topic, 10);

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        base_odom_topic, 1, std::bind(&Navigator::readOdometryCallback, this, std::placeholders::_1));

    move_ahead_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        move_ahead_topic, 1, std::bind(&Navigator::readMoveAheadCallback, this, std::placeholders::_1));

    move_ahead_ = true;

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(base_cmd_topic, 10);

    reached_waypoint_publisher_ = this->create_publisher<std_msgs::msg::Bool>(reached_waypoint_topic, 10);

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

        waypoint.setZ(ee_traj_level);

        ee_trajectory_.push_back(waypoint);

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

void Navigator::readMoveAheadCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    move_ahead_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Move ahead command: %s", move_ahead_ ? "true" : "false");
}

void Navigator::timerCallback() {
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.angular.z = 0.0;

    publish_ee_trajectory(ee_trajectory_);

    if (!move_ahead_) {
        // Robot is told to stop by sync node
        RCLCPP_DEBUG(this->get_logger(), "Waiting for move_ahead signal...");
        cmd_vel_publisher_->publish(cmd_vel_);
        return;
    }

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
        std_msgs::msg::Bool reached_msg;
        reached_msg.data = true;
        reached_waypoint_publisher_->publish(reached_msg);

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
    double delta_theta = angles::shortest_angular_distance(previous_theta_LOS, theta_LOS);
    theta_LOS_rate = delta_theta / delta_t;

    // theta_LOS_rate = (theta_LOS - previous_theta_LOS) / delta_t;

    previous_theta_LOS = theta_LOS;

    double ang_vel(0.0);

    ang_vel = nav_const_* theta_LOS_rate;
    ang_vel = std::clamp(ang_vel, -max_ang_vel_, max_ang_vel_);

    cmd_vel_.linear.x = lin_speed;
    cmd_vel_.angular.z = ang_vel;

    cmd_vel_publisher_->publish(cmd_vel_);
}

void Navigator::publish_ee_trajectory(std::vector<tf2::Vector3> ee_traj){
    std::vector<geometry_msgs::msg::Vector3> ee_trajectory;
    for(auto goal:ee_traj){
        geometry_msgs::msg::Vector3 waypoint;
        waypoint.x = goal.x();
        waypoint.y = goal.y();
        waypoint.z = goal.z();

        ee_trajectory.push_back(waypoint);
    }
    mm_interfaces::msg::TrajectoryDiff ee_trajectory_msg;
    ee_trajectory_msg.trajectory = ee_trajectory;
    ee_trajectory_publisher_->publish(ee_trajectory_msg);
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