#include "inv_kin_solver/inv_kin_node.hpp"

namespace inv_kin_node {

InvKinNode::InvKinNode() : Node("inv_kin_node")
    ,delta_t(0.2), traj_received_(false)
{   
    q_prev = "[0,0,0,0]";

    this->declare_parameter("robot_identity", "r1");
    robot_name = this->get_parameter("robot_identity").as_string();

    std::string trajectory_topic = "/" + robot_name + "/trajectory";
    std::string ee_trajectory_topic = "/" + robot_name + "/ee_trajectory";
    std::string base_cmd_topic = "/" + robot_name + "/cmd_vel";
    std::string base_odom_topic = "/" + robot_name + "/odom";
    std::string arm_controller_topic = "/" + robot_name + "/arm_controller/joint_trajectory";

    trajectory_subscription_ = this->create_subscription<mm_interfaces::msg::TrajectoryDiff>(
    ee_trajectory_topic, 1, std::bind(&InvKinNode::readTrajectoryCallback, this, std::placeholders::_1));

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        base_odom_topic, 1, std::bind(&InvKinNode::readOdometryCallback, this, std::placeholders::_1));

    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        base_cmd_topic, 1, std::bind(&InvKinNode::readCmdVelCallback, this, std::placeholders::_1));

    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        arm_controller_topic, 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(delta_t*1000)),
        std::bind(&InvKinNode::timerCallback, this));
        
    RCLCPP_INFO(this->get_logger(), "Node %s has started", this->get_logger().get_name());
}

void InvKinNode::readTrajectoryCallback(const mm_interfaces::msg::TrajectoryDiff::SharedPtr traj){
    RCLCPP_INFO(this->get_logger(), "Trajectory Received");
    traj_received_ = true;
    for(auto goal:traj->trajectory){
        tf2::Vector3 waypoint;
        waypoint.setX(goal.x);
        waypoint.setY(goal.y);
        waypoint.setZ(goal.z);

        ee_trajectory_.push_back(waypoint);
    }
}

void InvKinNode::timerCallback(){
    next_base_position = base_for_kin_solver(current_base_position, base_cmd_vel);
    projected_base_pos = next_base_position;
    if(traj_received_){
        projected_traj_pos = closest_point_on_trajectory_to_vertical_line(ee_trajectory_, projected_base_pos);

        x_ee_pos = projected_traj_pos.x - projected_base_pos.x();
        y_ee_pos = projected_traj_pos.y - projected_base_pos.y();
        z_ee_pos = projected_traj_pos.z - 0.1;  // mobile base height = 0.1 m

        RCLCPP_INFO(this->get_logger(), "Solving IK for EE position: [%.3f, %.3f, %.3f]", x_ee_pos, y_ee_pos, z_ee_pos);

        auto start = std::chrono::high_resolution_clock::now();

        // Call the IK solver
        joint_angles = solve_Ik(x_ee_pos, y_ee_pos, z_ee_pos, q_prev);

        q_prev = joint_angles;

        send_joint_command(joint_angles);

        auto end = std::chrono::high_resolution_clock::now();

        duration = end - start;

        // Call the FK solver
        est_ee_pos = forward_kinematics(joint_angles);

        // Log the solution
        std::string result_str = joint_angles.tostring(6);
        RCLCPP_INFO(this->get_logger(), "Joint angles: %s", result_str.c_str());
        RCLCPP_INFO(this->get_logger(), "IK computation took %.6f seconds", duration.count());
        RCLCPP_INFO(this->get_logger(), "Estimated EE position: [%.3f, %.3f, %.3f]", est_ee_pos.at(0), est_ee_pos.at(1), est_ee_pos.at(2));
    }
}

void InvKinNode::readOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom){
    current_base_position.setX(odom->pose.pose.position.x);
    current_base_position.setY(odom->pose.pose.position.y);
    current_base_position.setZ(odom->pose.pose.orientation.z);
}

void InvKinNode::readCmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel){
    base_cmd_vel.linear.set__x(cmd_vel->linear.x);
    base_cmd_vel.angular.set__z(cmd_vel->angular.z);
}

// Forward Kinematic Solver for Mobile Base
tf2::Vector3 InvKinNode::base_for_kin_solver(tf2::Vector3 current_pos, geometry_msgs::msg::Twist cmd_vel){
    // Parameters
    double dt = delta_t;

    // Current position and orientation
    double x = current_pos.x();
    double y = current_pos.y();
    double theta = current_pos.z();  // Assuming theta (yaw) is stored in z()

    // Velocities
    double v = cmd_vel.linear.x;     // Linear velocity
    double w = cmd_vel.angular.z;    // Angular velocity (yaw rate)

    // Forward Kinematics update
    double new_x = x + v * std::cos(theta) * dt;
    double new_y = y + v * std::sin(theta) * dt;
    double new_theta = theta + w * dt;

    // Normalize theta to [-pi, pi]
    new_theta = std::atan2(std::sin(new_theta), std::cos(new_theta));

    return tf2::Vector3(new_x, new_y, new_theta);
}

geometry_msgs::msg::Point InvKinNode::closest_point_on_trajectory_to_vertical_line(
    const std::vector<tf2::Vector3>& trajectory, const tf2::Vector3& q){
    double min_dist = std::numeric_limits<double>::max();
    geometry_msgs::msg::Point closest_point;

    for (size_t i = 0; i < trajectory.size() - 1; ++i) {
        const auto& p1 = trajectory[i];
        const auto& p2 = trajectory[i + 1];

        // Direction vector of the segment
        double dx = p2.x() - p1.x();
        double dy = p2.y() - p1.y();
        double dz = p2.z() - p1.z();

        // Vector from p1 to the projection of q on xy-plane
        double t_num = (q.x() - p1.x()) * dx + (q.y() - p1.y()) * dy + (q.z() - p1.z()) * dz;
        double t_den = dx * dx + dy * dy + dz * dz;
        double t = t_num / t_den;

        // Clamp t to [0, 1] to stay within the segment
        t = std::max(0.0, std::min(1.0, t));

        // Point on the segment
        geometry_msgs::msg::Point proj;
        proj.x = p1.x() + t * dx;
        proj.y = p1.y() + t * dy;
        proj.z = p1.z() + t * dz;

        // Distance from proj to the vertical line through q (only x and y matter)
        double dist = std::hypot(proj.x - q.x(), proj.y - q.y());

        if (dist < min_dist) {
            min_dist = dist;
            closest_point = proj;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Closest point on trajectory: [%.3f, %.3f, %.3f]", closest_point.x, closest_point.y, closest_point.z);

    return closest_point;
}

void InvKinNode::send_joint_command(const alglib::real_1d_array& joint_angles, double duration_sec){
    trajectory_msgs::msg::JointTrajectory traj_msg;

    traj_msg.joint_names = {robot_name + "/" + "joint1", robot_name + "/" + "joint2", robot_name + "/" + "joint3", robot_name + "/" + "joint4"};

    std::vector<double> joint_positions;
    for(int i{0}; i < 4; i++){
        if(i == 1){
            joint_positions.push_back(joint_angles[i]-1.57);    
        }
        else if(i == 2){
            joint_positions.push_back(joint_angles[i]+1.57);
        }
        else{
        joint_positions.push_back(joint_angles[i]);
        }
    }

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = joint_positions;
    point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);

    traj_msg.points.push_back(point);

    traj_pub_->publish(traj_msg);
}

InvKinNode::~InvKinNode() {};

} // namespace inv_kin_node

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<inv_kin_node::InvKinNode> node = std::make_shared<inv_kin_node::InvKinNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
