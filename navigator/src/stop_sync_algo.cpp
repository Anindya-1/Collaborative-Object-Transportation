#include "navigator/stop_sync_algo.hpp"

namespace stop_sync_algo {

StopSyncAlgo::StopSyncAlgo() : Node("StopSyncAlgoNode"), 
    delta_t(0.2)
{
    r1_waypoint_reached_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
    "/r1/reached_waypoint", 1, std::bind(&StopSyncAlgo::readWaypointReached_r1, this, std::placeholders::_1));

    r2_waypoint_reached_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
    "/r2/reached_waypoint", 1, std::bind(&StopSyncAlgo::readWaypointReached_r2, this, std::placeholders::_1));

    r1_move_ahead_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/r1/move_ahead", 10);

    r2_move_ahead_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/r2/move_ahead", 10);

    // timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(static_cast<int>(delta_t*1000)),
    //     std::bind(&StopSyncAlgo::timerCallback, this));
        
    RCLCPP_INFO(this->get_logger(), "Node %s has started", this->get_logger().get_name());
}

void StopSyncAlgo::readWaypointReached_r1(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        r1_reached_ = true;

        // Stop robot r1 immediately
        std_msgs::msg::Bool stop_msg;
        stop_msg.data = false;
        r1_move_ahead_publisher_->publish(stop_msg);

        // If both reached, allow both to move
        if (r2_reached_) {
            std_msgs::msg::Bool go_msg;
            go_msg.data = true;
            r1_move_ahead_publisher_->publish(go_msg);
            r2_move_ahead_publisher_->publish(go_msg);

            // Reset flags
            r1_reached_ = false;
            r2_reached_ = false;
        }
    }
}

void StopSyncAlgo::readWaypointReached_r2(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        r2_reached_ = true;

        // Stop robot r2 immediately
        std_msgs::msg::Bool stop_msg;
        stop_msg.data = false;
        r2_move_ahead_publisher_->publish(stop_msg);

        // If both reached, allow both to move
        if (r1_reached_) {
            std_msgs::msg::Bool go_msg;
            go_msg.data = true;
            r1_move_ahead_publisher_->publish(go_msg);
            r2_move_ahead_publisher_->publish(go_msg);

            // Reset flags
            r1_reached_ = false;
            r2_reached_ = false;
        }
    }
}

StopSyncAlgo::~StopSyncAlgo() {};

}   // namespace stop_sync_algo


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    std::shared_ptr<stop_sync_algo::StopSyncAlgo> stop_sync_algo_node = std::make_shared<stop_sync_algo::StopSyncAlgo>();
    rclcpp::spin(stop_sync_algo_node);
    rclcpp::shutdown();
    return 0;
}