#ifndef NAVIGATOR__STOP_SYNC_ALGO_HPP_
#define NAVIGATOR__STOP_SYNC_ALGO_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"


namespace stop_sync_algo {

class StopSyncAlgo : public rclcpp::Node {
public:
    StopSyncAlgo();
    ~StopSyncAlgo();

protected:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr r1_waypoint_reached_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr r2_waypoint_reached_subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr r1_move_ahead_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr r2_move_ahead_publisher_;
    // rclcpp::TimerBase::SharedPtr timer_;

private:
    void readWaypointReached_r1(const std_msgs::msg::Bool::SharedPtr r1_reached_status);
    void readWaypointReached_r2(const std_msgs::msg::Bool::SharedPtr r2_reached_status);
    // void timerCallback();

    double delta_t;

    bool r1_reached_{false};
    bool r2_reached_{false};

    
};

} // namespace stop_sync_algo

#endif  // NAVIGATOR__STOP_SYNC_ALGO_HPP_