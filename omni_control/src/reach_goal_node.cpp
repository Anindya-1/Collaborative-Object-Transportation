#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "omni_control/reach_goal.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<
    omni_control_1::ReachGoal> reach_goal_node = std::make_shared<omni_control_1::ReachGoal>();
  reach_goal_node->setAngularGain(0.5);
  reach_goal_node->setLinearXGain(2.0);
  reach_goal_node->setLinearYGain(2.0);
  reach_goal_node->setGoalDistanceTolerance(0.1);
  rclcpp::spin(reach_goal_node);
  rclcpp::shutdown();
  return 0;
}