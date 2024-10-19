#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "omni_control/go_to_goal.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<
    omni_control::GoToGoal> go_to_goal_node = std::make_shared<omni_control::GoToGoal>();
  go_to_goal_node->setAngularGain(2.0);
  go_to_goal_node->setLinearGain(0.5);
  go_to_goal_node->setGoalDistanceTolerance(0.1);
  rclcpp::spin(go_to_goal_node);
  rclcpp::shutdown();
  return 0;
}