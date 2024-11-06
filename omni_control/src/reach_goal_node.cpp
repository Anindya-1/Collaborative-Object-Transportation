#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "omni_control/reach_goal.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<
    omni_control_1::ReachGoal> reach_goal_node = std::make_shared<omni_control_1::ReachGoal>();
  reach_goal_node->declare_parameter("ang_gain", 2.0);
  reach_goal_node->declare_parameter("lin_gain", 0.5);

  double ang_gain = reach_goal_node->get_parameter("ang_gain").as_double();
  double lin_gain = reach_goal_node->get_parameter("lin_gain").as_double();

  reach_goal_node->setAngularGain(ang_gain);
  reach_goal_node->setLinearXGain(lin_gain);
  reach_goal_node->setLinearYGain(lin_gain);
  reach_goal_node->setGoalDistanceTolerance(0.1);
  rclcpp::spin(reach_goal_node);
  rclcpp::shutdown();
  return 0;
}

ros2 topic pub /trajectory mm_interfaces/msg/Trajectory2D "trajectory:
- {waypoint:{x: 0.0, y: 0.0, z: 0.0}, yaw: 0.0}
- {waypoint:{x: 0.25, y: 0.25, z: 0.0}, yaw: 0.784}
- {waypoint:{x: 0.5, y: 0.5, z: 0.0}, yaw: 0.784}
- {waypoint:{x: 0.75, y: 0.75, z: 0.0}, yaw: 0.784}
- {waypoint:{x: 1.0, y: 1.0, z: 0.0}, yaw: 0.784}
- {waypoint:{x: 1.25, y: 1.25, z: 0.0}, yaw: 0.784}
- {waypoint:{x: 1.5, y: 1.5, z: 0.0}, yaw: 0.784}"