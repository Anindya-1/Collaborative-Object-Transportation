#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "prop_nav/prop_nav_guiduance.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    std::shared_ptr<prop_nav::PropNavGuide> prop_nav_node = std::make_shared<prop_nav::PropNavGuide>();
    prop_nav_node->setNavConst(1.0);
    prop_nav_node->setGoalDistanceTolerance(0.1);
    rclcpp::spin(prop_nav_node);
    rclcpp::shutdown();
    return 0;
}


/*
ros2 topic pub /trajectory mm_interfaces/msg/TrajectoryDiff "{trajectory: [
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
