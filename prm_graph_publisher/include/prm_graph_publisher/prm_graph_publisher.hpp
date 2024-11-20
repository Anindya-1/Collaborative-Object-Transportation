#ifndef PRM_GRAPH_PUBLISHER_HPP
#define PRM_GRAPH_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "example_interfaces/msg/float64_multi_array.hpp"
// #include "mm_interfaces/msg/pair.hpp"

// // Define the Pair message type (for edges)
// namespace mm_interfaces {
// namespace msg {
// struct Pair {
//   double a;
//   double b;
// };
// } // namespace msg
// } // namespace mm_interfaces

class PRMGraphPublisher : public rclcpp::Node {
public:
  PRMGraphPublisher();
  
  void generatePRM(size_t num_samples, size_t k_neighbors, const std::vector<std::vector<double>>& obstacles);
  bool isCollisionFree(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2, const std::vector<std::vector<double>>& obstacles);

private:
  void runPRM();

  // Helper functions
  geometry_msgs::msg::Point sampleRandomPoint();
  double euclideanDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);

  // Publishers
  rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Graph data
  std::vector<geometry_msgs::msg::Point> nodes_;
  std::vector<std::pair<size_t, size_t>> edges_;

  // PRM parameters
  int num_samples;
  int k_neighbors;
  std::vector<std::vector<double>> obstacles;

};

#endif // PRM_GRAPH_PUBLISHER_HPP
