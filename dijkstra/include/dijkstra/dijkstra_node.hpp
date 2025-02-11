#ifndef DIJKSTRA_NODE_HPP_
#define DIJKSTRA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "mm_interfaces/msg/trajectory_diff.hpp"
#include "mm_interfaces/msg/undirected_graph.hpp"
#include "mm_interfaces/msg/terminal_points.hpp"
#include <vector>
#include <limits>
#include <queue>

double euclideanDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);

class DijkstraNode : public rclcpp::Node {
public:
    DijkstraNode();

private:
    void graphCallback(const mm_interfaces::msg::UndirectedGraph::SharedPtr msg);
    std::vector<int> computeDijkstra(int source, int target, const std::vector<std::vector<float>> &adj_matrix);
    std::vector<geometry_msgs::msg::Vector3> extractTrajectory(const std::vector<int> &path_indices, const std::vector<geometry_msgs::msg::Point> &nodes);
    void publishMarker(const std::vector<geometry_msgs::msg::Vector3> &trajectory);

    rclcpp::Subscription<mm_interfaces::msg::UndirectedGraph>::SharedPtr graph_subscription_;
    rclcpp::Publisher<mm_interfaces::msg::TrajectoryDiff>::SharedPtr trajectory_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Publisher<mm_interfaces::msg::TerminalPoints>::SharedPtr terminal_pt_publisher_;

    bool graph_received_;
    std::vector<geometry_msgs::msg::Vector3> trajectory;

    int nearestNode(const mm_interfaces::msg::UndirectedGraph::SharedPtr msg, std::vector<double> position);

    std::vector<double> source_position;
    int source;
    std::vector<double> target_position;
    int target;
};

#endif  // DIJKSTRA_NODE_HPP_
