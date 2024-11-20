#ifndef PRM_PLANNER_HPP
#define PRM_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "mm_interfaces/msg/undirected_graph.hpp"
#include "mm_interfaces/msg/pair.hpp"
#include <vector>
#include <random>
#include <cmath>

// Struct for a 2D point
struct Point2D {
    double x, y;
};

// Function to calculate Euclidean distance
double euclideanDistance(const Point2D &a, const Point2D &b);

class PRMPlanner : public rclcpp::Node {
public:
    PRMPlanner();

private:
    void runPRM();
    void generateRandomSamples();
    void constructRoadmap();
    void publishRoadmap();
    void publishGraph();

    bool isPointInObstacle(const Point2D &p);
    bool isEdgeInObstacle(const Point2D &a, const Point2D &b);

    // PRM parameters
    int num_samples_;
    double connection_radius_;
    double map_size_;
    int max_connection_per_node;
    std::vector<Point2D> samples_;
    std::vector<std::pair<size_t, size_t>> edges_;
    std::vector<Point2D> obstacles_;

    // Random number generation
    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_;

    // ROS2 publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr roadmap_pub_;
    rclcpp::Publisher<mm_interfaces::msg::UndirectedGraph>::SharedPtr graph_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // PRM_PLANNER_HPP
