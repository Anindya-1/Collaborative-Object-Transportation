#ifndef PRM_PLANNER_HPP
#define PRM_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "mm_interfaces/msg/undirected_graph.hpp"
#include "mm_interfaces/msg/pair.hpp"
#include <vector>
#include <random>
#include <cmath>

// Struct for a 3D point
struct Point3D {
    double x, y, z;
};

// Function to calculate Euclidean distance
double euclideanDistance(const Point3D &a, const Point3D &b);
// double euclideanDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);

class PRMPlanner : public rclcpp::Node {
public:
    PRMPlanner();

private:
    void runPRM();
    void generateRandomSamples();
    void constructRoadmap();
    void publishRoadmap();
    void publishGraph();

    bool isPointInObstacle(const Point3D &p);
    bool isEdgeInObstacle(const Point3D &a, const Point3D &b);
    // bool isPointInObstacle(const geometry_msgs::msg::Point &p);
    // bool isEdgeInObstacle(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);

    // PRM parameters
    int num_samples_;
    double connection_radius_;
    double map_size_;
    double obs_rad;
    int max_connection_per_node;
    std::vector<Point3D> samples_;
    // std::vector<geometry_msgs::msg::Point> samples_;
    std::vector<std::pair<size_t, size_t>> edges_;
    std::vector<Point3D> obstacles_;
    // std::vector<geometry_msgs::msg::Point> obstacles_;
    std::vector<int> parent_;  // Disjoint-set to prevent cycles

    // Random number generation
    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_;

    // ROS2 publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr roadmap_pub_;
    rclcpp::Publisher<mm_interfaces::msg::UndirectedGraph>::SharedPtr graph_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    //Disjoint-Set Operations
    int find(int node);
    void unionNodes(int node1, int node2);

};

#endif // PRM_PLANNER_HPP
