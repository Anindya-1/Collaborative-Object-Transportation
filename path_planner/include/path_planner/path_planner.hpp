#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "mm_interfaces/msg/undirected_graph.hpp"
#include "mm_interfaces/msg/pair.hpp"
#include "mm_interfaces/msg/terminal_points.hpp"
#include "mm_interfaces/msg/trajectory_diff.hpp"
#include <vector>
#include <random>
#include <cmath>
#include <limits>
#include <queue>

// Struct for a 3D point
struct Point2D {
    double x, y;
};

// Function to calculate Euclidean distance
double euclideanDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
double euclideanDistance(const geometry_msgs::msg::Vector3 &a, const geometry_msgs::msg::Vector3 &b);

class PathPlanner : public rclcpp::Node {
public:
    PathPlanner();

private:
    void runPRM();
    void generateRandomSamples();
    void constructRoadmap();
    void publishRoadmap();
    void publishObstacles();
    void publishGraph();
    void updateRoadmap(const geometry_msgs::msg::Point &s, const geometry_msgs::msg::Point &t);
    
    bool isPointInObstacle(const geometry_msgs::msg::Point &p);
    bool isEdgeInObstacle(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);

    bool target_received;


    // PRM parameters
    int num_samples_;
    double connection_radius_;
    double map_size_;
    double obs_rad;
    double obs_tolerance;
    double robot_rad;
    double rod_length;
    int max_connection_per_node;
    int max_connection_attempts;
    int max_connection_attempts_at_terminal;
    std::vector<geometry_msgs::msg::Point> samples_;
    std::vector<std::pair<size_t, size_t>> edges_;
    std::vector<Point2D> obstacles_;
    std::vector<int> parent_;  // Disjoint-set to prevent cycles
    mm_interfaces::msg::UndirectedGraph graph;

    // Dijkstra functions
    void ComputeTrajectory(const mm_interfaces::msg::TerminalPoints::SharedPtr terminal_pts);
    void Dijkstra();
    std::vector<int> computeDijkstra(int source, int target, const std::vector<std::vector<float>> &adj_matrix);
    std::vector<geometry_msgs::msg::Vector3> extractTrajectory(const std::vector<int> &path_indices, const std::vector<geometry_msgs::msg::Point> &nodes);
    void publishMarker_red(const std::vector<geometry_msgs::msg::Vector3> &trajectory);
    void publishMarker_blue(const std::vector<geometry_msgs::msg::Vector3> &trajectory);
    void publishMarker_black1(const std::vector<geometry_msgs::msg::Vector3> &trajectory);
    void publishMarker_black2(const std::vector<geometry_msgs::msg::Vector3> &trajectory);

    // Random number generation
    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_;

    // ROS2 publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr roadmap_pub_;
    rclcpp::Publisher<mm_interfaces::msg::UndirectedGraph>::SharedPtr graph_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<mm_interfaces::msg::TrajectoryDiff>::SharedPtr leader_trajectory_publisher_;
    rclcpp::Publisher<mm_interfaces::msg::TrajectoryDiff>::SharedPtr follower_trajectory_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher2_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_l_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_f_;


    // ROS2 subscribers
    rclcpp::Subscription<mm_interfaces::msg::TerminalPoints>::SharedPtr terminal_pts_subscription_;

    //Disjoint-Set Operations
    int find(int node);
    void unionNodes(int node1, int node2);

    bool graph_received_;
    std::vector<geometry_msgs::msg::Vector3> trajectory;
    std::vector<geometry_msgs::msg::Vector3> smoothened_trajectory;
    std::vector<geometry_msgs::msg::Vector3> leader_trajectory;
    std::vector<geometry_msgs::msg::Vector3> follower_trajectory;

    // int nearestNode(const mm_interfaces::msg::UndirectedGraph::SharedPtr msg, std::vector<double> position);

    geometry_msgs::msg::Point source_position;
    int source;
    geometry_msgs::msg::Point target_position;
    int target;

    std::vector<geometry_msgs::msg::Vector3> shortcutPath(const std::vector<geometry_msgs::msg::Vector3>& trajectory);
    std::vector<geometry_msgs::msg::Vector3> interpolateTrajectory(const std::vector<geometry_msgs::msg::Vector3>& waypoints);
    std::pair<std::vector<geometry_msgs::msg::Vector3>, std::vector<geometry_msgs::msg::Vector3>> shiftTrajectory(const std::vector<geometry_msgs::msg::Vector3>& waypoints);
};

#endif // PATH_PLANNER_HPP
