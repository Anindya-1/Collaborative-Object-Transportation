#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <vector>
#include <random>
#include <cmath>

// Helper struct for a 2D point
struct Point2D {
    double x, y;
};

// Helper function to calculate Euclidean distance
double euclideanDistance(const Point2D &a, const Point2D &b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

class PRMNode : public rclcpp::Node {
public:
    PRMNode() : Node("prm_node"), rng_(std::random_device{}()), dist_(0.0, 1.0) {
        // Initialize the PRM parameters
        num_samples_ = 500;       // Number of random samples
        connection_radius_ = 0.5; // Max distance for connecting nodes
        map_size_ = 5.0;          // Map is in [0, map_size_] x [0, map_size_]

        // Example obstacles (in normalized coordinates)
        obstacles_ = { {0.5, 0.5}, {0.3, 0.7} };

        // Publishers for visualizing PRM
        roadmap_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("prm_roadmap", 10);
        path_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("prm_path", 10);

        // Timer to run the PRM algorithm
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&PRMNode::runPRM, this));
    }

private:
    void runPRM() {
        generateRandomSamples();
        constructRoadmap();
        publishRoadmap();
    }

    void generateRandomSamples() {
        samples_.clear();
        for (int i = 0; i < num_samples_; ++i) {
            Point2D p;
            do {
                p.x = dist_(rng_) * map_size_;
                p.y = dist_(rng_) * map_size_;
            } while (isPointInObstacle(p));
            samples_.push_back(p);
        }
    }

    void constructRoadmap() {
        edges_.clear();
        for (size_t i = 0; i < samples_.size(); ++i) {
            for (size_t j = i + 1; j < samples_.size(); ++j) {
                if (euclideanDistance(samples_[i], samples_[j]) < connection_radius_) {
                    if (!isEdgeInObstacle(samples_[i], samples_[j])) {
                        edges_.emplace_back(i, j);
                    }
                }
            }
        }
    }

    void publishRoadmap() {
        // Publish the roadmap as a set of lines
        visualization_msgs::msg::Marker roadmap_msg;
        roadmap_msg.header.frame_id = "map";
        roadmap_msg.header.stamp = this->get_clock()->now();
        roadmap_msg.ns = "prm";
        roadmap_msg.id = 0;
        roadmap_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
        roadmap_msg.scale.x = 0.01; // Line width
        roadmap_msg.color.r = 0.0;
        roadmap_msg.color.g = 1.0;
        roadmap_msg.color.b = 0.0;
        roadmap_msg.color.a = 1.0;

        for (const auto &edge : edges_) {
            const auto &p1 = samples_[edge.first];
            const auto &p2 = samples_[edge.second];

            geometry_msgs::msg::Point point1, point2;
            point1.x = p1.x;
            point1.y = p1.y;
            point1.z = 0.0;
            point2.x = p2.x;
            point2.y = p2.y;
            point2.z = 0.0;

            roadmap_msg.points.push_back(point1);
            roadmap_msg.points.push_back(point2);
        }

        roadmap_pub_->publish(roadmap_msg);
    }

    bool isPointInObstacle(const Point2D &p) {
        for (const auto &obs : obstacles_) {
            if (euclideanDistance(p, obs) < 0.1) { // Obstacle radius = 0.1
                return true;
            }
        }
        return false;
    }

    bool isEdgeInObstacle(const Point2D &a, const Point2D &b) {
        // Simplified check: sample along the edge and check for collisions
        int num_checks = 10;
        for (int i = 0; i <= num_checks; ++i) {
            double t = static_cast<double>(i) / num_checks;
            Point2D p = { a.x + t * (b.x - a.x), a.y + t * (b.y - a.y) };
            if (isPointInObstacle(p)) {
                return true;
            }
        }
        return false;
    }

    // PRM parameters
    int num_samples_;
    double connection_radius_;
    double map_size_;
    std::vector<Point2D> samples_;
    std::vector<std::pair<size_t, size_t>> edges_;
    std::vector<Point2D> obstacles_;

    // Random number generation
    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_;

    // ROS2 publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr roadmap_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PRMNode>());
    rclcpp::shutdown();
    return 0;
}
