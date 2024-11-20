#include "prm_planner/prm.hpp"

// Helper function to calculate Euclidean distance
double euclideanDistance(const Point2D &a, const Point2D &b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

// Constructor for the PRMPlanner
PRMPlanner::PRMPlanner() : Node("prm_planner"), rng_(std::random_device{}()), dist_(0.0, 1.0) {
    // Initialize the PRM parameters
    num_samples_ = 1000;       // Number of random samples
    connection_radius_ = 0.5; // Max distance for connecting nodes
    map_size_ = 10.0;          // Map is in [0, map_size_] x [0, map_size_]
    max_connection_per_node = 30;

    // Example obstacles (in normalized coordinates)
    obstacles_ = { {0.5, 0.5}, {0.3, 0.7} };

    // Publishers for visualizing PRM
    roadmap_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("prm_roadmap", 10);
    graph_pub_ = this->create_publisher<mm_interfaces::msg::UndirectedGraph>("prm_path", 10);

    // Timer to run the PRM algorithm
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&PRMPlanner::runPRM, this));
}

void PRMPlanner::runPRM() {
    generateRandomSamples();
    constructRoadmap();
    publishRoadmap();
    publishGraph();
}

void PRMPlanner::generateRandomSamples() {
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

void PRMPlanner::constructRoadmap() {
    edges_.clear();
    for (size_t i = 0; i < samples_.size(); ++i) {
        int connections = 0;
        for (size_t j = i + 1; j < samples_.size(); ++j) {
            if ((euclideanDistance(samples_[i], samples_[j]) < connection_radius_) && (connections < max_connection_per_node)) {
                if (!isEdgeInObstacle(samples_[i], samples_[j])) {
                    edges_.emplace_back(i, j);
                }
            }
            connections++;
        }
    }
}

void PRMPlanner::publishRoadmap() {
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

void PRMPlanner::publishGraph() {
    mm_interfaces::msg::UndirectedGraph R;
    std::vector<geometry_msgs::msg::Point> n;
    for (const auto &sample : samples_){
        geometry_msgs::msg::Point node;
        node.x = sample.x;
        node.y = sample.y;
        node.z = 0.0;

        n.push_back(node);
    }
    R.nodes = n;

    std::vector<mm_interfaces::msg::Pair> e;
    for (const auto &single_e : edges_){
        mm_interfaces::msg::Pair edge;
        edge.a = single_e.first;
        edge.b = single_e.second;

        e.push_back(edge);
    }
    R.egdes = e;

    graph_pub_->publish(R);
}

bool PRMPlanner::isPointInObstacle(const Point2D &p) {
    for (const auto &obs : obstacles_) {
        if (euclideanDistance(p, obs) < 0.1) { // Obstacle radius = 0.1
            return true;
        }
    }
    return false;
}

bool PRMPlanner::isEdgeInObstacle(const Point2D &a, const Point2D &b) {
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

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PRMPlanner>());
    rclcpp::shutdown();
    return 0;
}
