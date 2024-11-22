#include "prm_planner/prm.hpp"

// Helper function to calculate Euclidean distance
double euclideanDistance(const Point3D &a, const Point3D &b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}
// double euclideanDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
//     return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
// }

// Constructor for the PRMPlanner
PRMPlanner::PRMPlanner() : Node("prm_planner"), rng_(std::random_device{}()), dist_(0.0, 1.0) {
    this->declare_parameter("sample_size", 1000);
    this->declare_parameter("map_size", 2.0);
    this->declare_parameter("obs_rad", 0.1);
    this->declare_parameter("connection_radius", 0.5);
    
    // Initialize the PRM parameters
    num_samples_ = this->get_parameter("sample_size").as_int();       // Number of random samples
    connection_radius_ = this->get_parameter("connection_radius").as_double();  // Max distance for connecting nodes
    map_size_ = this->get_parameter("map_size").as_double();          // Map is in [0, map_size_] x [0, map_size_] x [0, map_size_]
    max_connection_per_node = 30;
    obs_rad = this->get_parameter("obs_rad").as_double();

    // Example obstacles (in normalized coordinates)
    // obstacles_ = { {1.0, 1.0}, {2.3, 1.7}}; 
    // , {2.5, 5.5}, {2.3, 8.7}, {6.5, 4.5}, {1.3, 9.7},
    //                 {4.5, 1.5}, {8.3, 4.7}, {2.5, 7.5}, {2.3, 4.7} };
    obstacles_ = { {1.0, 1.0, 1.0}, {2.3, 1.7, 1.0}};

    // Publishers for visualizing PRM
    roadmap_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("prm_roadmap", 10);
    graph_pub_ = this->create_publisher<mm_interfaces::msg::UndirectedGraph>("graph", 10);

    generateRandomSamples();
    constructRoadmap();

    // Timer to run the PRM algorithm
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&PRMPlanner::runPRM, this));
}

void PRMPlanner::runPRM() {
    publishRoadmap();
    publishGraph();
}

void PRMPlanner::generateRandomSamples() {
    samples_.clear();
    for (int i = 0; i < num_samples_; ++i) {
        // geometry_msgs::msg::Point p;
        Point3D p;
        do {
            p.x = dist_(rng_) * map_size_;
            p.y = dist_(rng_) * map_size_;
            p.z = dist_(rng_) * map_size_;
        } while (isPointInObstacle(p));
        samples_.push_back(p);
    }
}

void PRMPlanner::constructRoadmap() {
    edges_.clear();
    parent_.resize(samples_.size());
    for (size_t i = 0; i < samples_.size(); ++i) {
        parent_[i] = i; // Initialize each node as its own parent
    }

    for (size_t i = 0; i < samples_.size(); ++i) {
        int connections = 0;
        for (size_t j = i + 1; j < samples_.size(); ++j) {
            if ((euclideanDistance(samples_[i], samples_[j]) < connection_radius_) && (connections < max_connection_per_node)) {
                if (!isEdgeInObstacle(samples_[i], samples_[j])) {
                    // Check for cycles
                    if (find(i) != find(j)) {
                        edges_.emplace_back(i, j);
                        unionNodes(i, j); // Merge the components
                    }
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
        point1.z = p1.z;
        point2.x = p2.x;
        point2.y = p2.y;
        point2.z = p2.z;

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
        node.z = sample.z;

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
    R.edges = e;

    graph_pub_->publish(R);
}

bool PRMPlanner::isPointInObstacle(const Point3D &p) {
    for (const auto &obs : obstacles_) {
        if (euclideanDistance(p, obs) < obs_rad) { // Obstacle radius = 0.1
            return true;
        }
    }
    return false;
}
// bool PRMPlanner::isPointInObstacle(const geometry_msgs::msg::Point &p) {
//     for (const auto &obs : obstacles_) {
//         if (euclideanDistance(p, obs) < obs_rad) { // Obstacle radius = 0.1
//             return true;
//         }
//     }
//     return false;
// }

bool PRMPlanner::isEdgeInObstacle(const Point3D &a, const Point3D &b) {
    // Simplified check: sample along the edge and check for collisions
    int num_checks = 10;
    for (int i = 0; i <= num_checks; ++i) {
        double t = static_cast<double>(i) / num_checks;
        Point3D p = { a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)};
        if (isPointInObstacle(p)) {
            return true;
        }
    }
    return false;
}
// bool PRMPlanner::isEdgeInObstacle(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
//     // Simplified check: sample along the edge and check for collisions
//     int num_checks = 10;
//     for (int i = 0; i <= num_checks; ++i) {
//         double t = static_cast<double>(i) / num_checks;
//         geometry_msgs::msg::Point p;
//         p.x =  a.x + t * (b.x - a.x);
//         p.y =  a.y + t * (b.y - a.y);
//         p.z =  a.z + t * (b.z - a.z);
//         if (isPointInObstacle(p)) {
//             return true;
//         }
//     }
//     return false;
// }

int PRMPlanner::find(int node) {
    if (parent_[node] != node) {
        parent_[node] = find(parent_[node]); // Path compression
    }
    return parent_[node];
}

// Union of two components
void PRMPlanner::unionNodes(int node1, int node2) {
    int root1 = find(node1);
    int root2 = find(node2);
    if (root1 != root2) {
        parent_[root1] = root2; // Merge components
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PRMPlanner>());
    rclcpp::shutdown();
    return 0;
}
