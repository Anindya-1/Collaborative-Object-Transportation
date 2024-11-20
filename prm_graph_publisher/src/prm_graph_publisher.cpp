#include "prm_graph_publisher/prm_graph_publisher.hpp"
#include <random>

PRMGraphPublisher::PRMGraphPublisher()
  : Node("prm_graph_publisher") {
    // Initialize the PRM parameters
    num_samples = 100;
    k_neighbors = 10;

    obstacles = {
      {1.0, 2.0, 1.0, 2.0, 1.0, 2.0},
      {3.0, 4.0, 3.0, 4.0, 3.0, 4.0}
    };

    // Publishers for visualizing PRM
    graph_pub_ = this->create_publisher<example_interfaces::msg::Float64MultiArray>("graph", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    // Add a timer to generate PRM periodically
    timer_ = this->create_wall_timer(
      std::chrono::seconds(5), std::bind(&PRMGraphPublisher::runPRM, this));  
        
}

void PRMGraphPublisher::runPRM() {
    generatePRM(num_samples, k_neighbors, obstacles);
}

geometry_msgs::msg::Point PRMGraphPublisher::sampleRandomPoint() {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_real_distribution<> dis(0.0, 5.0); // Workspace bounds

  geometry_msgs::msg::Point p;
  p.x = dis(gen);
  p.y = dis(gen);
  p.z = dis(gen);
  return p;
}

double PRMGraphPublisher::euclideanDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

bool PRMGraphPublisher::isCollisionFree(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2, const std::vector<std::vector<double>>& obstacles) {
  for (const auto& box : obstacles) {
    // Simple AABB check
    if ((p1.x >= box[0] && p1.x <= box[1] && p1.y >= box[2] && p1.y <= box[3] && p1.z >= box[4] && p1.z <= box[5]) ||
        (p2.x >= box[0] && p2.x <= box[1] && p2.y >= box[2] && p2.y <= box[3] && p2.z >= box[4] && p2.z <= box[5])) {
      return false;
    }
  }
  return true;
}

void PRMGraphPublisher::generatePRM(size_t num_samples, size_t k_neighbors, const std::vector<std::vector<double>>& obstacles) {
  nodes_.clear();
  edges_.clear();

  // Sample random points
  while (nodes_.size() < num_samples) {
    geometry_msgs::msg::Point p = sampleRandomPoint();
    if (isCollisionFree(p, p, obstacles)) {
      nodes_.push_back(p);
    }
  }

  // Connect neighbors
  for (size_t i = 0; i < nodes_.size(); ++i) {
    std::vector<std::pair<double, size_t>> distances;
    for (size_t j = 0; j < nodes_.size(); ++j) {
      if (i != j) {
        double dist = euclideanDistance(nodes_[i], nodes_[j]);
        distances.push_back({dist, j});
      }
    }

    // Sort by distance
    std::sort(distances.begin(), distances.end());
    for (size_t k = 0; k < std::min(k_neighbors, distances.size()); ++k) {
      size_t neighbor_idx = distances[k].second;
      if (isCollisionFree(nodes_[i], nodes_[neighbor_idx], obstacles)) {
        edges_.emplace_back(i, neighbor_idx);
      }
    }
  }

  // Publish the graph
  example_interfaces::msg::Float64MultiArray graph_msg;
  for (const auto& node : nodes_) {
    graph_msg.data.push_back(node.x);
    graph_msg.data.push_back(node.y);
    graph_msg.data.push_back(node.z);
  }
  graph_pub_->publish(graph_msg);

  // Publish markers for RViz
  visualization_msgs::msg::Marker nodes_marker, edges_marker;
  nodes_marker.header.frame_id = edges_marker.header.frame_id = "map";
  nodes_marker.header.stamp = edges_marker.header.stamp = this->get_clock()->now();
  nodes_marker.ns = edges_marker.ns = "prm_graph";
  nodes_marker.action = edges_marker.action = visualization_msgs::msg::Marker::ADD;

  // Nodes visualization
  nodes_marker.type = visualization_msgs::msg::Marker::POINTS;
  nodes_marker.scale.x = nodes_marker.scale.y = nodes_marker.scale.z = 0.1;
  nodes_marker.color.r = 0.0f;
  nodes_marker.color.g = 1.0f;
  nodes_marker.color.b = 0.0f;
  nodes_marker.color.a = 1.0f;

  for (const auto& node : nodes_) {
    nodes_marker.points.push_back(node);
  }

  // Edges visualization
  edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges_marker.scale.x = 0.05;
  edges_marker.color.r = 1.0f;
  edges_marker.color.g = 0.0f;
  edges_marker.color.b = 0.0f;
  edges_marker.color.a = 1.0f;

  for (const auto& edge : edges_) {
    edges_marker.points.push_back(nodes_[edge.first]);
    edges_marker.points.push_back(nodes_[edge.second]);
  }

  marker_pub_->publish(nodes_marker);
  marker_pub_->publish(edges_marker);
}


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PRMGraphPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}