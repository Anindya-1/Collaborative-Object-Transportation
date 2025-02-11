#include "path_planner/path_planner.hpp"


// Helper function to calculate Euclidean distance

double euclideanDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

// Constructor for the PathPlanner
PathPlanner::PathPlanner() : Node("path_planner"),
    rng_(std::random_device{}()), dist_(0.0, 1.0), pub_flag(false), graph_received_{false} {

    // store source and target data
    this->declare_parameter<std::vector<double>>("source", {0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("target", {1.0, 1.0, 0.0});

    this->get_parameter("source", source_position);
    this->get_parameter("target", target_position);

    this->declare_parameter("sample_size", 10000);
    this->declare_parameter("map_size", 5.0);
    this->declare_parameter("obs_rad", 0.2);
    this->declare_parameter("connection_radius", 0.50);
    this->declare_parameter("max_connection_count", 30);
    
    // Initialize the PRM parameters
    num_samples_ = this->get_parameter("sample_size").as_int();       // Number of random samples
    connection_radius_ = this->get_parameter("connection_radius").as_double();  // Max distance for connecting nodes
    map_size_ = this->get_parameter("map_size").as_double();          // Map is in [0, map_size_] x [0, map_size_]
    max_connection_per_node = this->get_parameter("max_connection_count").as_int();;
    obs_rad = this->get_parameter("obs_rad").as_double();

    // Example obstacles (in normalized coordinates)
    obstacles_ = { {1.0, 2.0}, {2.3, 3.7}, {3.9, 2.0}}; 
    // , {2.5, 5.5}, {2.3, 8.7}, {6.5, 4.5}, {1.3, 9.7},
    //                 {4.5, 1.5}, {8.3, 4.7}, {2.5, 7.5}, {2.3, 4.7} };

    // Publishers for visualizing PRM
    roadmap_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("prm_roadmap", 10);
    graph_pub_ = this->create_publisher<mm_interfaces::msg::UndirectedGraph>("graph", 10);
    trajectory_publisher_ = this->create_publisher<mm_interfaces::msg::TrajectoryDiff>("trajectory", 10);
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("trajectory_marker", 10);

    generateRandomSamples();
    constructRoadmap();

    // Timer to run the PRM algorithm
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&PathPlanner::runPRM, this));
}

void PathPlanner::runPRM() {
    publishRoadmap();
    publishGraph();
    publishObstacles();
}

void PathPlanner::generateRandomSamples() {
    samples_.clear();
    for (int i = 0; i < num_samples_; ++i) {
        geometry_msgs::msg::Point p;
        do {
            p.x = dist_(rng_) * map_size_;
            p.y = dist_(rng_) * map_size_;
        } while (isPointInObstacle(p));
        samples_.push_back(p);
    }
}

void PathPlanner::constructRoadmap() {
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

void PathPlanner::Dijkstra(const mm_interfaces::msg::UndirectedGraph::SharedPtr msg) {

    // mm_interfaces::msg::TerminalPoints term_pt_msg;
    // term_pt_msg.source.x = source_position[0];
    // term_pt_msg.source.y = source_position[1];

    // term_pt_msg.target.x = target_position[0];
    // term_pt_msg.target.y = target_position[1];

    // terminal_pt_publisher_->publish(term_pt_msg);

    if (!graph_received_) {
        RCLCPP_INFO(this->get_logger(), "Received graph data.");
        graph_received_ = true;

        // Create an adjacency matrix from the edges
        int n = msg->nodes.size();
        std::vector<std::vector<float>> adj_matrix(n, std::vector<float>(n, std::numeric_limits<float>::infinity()));

        for (const auto &edge : msg->edges) {
            int a = edge.a;
            int b = edge.b;
            if (a >= 0 && b >= 0 && a < n && b < n) {
                float distance = std::sqrt(
                    std::pow(msg->nodes[a].x - msg->nodes[b].x, 2) +
                    std::pow(msg->nodes[a].y - msg->nodes[b].y, 2) +
                    std::pow(msg->nodes[a].z - msg->nodes[b].z, 2));
                adj_matrix[a][b] = distance;
                adj_matrix[b][a] = distance;  // Assuming undirected graph
            }
        }

        source = nearestNode(msg, source_position);
        target = nearestNode(msg, target_position);

        auto path_indices = computeDijkstra(source, target, adj_matrix);
        trajectory = extractTrajectory(path_indices, msg->nodes);    
    }
    

    mm_interfaces::msg::TrajectoryDiff trajectory_msg;
    trajectory_msg.trajectory = trajectory;
    trajectory_publisher_->publish(trajectory_msg);

    // Publish the trajectory as a marker
    publishMarker(trajectory);
}

void PathPlanner::publishRoadmap() {
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
        point2.x = p2.x;
        point2.y = p2.y;

        roadmap_msg.points.push_back(point1);
        roadmap_msg.points.push_back(point2);
    }

    roadmap_pub_->publish(roadmap_msg);
}

void PathPlanner::publishGraph() {
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

    if(pub_flag==false){
        RCLCPP_INFO(this->get_logger(), "Graph published");
        pub_flag = true;
    }
}

void PathPlanner::publishObstacles() {
    visualization_msgs::msg::Marker obstacles_msg;
    obstacles_msg.header.frame_id = "map";
    obstacles_msg.header.stamp = this->get_clock()->now();
    obstacles_msg.ns = "obstacles";
    obstacles_msg.id = 1; // Unique ID for obstacles
    obstacles_msg.type = visualization_msgs::msg::Marker::SPHERE_LIST; // Render obstacles as spheres
    obstacles_msg.scale.x = 2 * obs_rad; // Diameter of obstacles
    obstacles_msg.scale.y = 2 * obs_rad;
    obstacles_msg.scale.z = 0.1; // Minimal height for 2D visualization
    obstacles_msg.color.r = 1.0; // Red color
    obstacles_msg.color.g = 0.0;
    obstacles_msg.color.b = 0.0;
    obstacles_msg.color.a = 1.0;

    for (const auto &obs : obstacles_) {
        geometry_msgs::msg::Point p;
        p.x = obs.x;
        p.y = obs.y;
        p.z = 0.0; // Obstacles are on the 2D plane
        obstacles_msg.points.push_back(p);
    }

    roadmap_pub_->publish(obstacles_msg); // Use the same publisher or define a new one
}


bool PathPlanner::isPointInObstacle(const geometry_msgs::msg::Point &p) {
    for (const auto &obs : obstacles_) {
        geometry_msgs::msg::Point obs_;
        obs_.x = obs.x;
        obs_.y = obs.y;
        obs_.z = 0.0;
        if (euclideanDistance(p, obs_) < obs_rad) { // Obstacle radius = 0.1
            return true;
        }
    }
    return false;
}

bool PathPlanner::isEdgeInObstacle(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    // Simplified check: sample along the edge and check for collisions
    int num_checks = 10;
    for (int i = 0; i <= num_checks; ++i) {
        double t = static_cast<double>(i) / num_checks;
        geometry_msgs::msg::Point p;
        p.x =  a.x + t * (b.x - a.x);
        p.y =  a.y + t * (b.y - a.y);
        p.z =  a.z + t * (b.z - a.z);
        if (isPointInObstacle(p)) {
            return true;
        }
    }
    return false;
}

int PathPlanner::find(int node) {
    if (parent_[node] != node) {
        parent_[node] = find(parent_[node]); // Path compression
    }
    return parent_[node];
}

// Union of two components
void PathPlanner::unionNodes(int node1, int node2) {
    int root1 = find(node1);
    int root2 = find(node2);
    if (root1 != root2) {
        parent_[root1] = root2; // Merge components
    }
}

void PathPlanner::publishMarker(const std::vector<geometry_msgs::msg::Vector3> &trajectory) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map"; // Change this to your desired frame
    marker.header.stamp = this->now();
    marker.ns = "trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set marker color
    marker.scale.x = 0.1; // Line width
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Add points to the marker
    for (const auto &point : trajectory) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        marker.points.push_back(p);
    }

    marker_publisher_->publish(marker);
}

std::vector<int> PathPlanner::computeDijkstra(int source, int target, const std::vector<std::vector<float>> &adj_matrix) {
    int n = adj_matrix.size();
    std::vector<float> distances(n, std::numeric_limits<float>::infinity());
    std::vector<int> previous(n, -1);
    std::vector<bool> visited(n, false);

    distances[source] = 0.0f;

    using Node = std::pair<float, int>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    pq.push({0.0f, source});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (visited[u]) continue;
        visited[u] = true;

        for (int v = 0; v < n; ++v) {
            if (!visited[v] && adj_matrix[u][v] < std::numeric_limits<float>::infinity()) {
                float new_dist = distances[u] + adj_matrix[u][v];
                if (new_dist < distances[v]) {
                    distances[v] = new_dist;
                    previous[v] = u;
                    pq.push({new_dist, v});
                }
            }
        }
    }

    std::vector<int> path;
    for (int at = target; at != -1; at = previous[at]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());

    if (path.front() != source) {
        RCLCPP_WARN(this->get_logger(), "No path found from source to target.");
        return {};
    }

    return path;
}

std::vector<geometry_msgs::msg::Vector3> PathPlanner::extractTrajectory(const std::vector<int> &path_indices, const std::vector<geometry_msgs::msg::Point> &nodes) {
    std::vector<geometry_msgs::msg::Vector3> trajectory;
    geometry_msgs::msg::Vector3 source_waypoint;
    source_waypoint.x = source_position[0];
    source_waypoint.y = source_position[1];
    // source_waypoint.z = source_position[2];
    source_waypoint.z = 0.0;
    trajectory.push_back(source_waypoint);
    for (int index : path_indices) {
        geometry_msgs::msg::Vector3 point;
        point.x = nodes[index].x;
        point.y = nodes[index].y;
        // point.z = nodes[index].z;
        point.z = 0.0;
        trajectory.push_back(point);
    }
    geometry_msgs::msg::Vector3 target_waypoint;
    target_waypoint.x = target_position[0];
    target_waypoint.y = target_position[1];
    // target_waypoint.z = target_position[2];
    target_waypoint.z = 0.0;
    trajectory.push_back(target_waypoint);
    return trajectory;
}

int PathPlanner::nearestNode(const mm_interfaces::msg::UndirectedGraph::SharedPtr graph, std::vector<double> position){
    int node_num = 0;
    int min_node_num = 0;
    int dist = INT32_MAX;
    geometry_msgs::msg::Point pos;
    pos.x = position[0];
    pos.y = position[1];
    pos.z = position[2];
    for(auto n:graph->nodes){
        double dist_from_pos = euclideanDistance(pos, n);
        if(dist_from_pos < dist){
            min_node_num = node_num;
            dist = dist_from_pos;
        }
        node_num++;
    }
    return min_node_num;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}


