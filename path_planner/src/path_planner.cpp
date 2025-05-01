#include "path_planner/path_planner.hpp"


// Helper function to calculate Euclidean distance

double euclideanDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

double euclideanDistance(const geometry_msgs::msg::Vector3 &a, const geometry_msgs::msg::Vector3 &b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

// Constructor for the PathPlanner
PathPlanner::PathPlanner() : Node("path_planner"),
    target_received(false), robot_rad(0.13), rod_length(0.34), max_connection_attempts_at_terminal(1000), rng_(std::random_device{}()), dist_(0.0, 1.0), graph_received_{false} {

    // Declare PRM parameters
    this->declare_parameter("sample_size", 20000);
    this->declare_parameter("map_size", 10.0);
    this->declare_parameter("obs_rad", 0.1);
    this->declare_parameter("connection_radius", 0.75);
    this->declare_parameter("max_connection_count", 50);
    this->declare_parameter("max_connection_attempts", 100);
    
    // Initialize the PRM parameters
    num_samples_ = this->get_parameter("sample_size").as_int();       // Number of random samples
    connection_radius_ = this->get_parameter("connection_radius").as_double();  // Max distance for connecting nodes
    map_size_ = this->get_parameter("map_size").as_double();          // Map is in [0, map_size_] x [0, map_size_]
    max_connection_per_node = this->get_parameter("max_connection_count").as_int();;
    max_connection_attempts = this->get_parameter("max_connection_attempts").as_int();
    obs_rad = this->get_parameter("obs_rad").as_double();

    // Example obstacles (in normalized coordinates)
    // obstacles_ = { {0.8, 2.8}, {2.3, 1.7}, {8.9, 2.0} 
    //     , {8.5, 9.5}, {2.3, 9.3}, {6.5, 4.5}, {4.3, 6.7},
    //     {4.5, 1.5}, {8.3, 6.7}, {2.5, 7.5}, {1.3, 5.7} };

    obstacles_ = { {2.3, 1.7}, {8.9, 2.0} 
        , {8.5, 9.5}, {2.3, 9.3}, {6.5, 4.5}, {4.3, 6.7},
        {4.5, 1.5}, {8.3, 6.7}, {1.3, 5.7} };

    obs_tolerance = 0.05;

    // Publishers for visualizing PRM
    roadmap_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("prm_roadmap", 10);
    // graph_pub_ = this->create_publisher<mm_interfaces::msg::UndirectedGraph>("graph", 10);
    leader_trajectory_publisher_ = this->create_publisher<mm_interfaces::msg::TrajectoryDiff>("robot1/trajectory", 10);
    follower_trajectory_publisher_ = this->create_publisher<mm_interfaces::msg::TrajectoryDiff>("robot2/trajectory", 10);
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("trajectory_marker", 10);
    marker_publisher2_ = this->create_publisher<visualization_msgs::msg::Marker>("smoothened_trajectory_marker", 10);

    marker_publisher_l_ = this->create_publisher<visualization_msgs::msg::Marker>("leader_trajectory_marker", 10);
    marker_publisher_f_ = this->create_publisher<visualization_msgs::msg::Marker>("follower_trajectory_marker", 10);

    generateRandomSamples();
    constructRoadmap();

    // Trajectory terminal points subsriber
    terminal_pts_subscription_ = this->create_subscription<mm_interfaces::msg::TerminalPoints>(
        "terminal_points", 10, std::bind(&PathPlanner::ComputeTrajectory, this, std::placeholders::_1));

    // Timer to run the PRM+Dijkstra algorithm
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&PathPlanner::runPRM, this));
}

void PathPlanner::runPRM() {
    publishRoadmap();
    publishGraph();
    publishObstacles();
    if(target_received){
        Dijkstra();
    }
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
        int connections_attempted = 0;
        int connections = 0;
        for (size_t j = i + 1; j < samples_.size(); ++j) {
            if ((euclideanDistance(samples_[i], samples_[j]) < connection_radius_) && (connections < max_connection_per_node) && (connections_attempted < max_connection_attempts)) {
                if (!isEdgeInObstacle(samples_[i], samples_[j])) {
                    // Check for cycles
                    if (find(i) != find(j)) {
                        edges_.emplace_back(i, j);
                        unionNodes(i, j); // Merge the components
                        connections++;
                    }
                }
            }
            connections_attempted++;
        }
    }

    runPRM();
}

void PathPlanner::updateRoadmap(const geometry_msgs::msg::Point &s, const geometry_msgs::msg::Point &t){

    if (isPointInObstacle(s) || isPointInObstacle(t)) {
        RCLCPP_WARN(this->get_logger(), "Attempted to add points inside an obstacle! Skipping update.");
        return;
    }

    samples_.push_back(s);
    samples_.push_back(t);

    size_t source_idx = samples_.size() - 2;  // Index of s
    size_t target_idx = samples_.size() - 1; // Index of t

    // Resize and initialize new parents
    parent_.resize(samples_.size());
    parent_[source_idx] = source_idx;
    parent_[target_idx] = target_idx;

    for (size_t i = source_idx; i <= target_idx; ++i) {
        int connections = 0;
        int connections_attempted = 0;
        for (size_t j = 0; j < samples_.size(); ++j) {
            if (i == j) continue; // Skip self-connection

            if ((euclideanDistance(samples_[i], samples_[j]) < connection_radius_) && (connections < max_connection_per_node) && (connections_attempted < max_connection_attempts_at_terminal)) {
                if (!isEdgeInObstacle(samples_[i], samples_[j])) {
                    // Check for cycles
                    if (find(i) != find(j)) {
                        edges_.emplace_back(i, j);
                        unionNodes(i, j); // Merge the components
                        connections++;
                    }
                }
            }
            connections_attempted++;
        }
    }

    publishGraph();

}

void PathPlanner::publishRoadmap() {
    // Publish the roadmap as a set of lines
    visualization_msgs::msg::Marker roadmap_msg;
    roadmap_msg.header.frame_id = "odom";
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

    graph = R;

}

void PathPlanner::publishObstacles() {
    visualization_msgs::msg::Marker obstacles_msg;
    obstacles_msg.header.frame_id = "odom";
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
        if (euclideanDistance(p, obs_) < obs_rad + obs_tolerance + ((2*(robot_rad) + rod_length)*0.5) ) { // Obstacle radius = 0.1
            return true;
        }
    }
    return false;
}

bool PathPlanner::isEdgeInObstacle(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    // Simplified check: sample along the edge and check for collisions
    int num_checks;
    if (euclideanDistance(a, b) < 1.0){
        num_checks = 10;
    }
    else if (euclideanDistance(a,b) >= 1.0 && euclideanDistance(a,b) < 3.0){
        num_checks = 25;
    }
    else if (euclideanDistance(a,b) >= 1.0 && euclideanDistance(a,b) < 3.0){
        num_checks = 50;
    }
    else{
        num_checks = 100;
    }
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

void PathPlanner::ComputeTrajectory(const mm_interfaces::msg::TerminalPoints::SharedPtr terminal_pts) {
    RCLCPP_INFO(this->get_logger(), "New terminal Points obtained. Recomputing the trajectory.");

    source_position.x = terminal_pts->source.x;
    source_position.y = terminal_pts->source.y;
    target_position.x = terminal_pts->target.x;
    target_position.y = terminal_pts->target.y;

    updateRoadmap(source_position, target_position);

    graph_received_ = false;
    target_received = true;

    Dijkstra();
}

void PathPlanner::Dijkstra() {

    if (!graph_received_) {
        mm_interfaces::msg::UndirectedGraph::SharedPtr msg = std::make_shared<mm_interfaces::msg::UndirectedGraph>(graph);
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

        source = samples_.size() - 2;
        target = samples_.size() - 1;

        auto path_indices = computeDijkstra(source, target, adj_matrix);
        trajectory = extractTrajectory(path_indices, msg->nodes);
        smoothened_trajectory = shortcutPath(trajectory);
        // smoothened_trajectory = interpolateTrajectory(smoothened_trajectory);

        auto shifted_traj = shiftTrajectory(smoothened_trajectory);
        leader_trajectory = shifted_traj.first;
        follower_trajectory = shifted_traj.second;

    }

    // mm_interfaces::msg::TrajectoryDiff trajectory_msg;
    // trajectory_msg.trajectory = trajectory;
    // trajectory_publisher_->publish(trajectory_msg);

    // mm_interfaces::msg::TrajectoryDiff smoothened_trajectory_msg;
    // smoothened_trajectory_msg.trajectory = smoothened_trajectory;
    // smoothened_trajectory_publisher_->publish(smoothened_trajectory_msg);

    mm_interfaces::msg::TrajectoryDiff leader_trajectory_msg;
    leader_trajectory_msg.trajectory = leader_trajectory;
    leader_trajectory_publisher_->publish(leader_trajectory_msg);

    mm_interfaces::msg::TrajectoryDiff follower_trajectory_msg;
    follower_trajectory_msg.trajectory = follower_trajectory;
    follower_trajectory_publisher_->publish(follower_trajectory_msg);

    // Publish the trajectory as a marker
    publishMarker_red(trajectory);
    publishMarker_blue(smoothened_trajectory);

    publishMarker_black1(leader_trajectory);
    publishMarker_black2(follower_trajectory);
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
    source_waypoint.x = source_position.x;
    source_waypoint.y = source_position.y;
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
    target_waypoint.x = target_position.x;
    target_waypoint.y = target_position.y;
    // target_waypoint.z = target_position[2];
    target_waypoint.z = 0.0;
    trajectory.push_back(target_waypoint);
    return trajectory;
}

std::vector<geometry_msgs::msg::Vector3> PathPlanner::shortcutPath(const std::vector<geometry_msgs::msg::Vector3>& trajectory) {
    if (trajectory.size() < 3) return trajectory; // No need to smooth if too few points
    
    std::vector<geometry_msgs::msg::Vector3> smooth_path;
    smooth_path.push_back(trajectory.front()); // Start point
    
    size_t i = 0;
    while (i < trajectory.size() - 1) {
        size_t j = trajectory.size() - 1;
        while (j > i + 1) {
            geometry_msgs::msg::Point pt1;
            pt1.x = trajectory[i].x;
            pt1.y = trajectory[i].y;
            geometry_msgs::msg::Point pt2;
            pt2.x = trajectory[j].x;
            pt2.y = trajectory[j].y;
            if (!isEdgeInObstacle(pt1, pt2)) {
                break; // Found a shortcut
            }
            j--;
        }
        smooth_path.push_back(trajectory[j]);
        i = j;
    }
    
    return smooth_path;
}

std::vector<geometry_msgs::msg::Vector3> PathPlanner::interpolateTrajectory(const std::vector<geometry_msgs::msg::Vector3>& waypoints) {
    std::vector<geometry_msgs::msg::Vector3> interpolated;
    int m = 1;
    if (waypoints.size() < 2) return waypoints;
    
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        geometry_msgs::msg::Vector3 start = waypoints[i];
        geometry_msgs::msg::Vector3 end = waypoints[i + 1];

        if(euclideanDistance(start, end) < 1.0){
            m = 3;
        }
        else if(euclideanDistance(start, end) < 3.0 && euclideanDistance(start, end) >= 1.0){
            m = 5;
        }
        else if(euclideanDistance(start, end) < 5.0 && euclideanDistance(start, end) >= 3.0){
            m = 7;
        }
        else{
            m = 10;
        }

        for (int j = 0; j < m; ++j) {
            double t = static_cast<double>(j) / m;
            geometry_msgs::msg::Vector3 interpolated_point;
            interpolated_point.x = start.x + t * (end.x - start.x);
            interpolated_point.y = start.y + t * (end.y - start.y);
            interpolated_point.z = start.z + t * (end.z - start.z);
            interpolated.push_back(interpolated_point);
        }
    }
    interpolated.push_back(waypoints.back());
    return interpolated;
}

std::pair<std::vector<geometry_msgs::msg::Vector3>, std::vector<geometry_msgs::msg::Vector3>> PathPlanner::shiftTrajectory(const std::vector<geometry_msgs::msg::Vector3>& smoothened_traj) {
    std::vector<geometry_msgs::msg::Vector3> interpolated = interpolateTrajectory(smoothened_traj);
    std::vector<geometry_msgs::msg::Vector3> left_traj, right_traj;
    double l = (rod_length)/2;
    
    for (size_t i = 0; i < interpolated.size(); ++i) {
        double dx, dy;
        if(i != interpolated.size()-1){
            dx = interpolated[i + 1].x - interpolated[i].x;
            dy = interpolated[i + 1].y - interpolated[i].y;
        }
        else{
            dx = interpolated[i].x - interpolated[i-1].x;
            dy = interpolated[i].y - interpolated[i-1].y;
        }
        
        double length = hypot(dx, dy);
        if (length == 0) continue;
        dx /= length;
        dy /= length;
        
        double perp_x = -dy;
        double perp_y = dx;
        
        geometry_msgs::msg::Vector3 left_point, right_point;
        left_point.x = interpolated[i].x + l * perp_x;
        left_point.y = interpolated[i].y + l * perp_y;
        left_point.z = interpolated[i].z;
        
        right_point.x = interpolated[i].x - l * perp_x;
        right_point.y = interpolated[i].y - l * perp_y;
        right_point.z = interpolated[i].z;
        
        left_traj.push_back(left_point);
        right_traj.push_back(right_point);
    }
    
    // left_traj.push_back(interpolated.back());
    // right_traj.push_back(interpolated.back());
    
    return {left_traj, right_traj};
}

void PathPlanner::publishMarker_red(const std::vector<geometry_msgs::msg::Vector3> &trajectory) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom"; // Change this to your desired frame
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

void PathPlanner::publishMarker_blue(const std::vector<geometry_msgs::msg::Vector3> &trajectory) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom"; // Change this to your desired frame
    marker.header.stamp = this->now();
    marker.ns = "trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set marker color
    marker.scale.x = 0.1; // Line width
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // Add points to the marker
    for (const auto &point : trajectory) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        marker.points.push_back(p);
    }

    marker_publisher2_->publish(marker);
}

void PathPlanner::publishMarker_black1(const std::vector<geometry_msgs::msg::Vector3> &trajectory) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom"; // Change this to your desired frame
    marker.header.stamp = this->now();
    marker.ns = "trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set marker color
    marker.scale.x = 0.1; // Line width
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // Add points to the marker
    for (const auto &point : trajectory) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        marker.points.push_back(p);
    }

    marker_publisher_l_->publish(marker);
}

void PathPlanner::publishMarker_black2(const std::vector<geometry_msgs::msg::Vector3> &trajectory) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom"; // Change this to your desired frame
    marker.header.stamp = this->now();
    marker.ns = "trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set marker color
    marker.scale.x = 0.1; // Line width
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // Add points to the marker
    for (const auto &point : trajectory) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        marker.points.push_back(p);
    }

    marker_publisher_f_->publish(marker);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}


