#include "dijkstra/dijkstra_node.hpp"

double euclideanDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

DijkstraNode::DijkstraNode() : Node("dijkstra_node"), 
    graph_received_{false} {
    this->declare_parameter<std::vector<double>>("source", {0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("target", {1.0, 1.0, 0.0});

    this->get_parameter("source", source_position);
    this->get_parameter("target", target_position);

    graph_subscription_ = this->create_subscription<mm_interfaces::msg::UndirectedGraph>(
        "graph", 10, std::bind(&DijkstraNode::graphCallback, this, std::placeholders::_1));
    trajectory_publisher_ = this->create_publisher<mm_interfaces::msg::TrajectoryDiff>("trajectory", 10);
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("trajectory_marker", 10);
    terminal_pt_publisher_ = this->create_publisher<mm_interfaces::msg::TerminalPoints>("terminal_points", 10);

    RCLCPP_INFO(this->get_logger(), "DijkstraNode initialized.");
}

void DijkstraNode::graphCallback(const mm_interfaces::msg::UndirectedGraph::SharedPtr msg) {

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

void DijkstraNode::publishMarker(const std::vector<geometry_msgs::msg::Vector3> &trajectory) {
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

std::vector<int> DijkstraNode::computeDijkstra(int source, int target, const std::vector<std::vector<float>> &adj_matrix) {
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

std::vector<geometry_msgs::msg::Vector3> DijkstraNode::extractTrajectory(const std::vector<int> &path_indices, const std::vector<geometry_msgs::msg::Point> &nodes) {
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

int DijkstraNode::nearestNode(const mm_interfaces::msg::UndirectedGraph::SharedPtr graph, std::vector<double> position){
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
    rclcpp::spin(std::make_shared<DijkstraNode>());
    rclcpp::shutdown();
    return 0;
}
