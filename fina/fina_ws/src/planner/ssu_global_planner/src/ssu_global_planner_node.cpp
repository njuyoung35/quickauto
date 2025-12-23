#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "ssu_global_planner/lanelet2_graph.hpp"

namespace ssu_global_planner
{

class Lanelet2GlobalPlannerNode : public rclcpp::Node
{
public:
  Lanelet2GlobalPlannerNode()
  : Node("ssu_global_planner")
  {
    // ----- Parameters -----
    std::string pkg_share =
      ament_index_cpp::get_package_share_directory("ssu_global_planner");
    std::string default_osm = pkg_share + "/maps/lanelet2_map.osm";

    this->declare_parameter<std::string>("osm_file", default_osm);
    this->declare_parameter<std::string>("frame_id", "map");

    this->get_parameter("osm_file", osm_file_);
    this->get_parameter("frame_id", frame_id_);

    RCLCPP_INFO(this->get_logger(), "OSM file: %s", osm_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "Frame id: %s", frame_id_.c_str());

    // ----- Load Lanelet Graph -----
    try {
      lanelet_graph_ = std::make_shared<LaneletGraph>(osm_file_);
      RCLCPP_INFO(this->get_logger(), "Loaded %zu nodes from lanelet2 OSM.",
                  lanelet_graph_->nodeCount());
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Failed to load lanelet2 graph: %s", e.what());
      throw;
    }

    // ----- Publisher -----
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_waypoints", 10);

    // ----- Subscribers -----
    // RViz 2D Pose Estimate -> /initialpose (start)
    start_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose",
        10,
        std::bind(&Lanelet2GlobalPlannerNode::onStartPose,
                  this, std::placeholders::_1));

    // RViz 2D Nav Goal
    goal_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose",
        10,
        std::bind(&Lanelet2GlobalPlannerNode::onGoal,
                  this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "Waiting for start on '/initialpose' and goal on '/goal_pose' (frame_id='%s')",
      frame_id_.c_str());
  }

private:
  // ----- Callbacks -----

  // 시작점: /initialpose
  void onStartPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (!msg) {
      RCLCPP_ERROR(this->get_logger(), "Received null start pose.");
      return;
    }

    if (msg->header.frame_id != frame_id_) {
      RCLCPP_WARN(this->get_logger(),
        "Start frame_id is '%s' but planner frame_id is '%s'. "
        "Assuming they are the same frame.",
        msg->header.frame_id.c_str(), frame_id_.c_str());
    }

    start_x_ = msg->pose.pose.position.x;
    start_y_ = msg->pose.pose.position.y;

    auto latlon = lanelet_graph_->xyToLatLon(start_x_, start_y_);
    start_lat_ = latlon.first;
    start_lon_ = latlon.second;
    start_received_ = true;

    RCLCPP_INFO(this->get_logger(),
      "Start set from RViz: map(%.2f, %.2f) -> latlon(%.7f, %.7f)",
      start_x_, start_y_, start_lat_, start_lon_);

    tryPlan();
  }

  // 도착점: /goal 또는 /goal_pose
  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg) {
      RCLCPP_ERROR(this->get_logger(), "Received null goal.");
      return;
    }

    if (msg->header.frame_id != frame_id_) {
      RCLCPP_WARN(this->get_logger(),
        "Goal frame_id is '%s' but planner frame_id is '%s'. "
        "Assuming they are the same frame.",
        msg->header.frame_id.c_str(), frame_id_.c_str());
    }

    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;

    auto latlon = lanelet_graph_->xyToLatLon(goal_x_, goal_y_);
    goal_lat_ = latlon.first;
    goal_lon_ = latlon.second;
    goal_received_ = true;

    RCLCPP_INFO(this->get_logger(),
      "Goal set from RViz: map(%.2f, %.2f) -> latlon(%.7f, %.7f)",
      goal_x_, goal_y_, goal_lat_, goal_lon_);

    tryPlan();
  }

  // 두 점이 다 들어왔을 때만 플래닝
  void tryPlan()
  {
    if (!start_received_ || !goal_received_) {
      RCLCPP_INFO(this->get_logger(),
        "Waiting for both start and goal. start=%s, goal=%s",
        start_received_ ? "OK" : "N/A",
        goal_received_ ? "OK" : "N/A");
      return;
    }

    computeRoute();
  }

  void computeRoute()
  {
    if (!lanelet_graph_) {
      RCLCPP_ERROR(this->get_logger(), "Lanelet graph not loaded.");
      return;
    }

    LaneletGraph::NodeId start_id =
      lanelet_graph_->nearestNode(start_lat_, start_lon_);
    LaneletGraph::NodeId goal_id  =
      lanelet_graph_->nearestNode(goal_lat_, goal_lon_);

    RCLCPP_INFO(this->get_logger(), "Nearest start node: %lld",
                static_cast<long long>(start_id));
    RCLCPP_INFO(this->get_logger(), "Nearest goal  node: %lld",
                static_cast<long long>(goal_id));

    std::vector<LaneletGraph::NodeId> node_path =
      lanelet_graph_->dijkstra(start_id, goal_id);

    if (node_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No path found between start and goal.");
      return;
    }

    auto xy_list = lanelet_graph_->nodeIdsToXY(node_path);

    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id_;
    path.header.stamp = this->now();
    path.poses.reserve(xy_list.size());

    for (const auto & p : xy_list) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = p.first;
      pose.pose.position.y = p.second;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }

    RCLCPP_INFO(this->get_logger(),
      "Publishing global path with %zu poses.", path.poses.size());
    path_pub_->publish(path);
  }

private:
  std::string osm_file_;
  std::string frame_id_;

  // RViz에서 받은 start / goal (map 좌표 + 위경도)
  double start_x_{0.0}, start_y_{0.0};
  double goal_x_{0.0}, goal_y_{0.0};
  double start_lat_{0.0}, start_lon_{0.0};
  double goal_lat_{0.0}, goal_lon_{0.0};
  bool start_received_{false};
  bool goal_received_{false};

  std::shared_ptr<LaneletGraph> lanelet_graph_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
};

}  // namespace ssu_global_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ssu_global_planner::Lanelet2GlobalPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

