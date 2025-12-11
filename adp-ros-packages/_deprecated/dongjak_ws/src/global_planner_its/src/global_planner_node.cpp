#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "global_planner_its/msg/waypoint_lla.hpp"
#include "global_planner_its/msg/waypoint_lla_array.hpp"

#include <queue>
#include <vector>
#include <cmath>
#include <memory>



class GlobalPlanner : public rclcpp::Node {
public:
  GlobalPlanner() : Node("global_planner") {
    declare_parameter<std::string>("mode", "grid");
    declare_parameter<std::vector<double>>("start_xy", {0.0, 0.0});
    declare_parameter<std::vector<double>>("goal_xy",  {10.0, 0.0});
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<bool>("publish_wgs84", true);

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planning/global_path", 1);
    waypoints_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/planning/waypoints", 1);
    lla_pub_ = create_publisher<global_planner_its::msg::WaypointLLAArray>("/planning/waypoints_lla", 1);

    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map/occupancy", 1, std::bind(&GlobalPlanner::onGrid, this, std::placeholders::_1));

    OGRRegisterAll();
  }

private:
void publishLLA(double mx, double my,
                global_planner_its::msg::WaypointLLAArray &out)
{
  // 원본(레이어) SRS: ITRF2000_Central_Belt_60  (WKT는 PRJ에서 읽는 것이 베스트)
  OGRSpatialReference src, dst;
  // PRJ를 읽어서 설정하는 게 가장 정확하지만, 여기선 파이프라인 내부에 고정:
  src.SetFromUserInput("PROJCS[\"ITRF2000_Central_Belt_60\","
                       "GEOGCS[\"ITRF2000\",DATUM[\"ITRF2000\",SPHEROID[\"GRS 1980\",6378137,298.257222101]],"
                       "PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433]],"
                       "PROJECTION[\"Transverse_Mercator\"],"
                       "PARAMETER[\"latitude_of_origin\",38],"
                       "PARAMETER[\"central_meridian\",127],"
                       "PARAMETER[\"scale_factor\",1],"
                       "PARAMETER[\"false_easting\",200000],"
                       "PARAMETER[\"false_northing\",600000],UNIT[\"metre\",1]]");

  dst.SetWellKnownGeogCS("WGS84");

  OGRCoordinateTransformation *ct = OGRCreateCoordinateTransformation(&src, &dst);
  double x = mx, y = my, z = 0.0;
  if (ct && ct->Transform(1, &x, &y, &z)) {
    global_planner_its::msg::WaypointLLA w;
    w.lat = y; w.lon = x; w.alt = 0.0;
    out.points.push_back(w);
  }
  if (ct) OCTDestroyCoordinateTransformation(ct);
}

void onGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  grid_ = *msg;

  auto start = get_parameter("start_xy").as_double_array();
  auto goal  = get_parameter("goal_xy").as_double_array();
  auto cells = astarXY(start[0], start[1], goal[0], goal[1]);

  nav_msgs::msg::Path path; path.header.frame_id = grid_.header.frame_id; path.header.stamp = now();
  geometry_msgs::msg::PoseArray arr; arr.header = path.header;
  global_planner_its::msg::WaypointLLAArray lla_arr;

  for (auto &c : cells) {
    double mx = grid_.info.origin.position.x + (c.first + 0.5) * grid_.info.resolution;
    double my = grid_.info.origin.position.y + (c.second + 0.5) * grid_.info.resolution;

    geometry_msgs::msg::PoseStamped ps; ps.header = path.header;
    ps.pose.position.x = mx; ps.pose.position.y = my; ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);

    geometry_msgs::msg::Pose p; p.position.x = mx; p.position.y = my; p.orientation.w = 1.0;
    arr.poses.push_back(p);

    if (get_parameter("publish_wgs84").as_bool()) {
      publishLLA(mx, my, lla_arr);
    }
  }

  path_pub_->publish(path);
  waypoints_pub_->publish(arr);
  if (get_parameter("publish_wgs84").as_bool()) lla_pub_->publish(lla_arr);

  RCLCPP_INFO(get_logger(), "Published global path with %zu points", path.poses.size());
}







int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlanner>());
  rclcpp::shutdown();
  return 0;
}

