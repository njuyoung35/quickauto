#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <ogr_api.h>
#include <ogrsf_frmts.h>
#include <cpl_conv.h>

#include <unordered_map>
#include <string>
#include <vector>
#include <cmath>
#include <optional>

struct Node { double x, y; std::string id; };
struct Edge { std::string id; std::string from, to; double length; int max_spd; };

class ItsMapLoader : public rclcpp::Node {
public:
  ItsMapLoader() : Node("its_map_loader") {
    declare_parameter<std::string>("node_shp", "");
    declare_parameter<std::string>("link_shp", "");
    declare_parameter<std::string>("multilink_dbf", "");
    declare_parameter<std::string>("turninfo_dbf", "");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<double>("resolution", 1.0);
    declare_parameter<double>("origin_x", 0.0);
    declare_parameter<double>("origin_y", 0.0);
    declare_parameter<bool>("override_grid", false);
    declare_parameter<int>("grid_width", 2000);
    declare_parameter<int>("grid_height", 2000);
    declare_parameter<bool>("publish_markers", true);
    declare_parameter<bool>("publish_occupancy", true);

    lane_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/map/lane_graph", 1);
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map/occupancy", 1);

    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&ItsMapLoader::onTimer, this));
    OGRRegisterAll();
  }

private:
// (중략) 기존 include/멤버 동일

void onTimer() {
  if (loaded_) return;
  loaded_ = true;

  std::string node_path = get_parameter("node_shp").as_string();
  std::string link_path = get_parameter("link_shp").as_string();
  std::string map_frame = get_parameter("map_frame").as_string();

  GDALDataset *node_ds = (GDALDataset*) GDALOpenEx(node_path.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
  GDALDataset *link_ds = (GDALDataset*) GDALOpenEx(link_path.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
  if (!node_ds || !link_ds) {
    RCLCPP_ERROR(get_logger(), "Failed to open SHP: %s or %s", node_path.c_str(), link_path.c_str());
    return;
  }

  OGRLayer *node_lyr = node_ds->GetLayer(0);
  OGRLayer *link_lyr = link_ds->GetLayer(0);

  // --- Extent로 격자 범위 자동 산출 ---
  OGREnvelope env;
  link_lyr->GetExtent(&env, true); // 전체 링크 범위
  const double margin = 50.0;      // 50m 여유
  const double res = get_parameter("resolution").as_double();
  const double origin_x = env.MinX - margin;
  const double origin_y = env.MinY - margin;
  const int grid_w = static_cast<int>(std::ceil((env.MaxX - env.MinX + 2*margin)/res));
  const int grid_h = static_cast<int>(std::ceil((env.MaxY - env.MinY + 2*margin)/res));

  // --- NODE 로드 ---
  std::unordered_map<std::string, Node> nodes;
  node_lyr->ResetReading();
  OGRFeature *feat;
  while ((feat = node_lyr->GetNextFeature()) != nullptr) {
    OGRGeometry *geom = feat->GetGeometryRef();
    if (geom && wkbFlatten(geom->getGeometryType()) == wkbPoint) {
      OGRPoint *pt = geom->toPoint();
      const char* nid = feat->GetFieldAsString("NODE_ID"); // 실제 필드명
      std::string id = nid ? nid : std::to_string(feat->GetFID());
      nodes[id] = Node{pt->getX(), pt->getY(), 0};
    }
    OGRFeature::DestroyFeature(feat);
  }

  // --- LINK 로드 & MarkerArray ---
  visualization_msgs::msg::MarkerArray marr;
  int mid = 0;
  link_lyr->ResetReading();
  while ((feat = link_lyr->GetNextFeature()) != nullptr) {
    OGRGeometry *geom = feat->GetGeometryRef();
    if (geom && wkbFlatten(geom->getGeometryType()) == wkbLineString) {
      OGRLineString *ls = geom->toLineString();

      // 필요시 속도/길이 사용
      int max_spd = feat->GetFieldAsInteger("MAX_SPD");
      double length = feat->GetFieldAsDouble("LENGTH");
      // const char* fnode = feat->GetFieldAsString("F_NODE");
      // const char* tnode = feat->GetFieldAsString("T_NODE");

      visualization_msgs::msg::Marker m;
      m.header.frame_id = map_frame;
      m.header.stamp = now();
      m.ns = "lane_graph";
      m.id = mid++;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.scale.x = 0.2;
      m.color.a = 1.0; m.color.r = 0.1; m.color.g = 0.7; m.color.b = 1.0;

      for (int i = 0; i < ls->getNumPoints(); ++i) {
        geometry_msgs::msg::Point p; p.x = ls->getX(i); p.y = ls->getY(i); p.z = 0.0;
        m.points.push_back(p);
      }
      marr.markers.push_back(m);
    }
    OGRFeature::DestroyFeature(feat);
  }

  if (get_parameter("publish_markers").as_bool()) {
    marker_pub_->publish(marr);
    RCLCPP_INFO(get_logger(), "Published lane_graph markers: %zu", marr.markers.size());
  }

  if (get_parameter("publish_occupancy").as_bool()) {
    // Extent 기반 빈 Occupancy 발행
    auto msg = nav_msgs::msg::OccupancyGrid();
    msg.header.frame_id = map_frame;
    msg.info.resolution = res;
    msg.info.width  = grid_w;
    msg.info.height = grid_h;
    msg.info.origin.position.x = origin_x;
    msg.info.origin.position.y = origin_y;
    msg.data.assign(grid_w * grid_h, 0);
    grid_pub_->publish(msg);
  }

  GDALClose(node_ds);
  GDALClose(link_ds);
}
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ItsMapLoader>());
  rclcpp::shutdown();
  return 0;
}

