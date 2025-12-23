#include "ssu_global_planner/lanelet2_graph.hpp"

#include <tinyxml2.h>
#include <cmath>
#include <limits>
#include <queue>
#include <set>
#include <stdexcept>
#include <algorithm>

namespace ssu_global_planner
{

LaneletGraph::LaneletGraph(const std::string & osm_path)
{
  loadOSM(osm_path);
}

void LaneletGraph::loadOSM(const std::string & path)
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Failed to load OSM file: " + path);
  }

  tinyxml2::XMLElement * osm = doc.FirstChildElement("osm");
  if (!osm) {
    throw std::runtime_error("Invalid OSM: no <osm> root");
  }

  // 1) 모든 <node> 읽기
  bool first_node = true;
  for (tinyxml2::XMLElement * node = osm->FirstChildElement("node");
       node; node = node->NextSiblingElement("node"))
  {
    const char * id_str  = node->Attribute("id");
    const char * lat_str = node->Attribute("lat");
    const char * lon_str = node->Attribute("lon");
    if (!id_str || !lat_str || !lon_str) continue;

    NodeId id = std::stoll(id_str);
    double lat = std::stod(lat_str);
    double lon = std::stod(lon_str);
    nodes_[id] = {lat, lon};

    if (first_node) {
      first_node = false;
      ref_lat_ = lat;
      ref_lon_ = lon;
      ref_phi_ = ref_lat_ * M_PI / 180.0;
    }
  }

  // 2) lanelet relation에 포함된 way id 수집
  std::set<long long> lanelet_way_ids;
  for (tinyxml2::XMLElement * rel = osm->FirstChildElement("relation");
       rel; rel = rel->NextSiblingElement("relation"))
  {
    bool is_lanelet = false;
    for (tinyxml2::XMLElement * tag = rel->FirstChildElement("tag");
         tag; tag = tag->NextSiblingElement("tag"))
    {
      const char * k = tag->Attribute("k");
      const char * v = tag->Attribute("v");
      if (!k || !v) continue;
      if (std::string(k) == "type" && std::string(v) == "lanelet") {
        is_lanelet = true;
        break;
      }
    }
    if (!is_lanelet) continue;

    for (tinyxml2::XMLElement * mem = rel->FirstChildElement("member");
         mem; mem = mem->NextSiblingElement("member"))
    {
      const char * type = mem->Attribute("type");
      const char * ref  = mem->Attribute("ref");
      if (!type || !ref) continue;
      if (std::string(type) == "way") {
        lanelet_way_ids.insert(std::stoll(ref));
      }
    }
  }

  // 3) lanelet way들로부터 edges 생성
  for (tinyxml2::XMLElement * way = osm->FirstChildElement("way");
       way; way = way->NextSiblingElement("way"))
  {
    const char * id_str = way->Attribute("id");
    if (!id_str) continue;
    long long wid = std::stoll(id_str);

    // lanelet relation에 포함된 way만 사용 (엄격 버전)
    if (!lanelet_way_ids.empty() &&
        lanelet_way_ids.find(wid) == lanelet_way_ids.end())
    {
      continue;
    }

    std::vector<NodeId> nd_refs;
    for (tinyxml2::XMLElement * nd = way->FirstChildElement("nd");
         nd; nd = nd->NextSiblingElement("nd"))
    {
      const char * ref = nd->Attribute("ref");
      if (!ref) continue;
      NodeId nid = std::stoll(ref);
      nd_refs.push_back(nid);
    }

    for (std::size_t i = 0; i + 1 < nd_refs.size(); ++i) {
      NodeId u = nd_refs[i];
      NodeId v = nd_refs[i + 1];
      addEdge(u, v);
      addEdge(v, u);  // 필요시 방향성 조정
    }
  }
}

void LaneletGraph::addEdge(NodeId u, NodeId v)
{
  auto it_u = nodes_.find(u);
  auto it_v = nodes_.find(v);
  if (it_u == nodes_.end() || it_v == nodes_.end()) {
    return;
  }

  double lat1 = it_u->second.first;
  double lon1 = it_u->second.second;
  double lat2 = it_v->second.first;
  double lon2 = it_v->second.second;

  double w = haversine(lat1, lon1, lat2, lon2);
  edges_[u].push_back({v, w});
}

double LaneletGraph::haversine(double lat1, double lon1, double lat2, double lon2)
{
  constexpr double R = 6378137.0;
  double phi1 = lat1 * M_PI / 180.0;
  double phi2 = lat2 * M_PI / 180.0;
  double dphi = (lat2 - lat1) * M_PI / 180.0;
  double dlambda = (lon2 - lon1) * M_PI / 180.0;

  double a = std::sin(dphi / 2.0) * std::sin(dphi / 2.0) +
             std::cos(phi1) * std::cos(phi2) *
             std::sin(dlambda / 2.0) * std::sin(dlambda / 2.0);
  double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
  return R * c;
}

LaneletGraph::NodeId LaneletGraph::nearestNode(double lat, double lon) const
{
  NodeId best_id = 0;
  double best_dist = std::numeric_limits<double>::infinity();

  for (const auto & kv : nodes_) {
    NodeId nid = kv.first;
    double nlat = kv.second.first;
    double nlon = kv.second.second;
    double d = haversine(lat, lon, nlat, nlon);
    if (d < best_dist) {
      best_dist = d;
      best_id = nid;
    }
  }
  return best_id;
}

std::vector<LaneletGraph::NodeId>
LaneletGraph::dijkstra(NodeId start_id, NodeId goal_id) const
{
  std::unordered_map<NodeId, double> dist;
  std::unordered_map<NodeId, NodeId> prev;

  struct QItem {
    NodeId id;
    double d;
    bool operator>(const QItem & other) const { return d > other.d; }
  };

  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;

  dist[start_id] = 0.0;
  pq.push({start_id, 0.0});

  std::set<NodeId> visited;

  while (!pq.empty()) {
    QItem cur = pq.top();
    pq.pop();

    if (visited.count(cur.id)) continue;
    visited.insert(cur.id);

    if (cur.id == goal_id) {
      break;
    }

    auto it = edges_.find(cur.id);
    if (it == edges_.end()) continue;

    for (const auto & edge : it->second) {
      NodeId v = edge.first;
      double w = edge.second;
      double nd = cur.d + w;
      auto it_dist_v = dist.find(v);
      if (it_dist_v == dist.end() || nd < it_dist_v->second) {
        dist[v] = nd;
        prev[v] = cur.id;
        pq.push({v, nd});
      }
    }
  }

  std::vector<NodeId> path;
  if (dist.find(goal_id) == dist.end()) {
    return path;  // 경로 없음
  }

  NodeId cur = goal_id;
  path.push_back(cur);
  while (cur != start_id) {
    cur = prev[cur];
    path.push_back(cur);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<std::pair<double, double>>
LaneletGraph::nodeIdsToXY(const std::vector<NodeId> & ids) const
{
  std::vector<std::pair<double, double>> xy;
  if (ids.empty()) return xy;

  constexpr double R = 6378137.0;

  for (auto nid : ids) {
    auto it = nodes_.find(nid);
    if (it == nodes_.end()) continue;
    double lat = it->second.first;
    double lon = it->second.second;

    double dphi = (lat - ref_lat_) * M_PI / 180.0;
    double dlambda = (lon - ref_lon_) * M_PI / 180.0;

    double x = R * dlambda * std::cos(ref_phi_);
    double y = R * dphi;
    xy.emplace_back(x, y);
  }
  return xy;
}

std::pair<double, double> LaneletGraph::latLonToXY(double lat, double lon) const
{
  constexpr double R = 6378137.0;
  double dphi = (lat - ref_lat_) * M_PI / 180.0;
  double dlambda = (lon - ref_lon_) * M_PI / 180.0;

  double x = R * dlambda * std::cos(ref_phi_);
  double y = R * dphi;
  return {x, y};
}

LaneletGraph::LatLon LaneletGraph::xyToLatLon(double x, double y) const
{
  constexpr double R = 6378137.0;

  double dphi = y / R;
  double dlambda = x / (R * std::cos(ref_phi_));

  double lat = ref_lat_ + dphi * 180.0 / M_PI;
  double lon = ref_lon_ + dlambda * 180.0 / M_PI;
  return {lat, lon};
}

LaneletGraph::LatLon LaneletGraph::getLatLon(NodeId id) const
{
  auto it = nodes_.find(id);
  if (it == nodes_.end()) {
    return {0.0, 0.0};
  }
  return it->second;
}

}  // namespace ssu_global_planner

