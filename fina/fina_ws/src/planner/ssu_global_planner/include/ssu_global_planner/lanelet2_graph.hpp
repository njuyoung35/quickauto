#pragma once

#include <unordered_map>
#include <vector>
#include <string>
#include <utility>

namespace ssu_global_planner
{

class LaneletGraph
{
public:
  using NodeId = long long;
  using LatLon = std::pair<double, double>;
  using EdgeList = std::vector<std::pair<NodeId, double>>;  // neighbor id, weight [m]

  explicit LaneletGraph(const std::string & osm_path);

  std::size_t nodeCount() const { return nodes_.size(); }

  // 위경도 기준 최근접 노드
  NodeId nearestNode(double lat, double lon) const;

  // start→goal 최단경로 (노드 ID 시퀀스)
  std::vector<NodeId> dijkstra(NodeId start_id, NodeId goal_id) const;

  // 노드 ID들을 map 기준 x,y (m) 리스트로 변환
  std::vector<std::pair<double, double>>
  nodeIdsToXY(const std::vector<NodeId> & ids) const;

  // 단일 위경도 → map x,y
  std::pair<double, double> latLonToXY(double lat, double lon) const;

  // 단일 map x,y → 위경도
  LatLon xyToLatLon(double x, double y) const;

  // 노드 ID → 위경도
  LatLon getLatLon(NodeId id) const;

private:
  std::unordered_map<NodeId, LatLon> nodes_;
  std::unordered_map<NodeId, EdgeList> edges_;

  // 좌표 변환 기준 (첫 번째 노드 기준)
  double ref_lat_{0.0};
  double ref_lon_{0.0};
  double ref_phi_{0.0};  // rad

  void loadOSM(const std::string & path);
  void addEdge(NodeId u, NodeId v);

  static double haversine(double lat1, double lon1, double lat2, double lon2);
};

}  // namespace ssu_global_planner

