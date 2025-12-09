// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simplified_routing.hpp"

#include <autoware/qos_utils/qos_compatibility.hpp>
#include <memory>



namespace uuid
{

std::array<uint8_t, 16> generate_random_id()
{
  static std::independent_bits_engine<std::mt19937, 8, uint8_t> engine(std::random_device{}());
  std::array<uint8_t, 16> id;
  std::generate(id.begin(), id.end(), std::ref(engine));
  return id;
}

UUID generate_if_empty(const UUID & uuid)
{
  constexpr std::array<uint8_t, 16> zero_uuid = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  UUID result;
  result.uuid = (uuid.uuid == zero_uuid) ? generate_random_id() : uuid.uuid;
  return result;
}

}  // namespace autoware::mission_planner_universe::uuid



namespace
{

// using autoware_adapi_v1_msgs::msg::ResponseStatus;

// template <class InterfaceT>
// ResponseStatus route_already_set()
// {
//   ResponseStatus status;
//   status.success = false;
//   status.code = InterfaceT::Service::Response::ERROR_INVALID_STATE;
//   status.message = "The route is already set.";
//   return status;
// }

// template <class InterfaceT>
// ResponseStatus route_is_not_set()
// {
//   ResponseStatus status;
//   status.success = false;
//   status.code = InterfaceT::Service::Response::ERROR_INVALID_STATE;
//   status.message = "The route is not set yet.";
//   return status;
// }

// Simple route conversion utility
// autoware::adapi_specs::routing::RouteState::Message convert_state(
//   const autoware::component_interface_specs::planning::RouteState::Message & state_msg)
// {
//   autoware::adapi_specs::routing::RouteState::Message adapi_state;
//   adapi_state.stamp = state_msg.stamp;
  
//   // Simple state mapping
//   switch (state_msg.state) {
//     case autoware::component_interface_specs::planning::RouteState::Message::SET:
//       adapi_state.state = autoware::adapi_specs::routing::RouteState::Message::SET;
//       break;
//     case autoware::component_interface_specs::planning::RouteState::Message::UNSET:
//       adapi_state.state = autoware::adapi_specs::routing::RouteState::Message::UNSET;
//       break;
//     case autoware::component_interface_specs::planning::RouteState::Message::ARRIVED:
//       adapi_state.state = autoware::adapi_specs::routing::RouteState::Message::ARRIVED;
//       break;
//     default:
//       adapi_state.state = autoware::adapi_specs::routing::RouteState::Message::UNKNOWN;
//       break;
//   }
  
//   return adapi_state;
// }

// Convert and call service
template <typename ClientT, typename RequestT>
ResponseStatus convert_call(const typename ClientT::SharedPtr client, const RequestT & req)
{
  ResponseStatus status;
  
  if (!client->service_is_ready()) {
    status.success = false;
    status.code = ResponseStatus::SERVICE_UNREADY;
    status.message = "Service is not ready";
    return status;
  }
  
  auto future = client->async_send_request(req);
  
  // Wait for response with timeout
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    status.success = false;
    status.code = ResponseStatus::SERVICE_TIMEOUT;
    status.message = "Service call timeout";
    return status;
  }
  
  auto response = future.get();
  status.success = true;
  status.code = ResponseStatus::SUCCESS;
  status.message = "Success";
  
  return status;
}

}  // namespace

namespace autoware::simplified_routing
{

SimplifiedRoutingNode::SimplifiedRoutingNode(const rclcpp::NodeOptions & options)
: Node("simplified_routing", options)
{
  using namespace std::placeholders;
  
  // 단일 목적지 입력 토픽
  sub_goal_ = create_subscription<PoseStamped>(
    "/planning/simplied_routing/goal", 3, std::bind(&SimplifiedRoutingNode::on_goal, this, _1));
  
  // AD API 인터페이스
  // pub_state_ = create_publisher<autoware::adapi_specs::routing::RouteState::Message>(
  //   autoware::adapi_specs::routing::RouteState::name,
  //   autoware::component_interface_specs::get_qos<autoware::adapi_specs::routing::RouteState>());
  
  // pub_route_ = create_publisher<autoware::adapi_specs::routing::Route::Message>(
  //   autoware::adapi_specs::routing::Route::name,
  //   autoware::component_interface_specs::get_qos<autoware::adapi_specs::routing::Route>());
  
  // 단순화된 서비스 (2개만 유지)
  // srv_clear_route_ = create_service<autoware::adapi_specs::routing::ClearRoute::Service>(
  //   autoware::adapi_specs::routing::ClearRoute::name,
  //   std::bind(&SimplifiedRoutingNode::on_clear_route, this, _1, _2));
  
  // srv_set_route_points_ = create_service<autoware::adapi_specs::routing::SetRoutePoints::Service>(
  //   autoware::adapi_specs::routing::SetRoutePoints::name,
  //   std::bind(&SimplifiedRoutingNode::on_set_route_points, this, _1, _2));
  
  // Mission planner 클라이언트
  cli_set_waypoint_route_ =
    create_client<autoware::component_interface_specs::planning::SetWaypointRoute::Service>(
      autoware::component_interface_specs::planning::SetWaypointRoute::name,
      AUTOWARE_DEFAULT_SERVICES_QOS_PROFILE());
  
  // cli_clear_route_ =
  //   create_client<autoware::component_interface_specs::planning::ClearRoute::Service>(
  //     autoware::component_interface_specs::planning::ClearRoute::name,
  //     AUTOWARE_DEFAULT_SERVICES_QOS_PROFILE());
  
  // cli_operation_mode_ =
  //   create_client<autoware::component_interface_specs::system::ChangeOperationMode::Service>(
  //     autoware::component_interface_specs::system::ChangeOperationMode::name,
  //     AUTOWARE_DEFAULT_SERVICES_QOS_PROFILE());
  
  // 상태 구독
  // sub_state_ =
  //   create_subscription<autoware::component_interface_specs::planning::RouteState::Message>(
  //     autoware::component_interface_specs::planning::RouteState::name,
  //     autoware::component_interface_specs::get_qos<
  //       autoware::component_interface_specs::planning::RouteState>(),
  //     std::bind(&SimplifiedRoutingNode::on_state, this, _1));
  
  // sub_route_ =
  //   create_subscription<autoware::component_interface_specs::planning::LaneletRoute::Message>(
  //     autoware::component_interface_specs::planning::LaneletRoute::name,
  //     autoware::component_interface_specs::get_qos<
  //       autoware::component_interface_specs::planning::LaneletRoute>(),
  //     std::bind(&SimplifiedRoutingNode::on_route, this, _1));
  
  // sub_operation_mode_ =
  //   create_subscription<autoware::component_interface_specs::system::OperationModeState::Message>(
  //     autoware::component_interface_specs::system::OperationModeState::name,
  //     autoware::component_interface_specs::get_qos<
  //       autoware::component_interface_specs::system::OperationModeState>(),
  //     std::bind(&SimplifiedRoutingNode::on_operation_mode, this, _1));
  
  // 초기 상태 설정
  // state_.state = autoware::component_interface_specs::planning::RouteState::Message::UNKNOWN;
}

void SimplifiedRoutingNode::on_goal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose)
{
  RCLCPP_INFO(get_logger(), "Received goal at (%f, %f)", 
              pose->pose.position.x, pose->pose.position.y);
  
  // 단순히 목적지 전송
  send_route_request(pose);
}

void SimplifiedRoutingNode::send_route_request(const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_pose)
{
  // 현재 경로가 설정되어 있으면 먼저 클리어
  // if (state_.state == autoware::component_interface_specs::planning::RouteState::Message::SET) {
  //   auto clear_req = std::make_shared<
  //     autoware::component_interface_specs::planning::ClearRoute::Service::Request>();
  //   cli_clear_route_->async_send_request(clear_req);
  // }
  
  // 새로운 경로 설정 요청 생성
  auto route_req = std::make_shared<
    SetWaypointRoute::Service::Request>();
  
  route_req->header = goal_pose->header;
  route_req->goal_pose = goal_pose->pose;
  route_req->waypoints.clear();  // 중간 경유지 없음
  route_req->option.allow_modification = true;  // 고정 목적지
  route_req->uuid = uuid::generate_if_empty(route_req->uuid);
  
  // 경로 설정 요청 전송
  cli_set_waypoint_route_->async_send_request(route_req,
    [this](rclcpp::Client<SetWaypointRoute::Service>::SharedFuture future) {
      try {
        auto response = future.get();
        if (response->status.success) {
          RCLCPP_INFO(get_logger(), "Route set successfully");
        } else {
          RCLCPP_ERROR(get_logger(), "Failed to set route: %s", response->status.message.c_str());
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Exception while setting route: %s", e.what());
      }
    });
}

// void SimplifiedRoutingNode::on_state(
//   const autoware::component_interface_specs::planning::RouteState::Message::ConstSharedPtr msg)
// {
//   state_ = *msg;
  
//   // AD API 상태로 변환하여 발행
//   auto adapi_state = convert_state(*msg);
//   pub_state_->publish(adapi_state);
  
//   // 도착 시 자동 모드에서 정지 모드로 변경
//   if (msg->state == autoware::component_interface_specs::planning::RouteState::Message::ARRIVED) {
//     change_stop_mode();
//   }
// }

// void SimplifiedRoutingNode::on_route(
//   const autoware::component_interface_specs::planning::LaneletRoute::Message::ConstSharedPtr msg)
// {
//   // 간단한 라우트 정보 변환 (필요시 구현)
//   auto adapi_route = std::make_shared<autoware::adapi_specs::routing::Route::Message>();
//   adapi_route->header = msg->header;
//   pub_route_->publish(*adapi_route);
// }

// void SimplifiedRoutingNode::on_operation_mode(
//   const autoware::component_interface_specs::system::OperationModeState::Message::ConstSharedPtr msg)
// {
//   is_autoware_control_ = msg->is_autoware_control_enabled;
//   is_auto_mode_ = (msg->mode == autoware::component_interface_specs::system::OperationModeState::Message::AUTONOMOUS);
// }

// void SimplifiedRoutingNode::change_stop_mode()
// {
//   if (is_auto_mode_) {
//     if (!cli_operation_mode_->service_is_ready()) {
//       RCLCPP_ERROR(get_logger(), "Operation mode service is not ready");
//       return;
//     }
    
//     auto req = std::make_shared<
//       autoware::component_interface_specs::system::ChangeOperationMode::Service::Request>();
//     req->mode = autoware::component_interface_specs::system::ChangeOperationMode::Service::Request::STOP;
    
//     cli_operation_mode_->async_send_request(req);
//   }
// }

// void SimplifiedRoutingNode::on_clear_route(
//   const autoware::adapi_specs::routing::ClearRoute::Service::Request::SharedPtr req,
//   const autoware::adapi_specs::routing::ClearRoute::Service::Response::SharedPtr res)
// {
//   if (!cli_clear_route_->service_is_ready()) {
//     res->status.success = false;
//     res->status.code = autoware_adapi_v1_msgs::msg::ResponseStatus::SERVICE_UNREADY;
//     res->status.message = "Clear route service is not ready";
//     return;
//   }
  
//   res->status = convert_call(cli_clear_route_, req);
// }

// void SimplifiedRoutingNode::on_set_route_points(
//   const autoware::adapi_specs::routing::SetRoutePoints::Service::Request::SharedPtr req,
//   const autoware::adapi_specs::routing::SetRoutePoints::Service::Response::SharedPtr res)
// {
//   // 단순화: 항상 새로운 경로로 설정 (이전 경로 상태 확인 생략)
//   if (!cli_set_waypoint_route_->service_is_ready()) {
//     res->status.success = false;
//     res->status.code = autoware_adapi_v1_msgs::msg::ResponseStatus::SERVICE_UNREADY;
//     res->status.message = "Set waypoint route service is not ready";
//     return;
//   }
  
//   res->status = convert_call(cli_set_waypoint_route_, req);
// }

}  // namespace autoware::simplified_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::simplified_routing::SimplifiedRoutingNode)