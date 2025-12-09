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

#ifndef SIMPLIFIED_ROUTING_HPP_
#define SIMPLIFIED_ROUTING_HPP_

// #include <autoware/adapi_specs/routing.hpp>
// #include <autoware/component_interface_specs/planning.hpp>
// #include <autoware/component_interface_specs/system.hpp>
#include <autoware_planning_msgs/srv/set_waypoint_route.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>

namespace autoware::simplified_adapi
{

class SimplifiedRoutingNode : public rclcpp::Node
{
public:
  explicit SimplifiedRoutingNode(const rclcpp::NodeOptions & options);

private:
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using SetWaypointRoute = autoware_planning_msgs::srv::SetWaypointRoute;

  // Input topics
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_goal_;
  
  // State topics
  // rclcpp::Subscription<autoware::component_interface_specs::planning::RouteState::Message>::SharedPtr sub_state_;
  // rclcpp::Subscription<autoware::component_interface_specs::planning::LaneletRoute::Message>::SharedPtr sub_route_;
  // rclcpp::Subscription<autoware::component_interface_specs::system::OperationModeState::Message>::SharedPtr sub_operation_mode_;
  
  // Output topics
  // rclcpp::Publisher<autoware::adapi_specs::routing::RouteState::Message>::SharedPtr pub_state_;
  // rclcpp::Publisher<autoware::adapi_specs::routing::Route::Message>::SharedPtr pub_route_;
  
  // Services
  // rclcpp::Service<autoware::adapi_specs::routing::SetRoutePoints::Service>::SharedPtr srv_set_route_points_;
  // rclcpp::Service<autoware::adapi_specs::routing::ClearRoute::Service>::SharedPtr srv_clear_route_;
  
  // Clients to mission planner
  rclcpp::Client<SetWaypointRoute::Service>::SharedPtr cli_set_waypoint_route_;
  // rclcpp::Client<autoware::component_interface_specs::planning::ClearRoute::Service>::SharedPtr cli_clear_route_;
  // rclcpp::Client<autoware::component_interface_specs::system::ChangeOperationMode::Service>::SharedPtr cli_operation_mode_;
  
  // State variables
  // autoware::component_interface_specs::planning::RouteState::Message state_;
  // bool is_auto_mode_ = false;
  // bool is_autoware_control_ = false;
  
  // Callbacks
  void on_goal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose);
  // void on_state(const autoware::component_interface_specs::planning::RouteState::Message::ConstSharedPtr msg);
  // void on_route(const autoware::component_interface_specs::planning::LaneletRoute::Message::ConstSharedPtr msg);
  // void on_operation_mode(const autoware::component_interface_specs::system::OperationModeState::Message::ConstSharedPtr msg);
  
  // Service callbacks
  // void on_clear_route(
  //   const autoware::adapi_specs::routing::ClearRoute::Service::Request::SharedPtr req,
  //   const autoware::adapi_specs::routing::ClearRoute::Service::Response::SharedPtr res);
  
  // void on_set_route_points(
  //   const autoware::adapi_specs::routing::SetRoutePoints::Service::Request::SharedPtr req,
  //   const autoware::adapi_specs::routing::SetRoutePoints::Service::Response::SharedPtr res);
  
  // Helper functions
  // void change_stop_mode();
  void send_route_request(const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_pose);
};

}  // namespace autoware::simplified_adapi

#endif  // SIMPLIFIED_ROUTING_HPP_