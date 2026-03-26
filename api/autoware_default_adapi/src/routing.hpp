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

#ifndef ROUTING_HPP_
#define ROUTING_HPP_

#include <agnocast/agnocast.hpp>
#include <autoware/adapi_specs/routing.hpp>
#include <autoware/component_interface_specs/planning.hpp>
#include <autoware/component_interface_specs/system.hpp>
#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <rclcpp/rclcpp.hpp>

namespace autoware::default_adapi
{

class RoutingNode : public agnocast::Node
{
public:
  explicit RoutingNode(const rclcpp::NodeOptions & options);

private:
  using OperationModeState = autoware::component_interface_specs::system::OperationModeState;
  using State = autoware::component_interface_specs::planning::RouteState;
  using Route = autoware::component_interface_specs::planning::LaneletRoute;

  rclcpp::CallbackGroup::SharedPtr group_cli_;

  // AD API Interface
  agnocast::Publisher<autoware::adapi_specs::routing::RouteState::Message>::SharedPtr pub_state_;
  agnocast::Publisher<autoware::adapi_specs::routing::Route::Message>::SharedPtr pub_route_;
  agnocast::Service<autoware::adapi_specs::routing::SetRoutePoints::Service>::SharedPtr
    srv_set_route_points_;
  agnocast::Service<autoware::adapi_specs::routing::SetRoute::Service>::SharedPtr srv_set_route_;
  agnocast::Service<autoware::adapi_specs::routing::ChangeRoutePoints::Service>::SharedPtr
    srv_change_route_points_;
  agnocast::Service<autoware::adapi_specs::routing::ChangeRoute::Service>::SharedPtr
    srv_change_route_;
  agnocast::Service<autoware::adapi_specs::routing::ClearRoute::Service>::SharedPtr srv_clear_route_;

  // Component Interface
  agnocast::Subscription<
    autoware::component_interface_specs::planning::RouteState::Message>::SharedPtr sub_state_;
  agnocast::Subscription<
    autoware::component_interface_specs::planning::LaneletRoute::Message>::SharedPtr sub_route_;
  agnocast::Client<autoware::component_interface_specs::planning::SetWaypointRoute::Service>::
    SharedPtr cli_set_waypoint_route_;
  agnocast::Client<autoware::component_interface_specs::planning::SetLaneletRoute::Service>::SharedPtr
    cli_set_lanelet_route_;
  agnocast::Client<autoware::component_interface_specs::planning::ClearRoute::Service>::SharedPtr
    cli_clear_route_;
  agnocast::Subscription<autoware::component_interface_specs::system::OperationModeState::Message>::
    SharedPtr sub_operation_mode_;

  agnocast::Client<autoware::component_interface_specs::system::ChangeOperationMode::Service>::
    SharedPtr cli_operation_mode_;

  void change_stop_mode();
  void on_operation_mode(
    const agnocast::ipc_shared_ptr<const OperationModeState::Message> & msg);
  void on_state(const agnocast::ipc_shared_ptr<const State::Message> & msg);
  void on_route(const agnocast::ipc_shared_ptr<const Route::Message> & msg);
  void on_clear_route(
    const agnocast::ipc_shared_ptr<
      agnocast::Service<autoware::adapi_specs::routing::ClearRoute::Service>::RequestT> & req,
    agnocast::ipc_shared_ptr<
      agnocast::Service<autoware::adapi_specs::routing::ClearRoute::Service>::ResponseT> & res);
  void on_set_route_points(
    const agnocast::ipc_shared_ptr<
      agnocast::Service<autoware::adapi_specs::routing::SetRoutePoints::Service>::RequestT> & req,
    agnocast::ipc_shared_ptr<
      agnocast::Service<autoware::adapi_specs::routing::SetRoutePoints::Service>::ResponseT> &
      res);
  void on_set_route(
    const agnocast::ipc_shared_ptr<
      agnocast::Service<autoware::adapi_specs::routing::SetRoute::Service>::RequestT> & req,
    agnocast::ipc_shared_ptr<
      agnocast::Service<autoware::adapi_specs::routing::SetRoute::Service>::ResponseT> & res);
  void on_change_route_points(
    const agnocast::ipc_shared_ptr<
      agnocast::Service<autoware::adapi_specs::routing::ChangeRoutePoints::Service>::RequestT> &
      req,
    agnocast::ipc_shared_ptr<
      agnocast::Service<autoware::adapi_specs::routing::ChangeRoutePoints::Service>::ResponseT> &
      res);
  void on_change_route(
    const agnocast::ipc_shared_ptr<
      agnocast::Service<autoware::adapi_specs::routing::ChangeRoute::Service>::RequestT> & req,
    agnocast::ipc_shared_ptr<
      agnocast::Service<autoware::adapi_specs::routing::ChangeRoute::Service>::ResponseT> & res);

  bool is_autoware_control_;
  bool is_auto_mode_;
  State::Message state_;

  // Stop check for route clear.
  autoware::motion_utils::VehicleStopCheckerTemplate<agnocast::Node> vehicle_stop_checker_;
  double stop_check_duration_;
};

}  // namespace autoware::default_adapi

#endif  // ROUTING_HPP_
