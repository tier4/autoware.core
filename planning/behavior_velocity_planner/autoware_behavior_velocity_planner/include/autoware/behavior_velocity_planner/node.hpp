// Copyright 2019 Autoware Foundation
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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__NODE_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__NODE_HPP_

#include "autoware/behavior_velocity_planner/planner_manager.hpp"

#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <agnocast/agnocast.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_internal_planning_msgs/srv/load_plugin.hpp>
#include <autoware_internal_planning_msgs/srv/unload_plugin.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <agnocast/node/tf2/buffer.hpp>
#include <agnocast/node/tf2/transform_listener.hpp>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware_internal_debug_msgs::msg::Float64Stamped;
using autoware_internal_planning_msgs::msg::VelocityLimit;
using autoware_internal_planning_msgs::srv::LoadPlugin;
using autoware_internal_planning_msgs::srv::UnloadPlugin;
using autoware_map_msgs::msg::LaneletMapBin;

class BehaviorVelocityPlannerNode : public agnocast::Node
{
public:
  explicit BehaviorVelocityPlannerNode(const rclcpp::NodeOptions & node_options);

private:
  // tf
  agnocast::Buffer tf_buffer_;
  std::unique_ptr<agnocast::TransformListener> tf_listener_;

  // subscriber
  agnocast::Subscription<autoware_internal_planning_msgs::msg::PathWithLaneId>::SharedPtr
    trigger_sub_path_with_lane_id_;

  // polling subscribers
  agnocast::PollingSubscriber<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr
    sub_predicted_objects_;

  agnocast::PollingSubscriber<sensor_msgs::msg::PointCloud2>::SharedPtr sub_no_ground_pointcloud_;

  agnocast::PollingSubscriber<nav_msgs::msg::Odometry>::SharedPtr sub_vehicle_odometry_;

  agnocast::PollingSubscriber<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr
    sub_acceleration_;

  agnocast::PollingSubscriber<autoware_perception_msgs::msg::TrafficLightGroupArray>::SharedPtr
    sub_traffic_signals_;

  agnocast::PollingSubscriber<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_occupancy_grid_;

  agnocast::PollingSubscriber<LaneletMapBin>::SharedPtr sub_lanelet_map_;

  agnocast::PollingSubscriber<VelocityLimit>::SharedPtr sub_external_velocity_limit_;

  void onTrigger(
    const agnocast::ipc_shared_ptr<const autoware_internal_planning_msgs::msg::PathWithLaneId> &
      input_path_msg);

  void onParam();

  void processNoGroundPointCloud(
    const agnocast::ipc_shared_ptr<const sensor_msgs::msg::PointCloud2> & msg);
  void processOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void processTrafficSignals(
    const autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr msg);
  bool processData(rclcpp::Clock clock);

  // publisher
  agnocast::Publisher<autoware_planning_msgs::msg::Path>::SharedPtr path_pub_;
  agnocast::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_viz_pub_;
  agnocast::Publisher<Float64Stamped>::SharedPtr processing_time_publisher_;

  void publishDebugMarker(const autoware_planning_msgs::msg::Path & path);
  void publishProcessingTime();

  //  parameter
  double forward_path_length_;
  double backward_path_length_;
  double behavior_output_path_interval_;

  // member
  PlannerData planner_data_;
  BehaviorVelocityPlannerManager planner_manager_;
  bool is_driving_forward_{true};
  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch_;

  agnocast::Service<LoadPlugin>::SharedPtr srv_load_plugin_;
  agnocast::Service<UnloadPlugin>::SharedPtr srv_unload_plugin_;
  void onUnloadPlugin(
    const agnocast::ipc_shared_ptr<agnocast::Service<UnloadPlugin>::RequestT> & request,
    agnocast::ipc_shared_ptr<agnocast::Service<UnloadPlugin>::ResponseT> & response);
  void onLoadPlugin(
    const agnocast::ipc_shared_ptr<agnocast::Service<LoadPlugin>::RequestT> & request,
    agnocast::ipc_shared_ptr<agnocast::Service<LoadPlugin>::ResponseT> & response);

  // mutex for planner_data_
  std::mutex mutex_;

  // function
  bool isDataReady(rclcpp::Clock clock);
  autoware_planning_msgs::msg::Path generatePath(
    const agnocast::ipc_shared_ptr<const autoware_internal_planning_msgs::msg::PathWithLaneId> &
      input_path_msg,
    const PlannerData & planner_data);

  static constexpr int logger_throttle_interval = 3000;
};
}  // namespace autoware::behavior_velocity_planner

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__NODE_HPP_
