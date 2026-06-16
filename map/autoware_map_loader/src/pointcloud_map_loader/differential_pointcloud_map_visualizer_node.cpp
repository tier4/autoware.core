// Copyright 2022 The Autoware Contributors
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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <autoware_map_msgs/srv/get_differential_point_cloud_map.hpp>
#include <autoware_map_msgs/msg/area_info.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace autoware::map_loader
{

class DifferentialPointcloudMapVisualizerNode : public rclcpp::Node
{
public:
  explicit DifferentialPointcloudMapVisualizerNode(const rclcpp::NodeOptions & options)
  : Node("differential_pointcloud_map_visualizer", options)
  {
    center_x_ = declare_parameter<double>("center_x", 0.0);
    center_y_ = declare_parameter<double>("center_y", 0.0);
    radius_ = declare_parameter<double>("radius", 200.0);
    use_pose_ = declare_parameter<bool>("use_pose", false);
    update_interval_sec_ = declare_parameter<double>("update_interval_sec", 1.0);

    rclcpp::QoS durable_qos{1};
    durable_qos.transient_local();
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "output/differential_pointcloud_map", durable_qos);

    client_ = create_client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>(
      "/map/get_differential_pointcloud_map");

    if (use_pose_) {
      sub_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "input/pose", 10, std::bind(&DifferentialPointcloudMapVisualizerNode::on_pose, this, std::placeholders::_1));
    }

    timer_ = create_wall_timer(
      std::chrono::duration<double>(update_interval_sec_),
      std::bind(&DifferentialPointcloudMapVisualizerNode::on_timer, this));
  }

private:
  void on_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    center_x_ = msg->pose.pose.position.x;
    center_y_ = msg->pose.pose.position.y;
  }

  void on_timer()
  {
    if (!client_->service_is_ready()) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "Differential map service not ready");
      return;
    }

    auto request = std::make_shared<autoware_map_msgs::srv::GetDifferentialPointCloudMap::Request>();
    request->area.center_x = static_cast<float>(center_x_);
    request->area.center_y = static_cast<float>(center_y_);
    request->area.radius = static_cast<float>(radius_);
    request->cached_ids = {};  // Empty: get all cells in area (for visualization)

    auto result = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), result, std::chrono::seconds(5)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Differential map service call failed");
      return;
    }

    const auto & response = result.get();
    sensor_msgs::msg::PointCloud2 merged;
    for (const auto & cell : response->new_pointcloud_with_ids) {
      if (merged.width == 0) {
        merged = cell.pointcloud;
      } else {
        merged.width += cell.pointcloud.width;
        merged.row_step += cell.pointcloud.row_step;
        merged.data.insert(
          merged.data.end(), cell.pointcloud.data.begin(), cell.pointcloud.data.end());
      }
    }
    if (merged.width > 0) {
      merged.header = response->header;
      merged.header.stamp = now();
      pub_->publish(merged);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>::SharedPtr client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
  rclcpp::TimerBase::SharedPtr timer_;
  double center_x_;
  double center_y_;
  double radius_;
  bool use_pose_;
  double update_interval_sec_;
};

}  // namespace autoware::map_loader

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::map_loader::DifferentialPointcloudMapVisualizerNode)
