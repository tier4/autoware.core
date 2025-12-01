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

#include "gnss_module.hpp"

#include <autoware/component_interface_specs/localization.hpp>

#include <autoware_adapi_v1_msgs/msg/response_status.hpp>

#include <memory>

namespace autoware::pose_initializer
{
GnssModule::GnssModule(rclcpp::Node * node)
: fitter_(node),
  clock_(node->get_clock()),
  timeout_(node->declare_parameter<double>("gnss_pose_timeout"))
{
  RCLCPP_INFO(node->get_logger(), "GnssModule: Creating subscription to 'gnss_pose_cov' topic, timeout=%.1f", timeout_);
  sub_gnss_pose_ = node->create_subscription<PoseWithCovarianceStamped>(
    "gnss_pose_cov", 1, std::bind(&GnssModule::on_pose, this, std::placeholders::_1));
}

void GnssModule::on_pose(PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  RCLCPP_INFO(
    rclcpp::get_logger("gnss_module"),
    "GNSS pose received: stamp=%.3f, position=(%.3f, %.3f, %.3f)",
    msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.position.z);
  pose_ = msg;
}

geometry_msgs::msg::PoseWithCovarianceStamped GnssModule::get_pose()
{
  using Initialize = autoware::component_interface_specs::localization::Initialize;

  if (!pose_) {
    RCLCPP_ERROR(rclcpp::get_logger("gnss_module"), "get_pose: No GNSS pose received yet");
    autoware_adapi_v1_msgs::msg::ResponseStatus respose_status;
    respose_status.success = false;
    respose_status.code = Initialize::Service::Response::ERROR_GNSS;
    respose_status.message = "The GNSS pose has not arrived.";
    throw respose_status;
  }

  const auto elapsed = rclcpp::Time(pose_->header.stamp) - clock_->now();
  RCLCPP_INFO(
    rclcpp::get_logger("gnss_module"),
    "get_pose: Checking timeout - elapsed=%.3fs, timeout=%.1fs",
    elapsed.seconds(), timeout_);
  if (timeout_ < elapsed.seconds()) {
    autoware_adapi_v1_msgs::msg::ResponseStatus respose_status;
    respose_status.success = false;
    respose_status.code = Initialize::Service::Response::ERROR_GNSS;
    respose_status.message = "The GNSS pose is out of date.";
    throw respose_status;
  }

  PoseWithCovarianceStamped pose = *pose_;
  const auto fitted = fitter_.fit(pose.pose.pose.position, pose.header.frame_id);
  if (fitted) {
    pose.pose.pose.position = fitted.value();
  }
  return pose;
}
}  // namespace autoware::pose_initializer
