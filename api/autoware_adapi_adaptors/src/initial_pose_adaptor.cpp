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

#include "initial_pose_adaptor.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::adapi_adaptors
{
std::array<double, 36> get_covariance_parameter(agnocast::Node * node, const std::string & name)
{
  const auto vector = node->declare_parameter<std::vector<double>>(name);
  if (vector.size() != 36) {
    throw std::invalid_argument("The covariance parameter size is not 36.");
  }
  std::array<double, 36> array;
  std::copy_n(vector.begin(), array.size(), array.begin());
  return array;
}

InitialPoseAdaptor::InitialPoseAdaptor(const rclcpp::NodeOptions & options)
: agnocast::Node("autoware_initial_pose_adaptor", options), fitter_(this)
{
  rviz_particle_covariance_ = get_covariance_parameter(this, "initial_pose_particle_covariance");
  sub_initial_pose_ = create_subscription<PoseWithCovarianceStamped>(
    "~/initialpose", rclcpp::QoS(1),
    std::bind(&InitialPoseAdaptor::on_initial_pose, this, std::placeholders::_1));

  cli_initialize_ = create_client<Initialize::Service>(Initialize::name);
}

void InitialPoseAdaptor::on_initial_pose(
  const agnocast::ipc_shared_ptr<const PoseWithCovarianceStamped> & msg)
{
  PoseWithCovarianceStamped pose = *msg;
  const auto fitted = fitter_.fit(pose.pose.pose.position, pose.header.frame_id);
  if (fitted) {
    pose.pose.pose.position = fitted.value();
  }
  pose.pose.covariance = rviz_particle_covariance_;

  auto req = cli_initialize_->borrow_loaned_request();
  req->pose.push_back(pose);
  cli_initialize_->async_send_request(std::move(req));
}

}  // namespace autoware::adapi_adaptors

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::adapi_adaptors::InitialPoseAdaptor)
