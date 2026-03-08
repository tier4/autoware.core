// Copyright 2021 TierIV
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

#ifndef STOP_FILTER_HPP_
#define STOP_FILTER_HPP_

#include <agnocast/agnocast.hpp>

#include <autoware_internal_debug_msgs/msg/bool_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace autoware::stop_filter
{
class StopFilter : public agnocast::Node
{
public:
  explicit StopFilter(const rclcpp::NodeOptions & node_options);

private:
  agnocast::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  agnocast::Publisher<autoware_internal_debug_msgs::msg::BoolStamped>::SharedPtr pub_stop_flag_;
  agnocast::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  double vx_threshold_;
  double wz_threshold_;

  void callback_odometry(const agnocast::ipc_shared_ptr<nav_msgs::msg::Odometry> & msg);
};
}  // namespace autoware::stop_filter
#endif  // STOP_FILTER_HPP_
