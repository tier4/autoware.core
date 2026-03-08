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

#include "stop_filter.hpp"

#include <cmath>
#include <functional>
#include <memory>

namespace autoware::stop_filter
{
StopFilter::StopFilter(const rclcpp::NodeOptions & node_options)
: Node("stop_filter", node_options)
{
  vx_threshold_ = declare_parameter<double>("vx_threshold");
  wz_threshold_ = declare_parameter<double>("wz_threshold");

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "input/odom", 1, std::bind(&StopFilter::callback_odometry, this, std::placeholders::_1));

  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("output/odom", 1);
  pub_stop_flag_ =
    create_publisher<autoware_internal_debug_msgs::msg::BoolStamped>("debug/stop_flag", 1);
}

void StopFilter::callback_odometry(
  const agnocast::ipc_shared_ptr<nav_msgs::msg::Odometry> & msg)
{
  const bool is_stopped =
    std::fabs(msg->twist.twist.linear.x) < vx_threshold_ &&
    std::fabs(msg->twist.twist.angular.z) < wz_threshold_;

  {
    auto stop_flag_msg = pub_stop_flag_->borrow_loaned_message();
    stop_flag_msg->stamp = msg->header.stamp;
    stop_flag_msg->data = is_stopped;
    pub_stop_flag_->publish(std::move(stop_flag_msg));
  }

  {
    auto odom_msg = pub_odom_->borrow_loaned_message();
    *odom_msg = *msg;
    if (is_stopped) {
      odom_msg->twist.twist.linear.x = 0.0;
      odom_msg->twist.twist.angular.z = 0.0;
    }
    pub_odom_->publish(std::move(odom_msg));
  }
}
}  // namespace autoware::stop_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::stop_filter::StopFilter)
