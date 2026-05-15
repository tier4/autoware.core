// Copyright 2025 TIER IV, Inc.
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

#pragma once

#include "autoware/agnocast_wrapper/node.hpp"

#include <rclcpp/qos.hpp>

#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

#ifdef USE_AGNOCAST_ENABLED
#include <agnocast/node/tf2/transform_listener.hpp>
#endif

namespace autoware::agnocast_wrapper
{

// tf2_ros::Buffer inherits tf2::BufferCore, which is the base type accepted by both
// tf2_ros::TransformListener and agnocast::TransformListener. Using tf2_ros::Buffer lets us
// share the same buffer type across both transports.
using Buffer = tf2_ros::Buffer;

/// @brief TransformListener that dispatches between agnocast::TransformListener and
/// tf2_ros::TransformListener depending on whether the given Node is running in agnocast mode.
///
/// Constructor signature is the same in both modes so the caller writes:
///   autoware::agnocast_wrapper::TransformListener tf_listener_(tf_buffer_, *this);
/// where `*this` is a node derived from autoware::agnocast_wrapper::Node.
class TransformListener
{
public:
  TransformListener(tf2::BufferCore & buffer, Node & node, bool spin_thread = true)
  {
#ifdef USE_AGNOCAST_ENABLED
    if (node.is_using_agnocast()) {
      agnocast_impl_ = std::make_unique<agnocast::TransformListener>(
        buffer, *node.get_agnocast_node(), spin_thread);
      return;
    }
    ros2_impl_ = std::make_unique<tf2_ros::TransformListener>(
      buffer, node.get_rclcpp_node().get(), spin_thread);
#else
    // When Agnocast is disabled, Node is an alias for rclcpp::Node.
    ros2_impl_ = std::make_unique<tf2_ros::TransformListener>(buffer, &node, spin_thread);
#endif
  }

  ~TransformListener() = default;

  TransformListener(const TransformListener &) = delete;
  TransformListener & operator=(const TransformListener &) = delete;

private:
  std::unique_ptr<tf2_ros::TransformListener> ros2_impl_;
#ifdef USE_AGNOCAST_ENABLED
  std::unique_ptr<agnocast::TransformListener> agnocast_impl_;
#endif
};

}  // namespace autoware::agnocast_wrapper
