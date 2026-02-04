// Copyright 2015-2021 Autoware Foundation
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

#ifndef AUTOWARE__VEHICLE_INFO_UTILS__VEHICLE_INFO_UTILS_HPP_
#define AUTOWARE__VEHICLE_INFO_UTILS__VEHICLE_INFO_UTILS_HPP_

#include "autoware/vehicle_info_utils/vehicle_info.hpp"

#include <rclcpp/node.hpp>

#include <string>

namespace autoware::vehicle_info_utils
{

namespace detail
{
/// Helper function template to get parameter from node
template <typename T, typename NodeT>
T getParameter(NodeT & node, const std::string & name)
{
  if (node.has_parameter(name)) {
    return node.get_parameter(name).template get_value<T>();
  }

  try {
    return node.template declare_parameter<T>(name);
  } catch (const rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(
      node.get_logger(), "Failed to get parameter `%s`, please set it when you launch the node.",
      name.c_str());
    throw;
  }
}
}  // namespace detail

/// Template class for vehicle info utilities
/// \tparam NodeT The node type (rclcpp::Node or agnocast::Node)
template <typename NodeT>
class VehicleInfoUtilsTemplate
{
public:
  /// Constructor
  explicit VehicleInfoUtilsTemplate(NodeT & node)
  {
    const auto wheel_radius_m = detail::getParameter<double>(node, "wheel_radius");
    const auto wheel_width_m = detail::getParameter<double>(node, "wheel_width");
    const auto wheel_base_m = detail::getParameter<double>(node, "wheel_base");
    const auto wheel_tread_m = detail::getParameter<double>(node, "wheel_tread");
    const auto front_overhang_m = detail::getParameter<double>(node, "front_overhang");
    const auto rear_overhang_m = detail::getParameter<double>(node, "rear_overhang");
    const auto left_overhang_m = detail::getParameter<double>(node, "left_overhang");
    const auto right_overhang_m = detail::getParameter<double>(node, "right_overhang");
    const auto vehicle_height_m = detail::getParameter<double>(node, "vehicle_height");
    const auto max_steer_angle_rad = detail::getParameter<double>(node, "max_steer_angle");

    vehicle_info_ = createVehicleInfo(
      wheel_radius_m, wheel_width_m, wheel_base_m, wheel_tread_m, front_overhang_m, rear_overhang_m,
      left_overhang_m, right_overhang_m, vehicle_height_m, max_steer_angle_rad);
  }

  /// Get vehicle info
  [[nodiscard]] VehicleInfo getVehicleInfo() const { return vehicle_info_; }

private:
  /// Buffer for base parameters
  VehicleInfo vehicle_info_;
};

/// Backward compatibility: VehicleInfoUtils is the rclcpp::Node version
using VehicleInfoUtils = VehicleInfoUtilsTemplate<rclcpp::Node>;

}  // namespace autoware::vehicle_info_utils

#endif  // AUTOWARE__VEHICLE_INFO_UTILS__VEHICLE_INFO_UTILS_HPP_
