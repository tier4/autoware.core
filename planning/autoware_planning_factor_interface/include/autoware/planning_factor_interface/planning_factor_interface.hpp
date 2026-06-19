// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__PLANNING_FACTOR_INTERFACE__PLANNING_FACTOR_INTERFACE_HPP_
#define AUTOWARE__PLANNING_FACTOR_INTERFACE__PLANNING_FACTOR_INTERFACE_HPP_

#include <autoware/planning_factor_interface/planning_factor_builder.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <string>

namespace autoware::planning_factor_interface
{

class PlanningFactorInterface : public PlanningFactorBuilder
{
public:
  PlanningFactorInterface(
    rclcpp::Node * node, const std::string & name, bool enable_console_output = false,
    int throttle_duration_ms = 1000)
  : PlanningFactorBuilder{name},
    pub_factors_{
      node->create_publisher<PlanningFactorArray>("/planning/planning_factors/" + name, 1)},
    clock_{node->get_clock()},
    enable_console_output_{enable_console_output},
    throttle_duration_ms_{throttle_duration_ms}
  {
  }

  /**
   * @brief publish planning factors.
   */
  void publish()
  {
    const auto msg = make_array(clock_->now());

    pub_factors_->publish(msg);

    if (enable_console_output_ && !msg.factors.empty()) {
      print_factors_to_console(msg);
    }

    clear();
  }

  /**
   * @brief get the current factors (for test purpose).
   */
  using PlanningFactorBuilder::get_factors;

private:
  /**
   * @brief Print message to console in YAML format
   * @param msg The message to print
   */
  void print_factors_to_console(const PlanningFactorArray & msg)
  {
    const std::string output_str =
      "Planning factor:\n" + autoware_internal_planning_msgs::msg::to_yaml(msg);
    if (throttle_duration_ms_ > 0) {
      RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger(name()), *clock_, throttle_duration_ms_, "%s", output_str.c_str());
    } else {
      RCLCPP_INFO(rclcpp::get_logger(name()), "%s", output_str.c_str());
    }
  }

  rclcpp::Publisher<PlanningFactorArray>::SharedPtr pub_factors_;

  rclcpp::Clock::SharedPtr clock_;

  bool enable_console_output_{false};
  int throttle_duration_ms_{0};
};

}  // namespace autoware::planning_factor_interface

#endif  // AUTOWARE__PLANNING_FACTOR_INTERFACE__PLANNING_FACTOR_INTERFACE_HPP_
