// Copyright 2018-2019 Autoware Foundation
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

#ifndef AUTOWARE__EKF_LOCALIZER__EKF_LOCALIZER_HPP_
#define AUTOWARE__EKF_LOCALIZER__EKF_LOCALIZER_HPP_

#include "autoware/ekf_localizer/ekf_localizer_core.hpp"

#include <rclcpp/rclcpp.hpp>

namespace autoware::ekf_localizer
{

class EKFLocalizer : public rclcpp::Node
{
public:
  explicit EKFLocalizer(const rclcpp::NodeOptions & options);

  std::chrono::nanoseconds time_until_trigger() const { return core_.time_until_trigger(); }

private:
  EKFLocalizerCore core_;
};

}  // namespace autoware::ekf_localizer

#endif  // AUTOWARE__EKF_LOCALIZER__EKF_LOCALIZER_HPP_
