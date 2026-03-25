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

#ifndef AUTOWARE__MOTION_UTILS__VEHICLE__VEHICLE_STATE_CHECKER_HPP_
#define AUTOWARE__MOTION_UTILS__VEHICLE__VEHICLE_STATE_CHECKER_HPP_

#include "autoware/motion_utils/trajectory/trajectory.hpp"

#include <agnocast/agnocast.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <deque>
#include <functional>

namespace autoware::motion_utils
{

using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Odometry;

// Node type traits for subscription and message pointer types
template <typename NodeT>
struct VehicleCheckerNodeTraits
{
  template <typename T>
  using SubscriptionPtr = typename rclcpp::Subscription<T>::SharedPtr;
  template <typename T>
  using MessagePtr = typename T::ConstSharedPtr;
};

template <>
struct VehicleCheckerNodeTraits<agnocast::Node>
{
  template <typename T>
  using SubscriptionPtr = typename agnocast::Subscription<T>::SharedPtr;
  template <typename T>
  using MessagePtr = agnocast::ipc_shared_ptr<const T>;
};

class VehicleStopCheckerBase
{
public:
  template <typename NodeT>
  VehicleStopCheckerBase(NodeT * node, double buffer_duration)
  : clock_(node->get_clock()), logger_(node->get_logger()), buffer_duration_(buffer_duration)
  {
  }

  rclcpp::Logger getLogger() { return logger_; }
  void addTwist(const TwistStamped & twist);
  [[nodiscard]] bool isVehicleStopped(const double stop_duration = 0.0) const;

protected:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

private:
  double buffer_duration_;
  std::deque<TwistStamped> twist_buffer_;
};

template <typename NodeT = rclcpp::Node>
class VehicleStopCheckerTemplate : public VehicleStopCheckerBase
{
  using Traits = VehicleCheckerNodeTraits<NodeT>;

public:
  explicit VehicleStopCheckerTemplate(NodeT * node)
  : VehicleStopCheckerBase(node, velocity_buffer_time_sec)
  {
    using std::placeholders::_1;
    sub_odom_ = node->template create_subscription<Odometry>(
      "/localization/kinematic_state", rclcpp::QoS(1),
      std::bind(&VehicleStopCheckerTemplate::onOdom, this, _1));
  }

protected:
  typename Traits::template SubscriptionPtr<Odometry> sub_odom_;
  typename Traits::template MessagePtr<Odometry> odometry_ptr_;

private:
  static constexpr double velocity_buffer_time_sec = 10.0;

  void onOdom(const typename Traits::template MessagePtr<Odometry> & msg)
  {
    odometry_ptr_ = msg;

    TwistStamped current_velocity;
    current_velocity.header = msg->header;
    current_velocity.twist = msg->twist.twist;
    addTwist(current_velocity);
  }
};

using VehicleStopChecker = VehicleStopCheckerTemplate<rclcpp::Node>;

template <typename NodeT = rclcpp::Node>
class VehicleArrivalCheckerTemplate : public VehicleStopCheckerTemplate<NodeT>
{
  using Traits = VehicleCheckerNodeTraits<NodeT>;

public:
  explicit VehicleArrivalCheckerTemplate(NodeT * node)
  : VehicleStopCheckerTemplate<NodeT>(node)
  {
    using std::placeholders::_1;
    sub_trajectory_ = node->template create_subscription<Trajectory>(
      "/planning/trajectory", rclcpp::QoS(1),
      std::bind(&VehicleArrivalCheckerTemplate::onTrajectory, this, _1));
  }

  [[nodiscard]] bool isVehicleStoppedAtStopPoint(const double stop_duration = 0.0) const
  {
    if (!this->odometry_ptr_ || !trajectory_ptr_) {
      return false;
    }

    if (!this->isVehicleStopped(stop_duration)) {
      return false;
    }

    const auto & p = this->odometry_ptr_->pose.pose.position;
    const auto idx =
      autoware::motion_utils::searchZeroVelocityIndex(trajectory_ptr_->points);

    if (!idx) {
      return false;
    }

    return std::abs(autoware::motion_utils::calcSignedArcLength(
             trajectory_ptr_->points, p, idx.value())) < th_arrived_distance_m;
  }

private:
  static constexpr double th_arrived_distance_m = 1.0;

  typename Traits::template SubscriptionPtr<Trajectory> sub_trajectory_;
  typename Traits::template MessagePtr<Trajectory> trajectory_ptr_;

  void onTrajectory(const typename Traits::template MessagePtr<Trajectory> & msg)
  {
    trajectory_ptr_ = msg;
  }
};

using VehicleArrivalChecker = VehicleArrivalCheckerTemplate<rclcpp::Node>;

}  // namespace autoware::motion_utils

#endif  // AUTOWARE__MOTION_UTILS__VEHICLE__VEHICLE_STATE_CHECKER_HPP_
