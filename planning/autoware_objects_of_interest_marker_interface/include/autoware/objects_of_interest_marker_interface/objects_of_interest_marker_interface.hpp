// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__OBJECTS_OF_INTEREST_MARKER_INTERFACE_HPP_
#define AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__OBJECTS_OF_INTEREST_MARKER_INTERFACE_HPP_
#include "autoware/objects_of_interest_marker_interface/coloring.hpp"
#include "autoware/objects_of_interest_marker_interface/marker_data.hpp"
#include "autoware/objects_of_interest_marker_interface/marker_utils.hpp"

#include <agnocast/agnocast.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace autoware::objects_of_interest_marker_interface
{

// Publisher traits for node type dispatch
template <typename NodeT>
struct ObjectsOfInterestPublisherTraits
{
  using MarkerPublisherPtr =
    typename rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr;
  static constexpr bool is_agnocast = false;
};

template <>
struct ObjectsOfInterestPublisherTraits<agnocast::Node>
{
  using MarkerPublisherPtr =
    typename agnocast::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr;
  static constexpr bool is_agnocast = true;
};

template <typename NodeT = rclcpp::Node>
class ObjectsOfInterestMarkerInterfaceTemplate
{
  using Traits = ObjectsOfInterestPublisherTraits<NodeT>;

public:
  ObjectsOfInterestMarkerInterfaceTemplate(NodeT * node, const std::string & name)
  : name_{name}
  {
    pub_marker_ = node->template create_publisher<visualization_msgs::msg::MarkerArray>(
      topic_namespace_ + "/" + name, 1);
  }

  void insertObjectData(
    const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape,
    const ColorName & color_name)
  {
    insertObjectDataWithCustomColor(pose, shape, getColor(color_name));
  }

  void insertObjectDataWithCustomColor(
    const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape,
    const std_msgs::msg::ColorRGBA & color)
  {
    ObjectMarkerData data;
    data.pose = pose;
    data.shape = shape;
    data.color = color;
    obj_marker_data_array_.push_back(data);
  }

  void publishMarkerArray()
  {
    if (pub_marker_->get_subscription_count() == 0) {
      return;
    }
    visualization_msgs::msg::MarkerArray marker_array;
    for (size_t i = 0; i < obj_marker_data_array_.size(); ++i) {
      const auto data = obj_marker_data_array_.at(i);
      const visualization_msgs::msg::MarkerArray target_marker =
        marker_utils::createTargetMarker(i, data, getName(), getHeightOffset());
      marker_array.markers.insert(
        marker_array.markers.end(), target_marker.markers.begin(), target_marker.markers.end());
    }
    if constexpr (Traits::is_agnocast) {
      auto loaned = pub_marker_->borrow_loaned_message();
      *loaned = marker_array;
      pub_marker_->publish(std::move(loaned));
    } else {
      pub_marker_->publish(marker_array);
    }
    obj_marker_data_array_.clear();
  }

  void setHeightOffset(const double offset) { height_offset_ = offset; }

  static std_msgs::msg::ColorRGBA getColor(const ColorName & color_name, const float alpha = 0.99f)
  {
    switch (color_name) {
      case ColorName::GRAY:
        return coloring::getGray(alpha);
      case ColorName::GREEN:
        return coloring::getGreen(alpha);
      case ColorName::AMBER:
        return coloring::getAmber(alpha);
      case ColorName::RED:
        return coloring::getRed(alpha);
      default:
        return coloring::getGray(alpha);
    }
  }

  std::string getName() const { return name_; }
  double getHeightOffset() const { return height_offset_; }

private:
  typename Traits::MarkerPublisherPtr pub_marker_;

  double height_offset_{0.5};
  std::vector<ObjectMarkerData> obj_marker_data_array_;

  std::string name_;
  std::string topic_namespace_ = "/planning/debug/objects_of_interest";
};

using ObjectsOfInterestMarkerInterface = ObjectsOfInterestMarkerInterfaceTemplate<rclcpp::Node>;

}  // namespace autoware::objects_of_interest_marker_interface

#endif  // AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__OBJECTS_OF_INTEREST_MARKER_INTERFACE_HPP_
