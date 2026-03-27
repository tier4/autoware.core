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

#include <rclcpp/rclcpp.hpp>

#if __has_include(<agnocast/agnocast.hpp>)
#include <agnocast/agnocast.hpp>
#define OBJECTS_OF_INTEREST_HAS_AGNOCAST
#endif

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <type_traits>
#include <vector>

namespace autoware::objects_of_interest_marker_interface
{

/// @brief Traits to select publisher type based on NodeT
template <typename NodeT>
struct ObjectsOfInterestPublisherTraits
{
  using PublisherPtr = typename rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr;
  static constexpr bool is_agnocast = false;
};

#ifdef OBJECTS_OF_INTEREST_HAS_AGNOCAST
template <>
struct ObjectsOfInterestPublisherTraits<agnocast::Node>
{
  using PublisherPtr =
    typename agnocast::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr;
  static constexpr bool is_agnocast = true;
};
#endif

template <typename NodeT = rclcpp::Node>
class ObjectsOfInterestMarkerInterfaceTemplate
{
  using Traits = ObjectsOfInterestPublisherTraits<NodeT>;

public:
  /**
   * @brief Constructor
   * @param node Node that publishes marker
   * @param name Module name
   */
  ObjectsOfInterestMarkerInterfaceTemplate(NodeT * node, const std::string & name)
  : name_{name}
  {
    pub_marker_ = node->template create_publisher<visualization_msgs::msg::MarkerArray>(
      topic_namespace_ + "/" + name, 1);
  }

  /**
   * @brief Insert object data to visualize
   * @param pose Object pose
   * @param shape Object shape
   * @param color_name Color name
   */
  void insertObjectData(
    const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape,
    const ColorName & color_name)
  {
    insertObjectDataWithCustomColor(pose, shape, getColor(color_name));
  }

  /**
   * @brief Insert object data to visualize with custom color data
   * @param pose Object pose
   * @param shape Object shape
   * @param color Color data with alpha
   */
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

  /**
   * @brief Publish interest objects marker
   */
  void publishMarkerArray()
  {
    if constexpr (Traits::is_agnocast) {
      visualization_msgs::msg::MarkerArray marker_array;
      for (size_t i = 0; i < obj_marker_data_array_.size(); ++i) {
        const auto data = obj_marker_data_array_.at(i);
        const auto target_marker =
          marker_utils::createTargetMarker(i, data, getName(), getHeightOffset());
        marker_array.markers.insert(
          marker_array.markers.end(), target_marker.markers.begin(), target_marker.markers.end());
      }
      auto loaned = pub_marker_->borrow_loaned_message();
      *loaned = marker_array;
      pub_marker_->publish(std::move(loaned));
    } else {
      if (pub_marker_->get_subscription_count() == 0) {
        return;
      }
      visualization_msgs::msg::MarkerArray marker_array;
      for (size_t i = 0; i < obj_marker_data_array_.size(); ++i) {
        const auto data = obj_marker_data_array_.at(i);
        const auto target_marker =
          marker_utils::createTargetMarker(i, data, getName(), getHeightOffset());
        marker_array.markers.insert(
          marker_array.markers.end(), target_marker.markers.begin(), target_marker.markers.end());
      }
      pub_marker_->publish(marker_array);
    }
    obj_marker_data_array_.clear();
  }

  /**
   * @brief Set height offset of markers
   * @param offset Height offset of markers
   */
  void setHeightOffset(const double offset) { height_offset_ = offset; }

  /**
   * @brief Get color data from color name
   * @param color_name Color name
   * @param alpha Alpha
   */
  static std_msgs::msg::ColorRGBA getColor(
    const ColorName & color_name, const float alpha = 0.99f)
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

  /**
   * @brief Get module name including this interface
   */
  std::string getName() const { return name_; }

  /**
   * @brief Get height offset
   */
  double getHeightOffset() const { return height_offset_; }

private:
  typename Traits::PublisherPtr pub_marker_;

  double height_offset_{0.5};
  std::vector<ObjectMarkerData> obj_marker_data_array_;

  std::string name_;
  std::string topic_namespace_ = "/planning/debug/objects_of_interest";
};

// Backward-compatible alias for rclcpp::Node
using ObjectsOfInterestMarkerInterface = ObjectsOfInterestMarkerInterfaceTemplate<rclcpp::Node>;

}  // namespace autoware::objects_of_interest_marker_interface

#endif  // AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__OBJECTS_OF_INTEREST_MARKER_INTERFACE_HPP_
