// Copyright 2026 TIER IV, Inc.
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

#include "autoware/map_loader/map_loader_plugin_base.hpp"

#include <autoware/component_interface_specs/map.hpp>
#include <autoware/map_projection_loader/map_projection_loader.hpp>
#include <autoware_utils/ros/parameter.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace autoware::map_loader::plugin
{

class MapProjectionLoaderPlugin : public MapLoaderPluginBase
{
public:
  void initialize(const std::string & /*name*/, rclcpp::Node * node, MapLoaderData & /*data*/) override
  {
    using autoware_utils_rclcpp::get_or_declare_parameter;
    const std::string yaml_filename =
      get_or_declare_parameter<std::string>(*node, "map_projector_info_path");
    const std::string lanelet2_map_filename =
      get_or_declare_parameter<std::string>(*node, "lanelet2_map_path");

    projector_info_ = autoware::map_projection_loader::load_map_projector_info(
      yaml_filename, lanelet2_map_filename);

    using MapProjectorInfo = autoware::component_interface_specs::map::MapProjectorInfo;
    publisher_ = node->create_publisher<MapProjectorInfo::Message>(
      MapProjectorInfo::name, autoware::component_interface_specs::get_qos<MapProjectorInfo>());
  }

  void on_startup(MapLoaderData & data) override
  {
    publisher_->publish(projector_info_);
    data.projector_info = projector_info_;
    RCLCPP_INFO(
      get_node_ptr()->get_logger(), "Published map projector info (type: %s)",
      projector_info_.projector_type.c_str());
  }

private:
  autoware_map_msgs::msg::MapProjectorInfo projector_info_;
  rclcpp::Publisher<autoware_map_msgs::msg::MapProjectorInfo>::SharedPtr publisher_;
};

}  // namespace autoware::map_loader::plugin

PLUGINLIB_EXPORT_CLASS(
  autoware::map_loader::plugin::MapProjectionLoaderPlugin,
  autoware::map_loader::plugin::MapLoaderPluginBase)
