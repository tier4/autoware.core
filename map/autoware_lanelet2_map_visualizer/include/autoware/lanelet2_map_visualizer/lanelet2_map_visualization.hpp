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

#ifndef AUTOWARE__LANELET2_MAP_VISUALIZER__LANELET2_MAP_VISUALIZATION_HPP_
#define AUTOWARE__LANELET2_MAP_VISUALIZER__LANELET2_MAP_VISUALIZATION_HPP_

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace autoware::lanelet2_map_visualizer
{

visualization_msgs::msg::MarkerArray create_vector_map_marker_array(
  const autoware_map_msgs::msg::LaneletMapBin & map_bin_msg, bool viz_lanelets_centerline = true);

}  // namespace autoware::lanelet2_map_visualizer

#endif  // AUTOWARE__LANELET2_MAP_VISUALIZER__LANELET2_MAP_VISUALIZATION_HPP_
