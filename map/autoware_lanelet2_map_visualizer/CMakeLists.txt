cmake_minimum_required(VERSION 3.14)
project(autoware_lanelet2_map_visualizer)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

ament_auto_add_library(lanelet2_map_visualization_node SHARED
  src/lanelet2_map_visualization_node.cpp
)

ament_target_dependencies(lanelet2_map_visualization_node agnocastlib)

rclcpp_components_register_node(lanelet2_map_visualization_node
  PLUGIN "autoware::lanelet2_map_visualizer::Lanelet2MapVisualizationNode"
  EXECUTABLE ${PROJECT_NAME}
)

agnocast_components_register_node(lanelet2_map_visualization_node
  PLUGIN "autoware::lanelet2_map_visualizer::Lanelet2MapVisualizationNode"
  EXECUTABLE ${PROJECT_NAME}_agnocast
  EXECUTOR AgnocastOnlySingleThreadedExecutor
  RESOURCE_INDEX agnocast_components
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_node_initialization
    test/test_node_initialization.cpp
  )
  target_link_libraries(test_node_initialization
    lanelet2_map_visualization_node
  )

  # test disabled: agnocast::Node incompatible with rclcpp test harness
  # ament_add_gtest(test_lanelet2_map_visualization_node
  #   test/test_lanelet2_map_visualization_node.cpp
  # )
  # target_link_libraries(test_lanelet2_map_visualization_node
  #   lanelet2_map_visualization_node
  # )

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    test/lanelet2_map_visualizer_launch.test.py
    TIMEOUT "30"
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
