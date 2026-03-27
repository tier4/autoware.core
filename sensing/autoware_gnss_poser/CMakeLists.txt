cmake_minimum_required(VERSION 3.14)
project(autoware_gnss_poser)

find_package(autoware_cmake REQUIRED)
autoware_package()

## Find non-ROS library
find_package(GeographicLib REQUIRED)
find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

set(GNSS_POSER_HEADERS
  include/autoware/gnss_poser/gnss_poser_node.hpp
)

ament_auto_add_library(gnss_poser_node SHARED
  src/gnss_poser_node.cpp
  ${GNSS_POSER_HEADERS}
)
ament_target_dependencies(gnss_poser_node agnocastlib)

agnocast_components_register_node(gnss_poser_node
  PLUGIN "autoware::gnss_poser::GNSSPoser"
  EXECUTABLE gnss_poser
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # NOTE: test disabled (rclcpp::Executor incompatible with agnocast::Node)
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
