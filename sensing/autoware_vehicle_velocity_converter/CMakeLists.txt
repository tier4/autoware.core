cmake_minimum_required(VERSION 3.14)
project(autoware_vehicle_velocity_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/vehicle_velocity_converter.cpp
)
ament_target_dependencies(${PROJECT_NAME} agnocastlib)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::vehicle_velocity_converter::VehicleVelocityConverter"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # NOTE: test disabled (rclcpp::Executor incompatible with agnocast::Node)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
