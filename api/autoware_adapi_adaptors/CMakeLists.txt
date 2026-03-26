cmake_minimum_required(VERSION 3.14)
project(autoware_adapi_adaptors)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

# Initial pose adaptor (rclcpp only)
ament_auto_add_library(${PROJECT_NAME}_initial_pose SHARED
  src/initial_pose_adaptor.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_initial_pose
  PLUGIN "autoware::adapi_adaptors::InitialPoseAdaptor"
  EXECUTABLE initial_pose_adaptor_node
  EXECUTOR MultiThreadedExecutor
)

# Routing adaptor (agnocast)
ament_auto_add_library(${PROJECT_NAME}_routing SHARED
  src/routing_adaptor.cpp
)

target_link_libraries(${PROJECT_NAME}_routing
  ${agnocastlib_LIBRARIES}
)
target_include_directories(${PROJECT_NAME}_routing SYSTEM PUBLIC
  ${agnocastlib_INCLUDE_DIRS}
)

agnocast_components_register_node(${PROJECT_NAME}_routing
  PLUGIN "autoware::adapi_adaptors::RoutingAdaptor"
  EXECUTABLE routing_adaptor_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

ament_auto_package(INSTALL_TO_SHARE config launch)
