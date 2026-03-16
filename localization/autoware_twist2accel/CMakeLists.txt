cmake_minimum_required(VERSION 3.14)
project(autoware_twist2accel)

find_package(autoware_cmake REQUIRED)
find_package(autoware_agnocast_wrapper REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/twist2accel.cpp
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::twist2accel::Twist2Accel"
  EXECUTABLE ${PROJECT_NAME}_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR CallbackIsolatedAgnocastExecutor
)

autoware_ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
