cmake_minimum_required(VERSION 3.14)
project(autoware_default_adapi)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/interface.cpp
  src/localization.cpp
  src/routing.cpp
  src/utils/localization_conversion.cpp
  src/utils/route_conversion.cpp
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::InterfaceNode"
  EXECUTABLE interface_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::LocalizationNode"
  EXECUTABLE localization_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::RoutingNode"
  EXECUTABLE routing_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
)

if(BUILD_TESTING)
  add_launch_test(test/main.test.py)
endif()

ament_auto_package(INSTALL_TO_SHARE config launch test)
