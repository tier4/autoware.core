cmake_minimum_required(VERSION 3.14)
project(autoware_object_recognition_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/predicted_path_utils.cpp
  src/conversion.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  file(GLOB_RECURSE test_files test/**/*.cpp)

  ament_add_ros_isolated_gtest(test_object_recognition_utils ${test_files})

  target_link_libraries(test_object_recognition_utils
    ${PROJECT_NAME}
  )
endif()

# Explicitly export tf2_ros dependency before autoware_ament_auto_package
ament_export_dependencies(tf2_ros)

autoware_ament_auto_package()

# Debug:  Check what was actually exported
message(WARNING "=== POST-PACKAGE DEBUG autoware_object_recognition_utils ===")
get_directory_property(export_targets DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} EXPORT_TARGETS)
message(WARNING "EXPORT_TARGETS: ${export_targets}")

# Check if the config file will be generated
set(config_file "${CMAKE_CURRENT_BINARY_DIR}/autoware_object_recognition_utilsConfig.cmake")
if(EXISTS "${config_file}")
  message(WARNING "Config file exists at:  ${config_file}")
  file(READ "${config_file}" config_contents)
  message(WARNING "Config file contents: ${config_contents}")
endif()

message(WARNING "=== END POST-PACKAGE DEBUG ===")
