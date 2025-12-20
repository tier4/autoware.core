cmake_minimum_required(VERSION 3.14)
project(autoware_object_recognition_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)

# Let ament_auto find all dependencies from package. xml
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/predicted_path_utils. cpp
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

# Manually export with targets BEFORE autoware_ament_auto_package
# and remove from the auto-handled list to prevent double-install
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export the targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Also export old-style
ament_export_libraries(${PROJECT_NAME})

# Explicitly export tf2_ros dependency
ament_export_dependencies(tf2_ros)

# Clear the LIBRARIES list so autoware_ament_auto_package doesn't try to install again
set(${PROJECT_NAME}_LIBRARIES "")

autoware_ament_auto_package()
