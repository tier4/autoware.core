cmake_minimum_required(VERSION 3.14)
project(autoware_object_recognition_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)

ament_auto_find_build_dependencies()

# Debug: Check if tf2_ros was found
message(WARNING "=== DEBUG autoware_object_recognition_utils ===")
message(WARNING "PROJECT_NAME: ${PROJECT_NAME}")
message(WARNING "FOUND_BUILD_DEPENDS: ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}")
message(WARNING "BUILD_EXPORT_DEPENDS: ${${PROJECT_NAME}_BUILD_EXPORT_DEPENDS}")
message(WARNING "EXEC_DEPENDS: ${${PROJECT_NAME}_EXEC_DEPENDS}")
message(WARNING "LIBRARIES before ament_auto_add_library: ${${PROJECT_NAME}_LIBRARIES}")

if(tf2_ros_FOUND)
  message(WARNING "tf2_ros_FOUND: TRUE")
  message(WARNING "tf2_ros_INCLUDE_DIRS: ${tf2_ros_INCLUDE_DIRS}")
  message(WARNING "tf2_ros_LIBRARIES: ${tf2_ros_LIBRARIES}")
  message(WARNING "tf2_ros_TARGETS: ${tf2_ros_TARGETS}")
else()
  message(WARNING "tf2_ros_FOUND: FALSE")
endif()
message(WARNING "=== END DEBUG ===")

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/predicted_path_utils.cpp
  src/conversion.cpp
)

# Debug: Check what was created
message(WARNING "LIBRARIES after ament_auto_add_library: ${${PROJECT_NAME}_LIBRARIES}")
if(TARGET ${PROJECT_NAME})
  message(WARNING "Target ${PROJECT_NAME} EXISTS after ament_auto_add_library")
  get_target_property(lib_type ${PROJECT_NAME} TYPE)
  message(WARNING "  TYPE: ${lib_type}")
else()
  message(WARNING "Target ${PROJECT_NAME} DOES NOT EXIST after ament_auto_add_library - THIS IS A PROBLEM!")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  file(GLOB_RECURSE test_files test/**/*.cpp)

  ament_add_ros_isolated_gtest(test_object_recognition_utils ${test_files})

  target_link_libraries(test_object_recognition_utils
    ${PROJECT_NAME}
  )
endif()

# Debug: Check what will be exported
message(WARNING "Before autoware_ament_auto_package():")
message(WARNING "  LIBRARIES to export: ${${PROJECT_NAME}_LIBRARIES}")
message(WARNING "  EXECUTABLES to export: ${${PROJECT_NAME}_EXECUTABLES}")

# Try explicit target export
if(TARGET ${PROJECT_NAME})
  message(WARNING "Attempting explicit ament_export_targets for ${PROJECT_NAME}")
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  )
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_dependencies(tf2_ros)
endif()

autoware_ament_auto_package()
