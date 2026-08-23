cmake_minimum_required(VERSION 3.8)
project(rov2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rov2_core REQUIRED)
find_package(rov2_interfaces REQUIRED)

set(dependencies
  pluginlib
  rclcpp
  geometry_msgs
  rov2_core
  rov2_interfaces
)

add_library(${PROJECT_NAME} SHARED
  src/mock_sensor.cpp
  src/mock_actuator.cpp
  src/noop_brain.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Registers plugins.xml against the base package (rov2_core) in the ament index.
pluginlib_export_plugin_description_file(rov2_core plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
