cmake_minimum_required(VERSION 3.8)
project(rov2_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rov2_interfaces REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  lifecycle_msgs
  pluginlib
  diagnostic_msgs
  geometry_msgs
  rov2_interfaces
)

add_library(core_node SHARED src/core_node.cpp)
target_include_directories(core_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(core_node PUBLIC cxx_std_17)
ament_target_dependencies(core_node ${dependencies})
rclcpp_components_register_nodes(core_node "rov2_core::CoreNode")

add_executable(rov2_core_node src/main.cpp)
target_link_libraries(rov2_core_node core_node)
ament_target_dependencies(rov2_core_node ${dependencies})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS core_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS rov2_core_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(core_node)
ament_export_dependencies(${dependencies})
ament_package()
