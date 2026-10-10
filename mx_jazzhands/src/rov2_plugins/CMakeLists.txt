cmake_minimum_required(VERSION 3.8)
project(rov2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rov2_core REQUIRED)
find_package(rov2_interfaces REQUIRED)

set(dependencies
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  nav_msgs
  rov2_core
  rov2_interfaces
)

add_library(${PROJECT_NAME} SHARED
  src/mock_sensor.cpp
  src/mock_actuator.cpp
  src/noop_brain.cpp
  src/yahboom_board.cpp
  src/drivetrain_plugin.cpp
  src/nav_imu_plugin.cpp
  src/battery_plugin.cpp
  src/wheel_encoders_plugin.cpp
  src/odometry_plugin.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
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

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Pure math: wrap-safe encoder deltas, odometry integration (no ROS, no serial).
  ament_add_gtest(test_wheel_odometry test/test_wheel_odometry.cpp)
  target_include_directories(test_wheel_odometry PRIVATE include)
  target_compile_features(test_wheel_odometry PRIVATE cxx_std_17)

  # Driver transport/telemetry via a pseudo-terminal "fake board" (POSIX only;
  # compiles yahboom_board.cpp directly to avoid the pluginlib-loaded library).
  ament_add_gtest(test_yahboom_board test/test_yahboom_board.cpp src/yahboom_board.cpp)
  target_include_directories(test_yahboom_board PRIVATE include)
  target_compile_features(test_yahboom_board PRIVATE cxx_std_17)
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
