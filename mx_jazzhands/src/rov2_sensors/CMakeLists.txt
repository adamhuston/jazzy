cmake_minimum_required(VERSION 3.16)
project(rov2_sensors)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rov2_core REQUIRED)
find_package(Threads REQUIRED)
include(FetchContent)

# Upstream SDK is BSD-2-Clause. Pin the source; never build against floating HEAD.
fetchcontent_declare(slamtec
  GIT_REPOSITORY https://github.com/Slamtec/rplidar_sdk.git
  GIT_TAG 99478e5fb90de3b4a6db0080acacd373f8b36869
)
fetchcontent_makeavailable(slamtec)
set(SDK ${slamtec_SOURCE_DIR}/sdk)
file(GLOB_RECURSE unpackers CONFIGURE_DEPENDS "${SDK}/src/dataunpacker/*.cpp")
add_library(slamtec_sdk STATIC
  ${SDK}/src/sl_lidar_driver.cpp
  ${SDK}/src/hal/thread.cpp
  ${SDK}/src/sl_crc.cpp
  ${SDK}/src/sl_serial_channel.cpp
  ${SDK}/src/sl_lidarprotocol_codec.cpp
  ${SDK}/src/sl_async_transceiver.cpp
  ${SDK}/src/sl_tcp_channel.cpp
  ${SDK}/src/sl_udp_channel.cpp
  ${SDK}/src/arch/linux/net_serial.cpp
  ${SDK}/src/arch/linux/net_socket.cpp
  ${SDK}/src/arch/linux/timer.cpp
  ${unpackers}
)
set_target_properties(slamtec_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(slamtec_sdk SYSTEM PUBLIC ${SDK}/include PRIVATE
  ${SDK}/src ${SDK}/src/dataunpacker ${SDK}/src/dataunpacker/unpacker)
target_link_libraries(slamtec_sdk PUBLIC Threads::Threads)

add_library(${PROJECT_NAME} SHARED src/adxl345.cpp src/rplidar.cpp src/plugins.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_lifecycle pluginlib sensor_msgs diagnostic_msgs rov2_core)
target_link_libraries(${PROJECT_NAME} slamtec_sdk Threads::Threads)
pluginlib_export_plugin_description_file(rov2_core plugins.xml)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})
install(FILES ${slamtec_SOURCE_DIR}/LICENSE
  DESTINATION share/${PROJECT_NAME} RENAME SLAMTEC-LICENSE)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sensors test/test_sensors.cpp)
  target_link_libraries(test_sensors ${PROJECT_NAME})
  ament_target_dependencies(test_sensors rclcpp rclcpp_lifecycle sensor_msgs diagnostic_msgs)
  # Teach cppcheck the TEST/EXPECT macros rather than treating tests as syntax errors.
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(TESTNAME cppcheck_gtest LIBRARIES googletest INCLUDE_DIRS include test)
  set(ament_cmake_cppcheck_ADDITIONAL_EXCLUDE "test/test_sensors.cpp")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_lifecycle sensor_msgs diagnostic_msgs rov2_core)
ament_package()
