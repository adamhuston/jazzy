// Copyright 2026 ROV2 Maintainers
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef ROV2_SENSORS__RPLIDAR_HPP_
#define ROV2_SENSORS__RPLIDAR_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "rov2_sensors/source.hpp"

namespace rov2_sensors
{
struct LidarPoint
{
  double angle_rad;
  double range_m;
  uint8_t quality;
};

struct LidarConfig
{
  std::string device {"/dev/ttyUSB0"};
  int baud {115200};
  std::string frame {"laser"};
  double range_min {0.15};
  double range_max {12.0};
};

sensor_msgs::msg::LaserScan make_scan(
  const std::vector<LidarPoint> & points, double duration_s,
  const rclcpp::Time & first_ray, const LidarConfig & config);
std::unique_ptr<Source<sensor_msgs::msg::LaserScan>> slamtec_source(LidarConfig config);
}  // namespace rov2_sensors
#endif  // ROV2_SENSORS__RPLIDAR_HPP_
