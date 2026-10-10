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

#include <limits>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"
#include "rov2_sensors/adxl345.hpp"
#include "rov2_sensors/managed_sensor.hpp"
#include "rov2_sensors/rplidar.hpp"

namespace rov2_sensors
{
class Rplidar : public ManagedSensor<sensor_msgs::msg::LaserScan>
{
public:
  Rplidar()
  : ManagedSensor("rplidar") {}
  bool on_configure() override
  {
    try {
      LidarConfig config;
      config.device = param<std::string>("device", config.device);
      const int64_t baud = param<int64_t>("baud", config.baud);
      if (baud <= 0 || baud > std::numeric_limits<int>::max()) {
        throw std::runtime_error("RPLIDAR baud out of range");
      }
      config.baud = static_cast<int>(baud);
      config.frame = param<std::string>("frame_id", config.frame);
      config.range_min = param<double>("range_min_m", config.range_min);
      config.range_max = param<double>("range_max_m", config.range_max);
      return configure_source(slamtec_source(config), "/scan", config.device, 250.0);
    } catch (const std::runtime_error & error) {
      return configuration_error(error);
    }
  }
};

class Adxl345 : public ManagedSensor<sensor_msgs::msg::Imu>
{
public:
  Adxl345()
  : ManagedSensor("adxl345") {}
  bool on_configure() override
  {
    try {
      AdxlConfig config;
      const auto device = param<std::string>("device", "/dev/i2c-1");
      const int64_t address = param<int64_t>("address", 0x53);
      const int64_t range = param<int64_t>("range_g", config.range_g);
      const int64_t rate = param<int64_t>("rate_hz", config.rate_hz);
      if ((address != 0x53 && address != 0x1d) ||
        (range != 2 && range != 4 && range != 8 && range != 16) ||
        (rate != 50 && rate != 100 && rate != 200))
      {
        throw std::runtime_error("invalid ADXL345 address/range/rate");
      }
      config.range_g = static_cast<int>(range);
      config.rate_hz = static_cast<int>(rate);
      config.frame = param<std::string>("frame_id", config.frame);
      config.variance = param<double>("acceleration_variance", config.variance);
      const auto bias = param<std::vector<double>>("bias_mps2", {0.0, 0.0, 0.0});
      const auto scale = param<std::vector<double>>("scale", {1.0, 1.0, 1.0});
      if (bias.size() != 3 || scale.size() != 3) {
        throw std::runtime_error("ADXL345 bias/scale require three entries");
      }
      std::copy(bias.begin(), bias.end(), config.bias.begin());
      std::copy(scale.begin(), scale.end(), config.scale.begin());
      return configure_source(
        std::make_unique<AdxlSource>(linux_i2c(device, static_cast<int>(address)), config),
        "/accel", device, 100.0);
    } catch (const std::runtime_error & error) {
      return configuration_error(error);
    }
  }
};
}  // namespace rov2_sensors

PLUGINLIB_EXPORT_CLASS(rov2_sensors::Rplidar, rov2_core::SensorPlugin)
PLUGINLIB_EXPORT_CLASS(rov2_sensors::Adxl345, rov2_core::SensorPlugin)
