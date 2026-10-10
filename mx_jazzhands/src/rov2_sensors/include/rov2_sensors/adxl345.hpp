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
#ifndef ROV2_SENSORS__ADXL345_HPP_
#define ROV2_SENSORS__ADXL345_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "sensor_msgs/msg/imu.hpp"
#include "rov2_sensors/source.hpp"

namespace rov2_sensors
{
class RegisterIo
{
public:
  virtual ~RegisterIo() = default;
  virtual void open() = 0;
  virtual void close() = 0;
  virtual std::vector<uint8_t> read(uint8_t reg, size_t count) = 0;
  virtual void write(uint8_t reg, uint8_t value) = 0;
};

std::unique_ptr<RegisterIo> linux_i2c(const std::string & device, int address);

struct AdxlConfig
{
  std::string frame {"accel_link"};
  int range_g {16};
  int rate_hz {100};
  std::array<double, 3> bias {{0.0, 0.0, 0.0}};
  std::array<double, 3> scale {{1.0, 1.0, 1.0}};
  double variance {0.01};
};

std::array<double, 3> decode_acceleration(const std::vector<uint8_t> & bytes);

class AdxlSource : public Source<sensor_msgs::msg::Imu>
{
public:
  AdxlSource(std::unique_ptr<RegisterIo> io, AdxlConfig config);
  void connect() override;
  void disconnect() override;
  std::vector<Observation<sensor_msgs::msg::Imu>> read(const rclcpp::Clock & clock) override;

private:
  uint8_t register_value(uint8_t reg);
  std::unique_ptr<RegisterIo> io_;
  AdxlConfig config_;
  bool open_ {false};
  bool identified_ {false};
  Steady::time_point previous_ {};
};
}  // namespace rov2_sensors
#endif  // ROV2_SENSORS__ADXL345_HPP_
