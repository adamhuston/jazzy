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
#include "rov2_sensors/adxl345.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <utility>

namespace rov2_sensors
{
namespace
{
class LinuxI2c : public RegisterIo
{
public:
  LinuxI2c(std::string device, int address)
  : device_(std::move(device)), address_(address) {}
  ~LinuxI2c() override {close();}
  void open() override
  {
    fd_ = ::open(device_.c_str(), O_RDWR | O_CLOEXEC);
    if (fd_ < 0) {fail("open " + device_);}
    if (::ioctl(fd_, I2C_SLAVE, address_) < 0) {
      const std::string reason = std::strerror(errno);
      close();
      throw std::runtime_error("I2C_SLAVE: " + reason);
    }
  }
  void close() override
  {
    if (fd_ >= 0) {::close(fd_);}
    fd_ = -1;
  }
  std::vector<uint8_t> read(uint8_t reg, size_t count) override
  {
    std::vector<uint8_t> bytes(count);
    i2c_msg messages[2] {};
    messages[0].addr = static_cast<uint16_t>(address_);
    messages[0].len = 1;
    messages[0].buf = &reg;
    messages[1].addr = static_cast<uint16_t>(address_);
    messages[1].flags = I2C_M_RD;
    messages[1].len = static_cast<uint16_t>(count);
    messages[1].buf = bytes.data();
    i2c_rdwr_ioctl_data transaction {messages, 2};
    const int result = ::ioctl(fd_, I2C_RDWR, &transaction);
    if (result < 0) {fail("I2C read");}
    if (result != 2) {throw std::runtime_error("I2C read: incomplete transaction");}
    return bytes;
  }
  void write(uint8_t reg, uint8_t value) override
  {
    uint8_t bytes[2] {reg, value};
    i2c_msg message {};
    message.addr = static_cast<uint16_t>(address_);
    message.len = 2;
    message.buf = bytes;
    i2c_rdwr_ioctl_data transaction {&message, 1};
    const int result = ::ioctl(fd_, I2C_RDWR, &transaction);
    if (result < 0) {fail("I2C write");}
    if (result != 1) {throw std::runtime_error("I2C write: incomplete transaction");}
  }

private:
  void fail(const std::string & operation)
  {
    throw std::runtime_error(operation + ": " + std::strerror(errno));
  }
  std::string device_;
  int address_;
  int fd_ {-1};
};
}  // namespace

std::unique_ptr<RegisterIo> linux_i2c(const std::string & device, int address)
{
  if (device.empty() || (address != 0x53 && address != 0x1d)) {
    throw std::runtime_error("ADXL345 needs a device path and address 0x53 or 0x1d");
  }
  return std::make_unique<LinuxI2c>(device, address);
}

std::array<double, 3> decode_acceleration(const std::vector<uint8_t> & bytes)
{
  if (bytes.size() != 6) {throw std::runtime_error("ADXL345 requires six acceleration bytes");}
  std::array<double, 3> acceleration {};
  for (size_t axis = 0; axis < 3; ++axis) {
    const int raw = bytes[axis * 2] | (static_cast<int>(bytes[axis * 2 + 1]) << 8);
    const int signed_raw = raw >= 32768 ? raw - 65536 : raw;
    acceleration[axis] = signed_raw * 0.0039 * 9.80665;
  }
  return acceleration;
}

AdxlSource::AdxlSource(std::unique_ptr<RegisterIo> io, AdxlConfig config)
: io_(std::move(io)), config_(std::move(config))
{
  if (config_.frame.empty() || !std::isfinite(config_.variance) || config_.variance <= 0.0 ||
    (config_.range_g != 2 && config_.range_g != 4 &&
    config_.range_g != 8 && config_.range_g != 16) ||
    (config_.rate_hz != 50 && config_.rate_hz != 100 && config_.rate_hz != 200))
  {
    throw std::runtime_error("invalid ADXL345 frame, variance, range or rate (50/100/200 Hz)");
  }
  for (size_t axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(config_.bias[axis]) ||
      !std::isfinite(config_.scale[axis]) || config_.scale[axis] <= 0.0)
    {
      throw std::runtime_error("ADXL345 bias must be finite and scale positive finite");
    }
  }
}

void AdxlSource::connect()
{
  io_->open();
  open_ = true;
  if (register_value(0x00) != 0xe5) {throw std::runtime_error("ADXL345 identity mismatch");}
  identified_ = true;
  io_->write(0x2d, 0x00);
  int range_bits = 0;
  for (int range = 2; range < config_.range_g; range *= 2) {
    ++range_bits;
  }
  io_->write(0x31, static_cast<uint8_t>(0x08 | range_bits));  // full-resolution, right justified
  const uint8_t rate_bits = config_.rate_hz == 50 ? 0x09 :
    (config_.rate_hz == 100 ? 0x0a : 0x0b);
  io_->write(0x2c, rate_bits);
  io_->write(0x38, 0x00);  // flush old FIFO
  io_->write(0x38, 0x80);  // stream FIFO: retain short transients between publications
  io_->write(0x2e, 0x01);  // enable overrun indication (no GPIO interrupt required)
  io_->write(0x2d, 0x08);
  previous_ = {};
}

uint8_t AdxlSource::register_value(uint8_t reg)
{
  const auto bytes = io_->read(reg, 1);
  if (bytes.size() != 1) {throw std::runtime_error("ADXL345 incomplete register read");}
  return bytes.front();
}

void AdxlSource::disconnect()
{
  if (!open_) {return;}
  open_ = false;
  if (!identified_) {io_->close(); return;}
  identified_ = false;
  try {
    io_->write(0x2d, 0x00);
  } catch (const std::runtime_error &) {
    io_->close();
    throw;
  }
  io_->close();
}

std::vector<Observation<sensor_msgs::msg::Imu>> AdxlSource::read(const rclcpp::Clock & clock)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  const uint8_t interrupts = register_value(0x30);
  const size_t count = register_value(0x39) & 0x3f;
  if ((interrupts & 0x01) || count >= 32) {
    throw std::runtime_error("ADXL345 FIFO overrun; lost observations");
  }
  std::vector<Observation<sensor_msgs::msg::Imu>> observations;
  const auto now = Steady::now();
  const auto ros_now = clock.now();
  for (size_t index = 0; index < count; ++index) {
    const auto acceleration = decode_acceleration(io_->read(0x32, 6));
    const double age_s = static_cast<double>(count - index) / config_.rate_hz;
    auto acquired = now - std::chrono::duration_cast<Steady::duration>(
      std::chrono::duration<double>(age_s));
    // FIFO timestamps are estimated at the configured ODR, not hardware timestamps.
    if (previous_ != Steady::time_point{} && acquired <= previous_) {
      acquired = previous_ + std::chrono::nanoseconds(1);
    }
    previous_ = acquired;
    sensor_msgs::msg::Imu message;
    message.header.stamp = ros_now - rclcpp::Duration::from_seconds(
      std::chrono::duration<double>(now - acquired).count());
    message.header.frame_id = config_.frame;
    std::array<double, 3> calibrated {};
    for (size_t axis = 0; axis < 3; ++axis) {
      const double rail = (256 * config_.range_g - 1) * 0.0039 * 9.80665;
      if (std::abs(acceleration[axis]) >= rail) {
        throw std::runtime_error("ADXL345 saturation; observation rejected");
      }
      calibrated[axis] = (acceleration[axis] - config_.bias[axis]) * config_.scale[axis];
      message.linear_acceleration_covariance[axis * 4] = config_.variance;
    }
    message.linear_acceleration.x = calibrated[0];
    message.linear_acceleration.y = calibrated[1];
    message.linear_acceleration.z = calibrated[2];
    message.orientation.w = 1.0;
    message.orientation_covariance[0] = -1.0;
    message.angular_velocity_covariance[0] = -1.0;
    observations.push_back({message, acquired});
    std::this_thread::sleep_for(std::chrono::microseconds(10));  // FIFO pop settle time
  }
  return observations;
}
}  // namespace rov2_sensors
