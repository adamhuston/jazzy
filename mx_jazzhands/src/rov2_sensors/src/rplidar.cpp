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
#include "rov2_sensors/rplidar.hpp"

#include <algorithm>
#include <ctime>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "sl_lidar.h"  // NOLINT(build/include_subdir) - SDK exports flat headers.
#include "sl_lidar_driver.h"  // NOLINT(build/include_subdir)

namespace rov2_sensors
{
namespace
{
constexpr double pi = 3.14159265358979323846;
constexpr double circle = 2.0 * pi;

void checked(sl_result result, const std::string & operation)
{
  if (SL_IS_FAIL(result)) {
    throw std::runtime_error("RPLIDAR " + operation + " failed (" + std::to_string(result) + ")");
  }
}

class SlamtecSource : public Source<sensor_msgs::msg::LaserScan>
{
public:
  explicit SlamtecSource(LidarConfig config)
  : config_(std::move(config))
  {
    if (config_.device.empty() || config_.frame.empty() || config_.baud <= 0 ||
      !std::isfinite(config_.range_min) || !std::isfinite(config_.range_max) ||
      config_.range_min <= 0.0 || config_.range_max <= config_.range_min)
    {
      throw std::runtime_error("invalid RPLIDAR device, baud, frame or range limits");
    }
  }
  void connect() override
  {
    const auto channel = sl::createSerialPortChannel(config_.device, config_.baud);
    checked(channel.err, "create serial channel");
    channel_.reset(channel.value);
    const auto driver = sl::createLidarDriver();
    checked(driver.err, "create driver");
    driver_.reset(driver.value);
    checked(driver_->connect(channel_.get()), "connect");
    sl_lidar_response_device_info_t info {};
    checked(driver_->getDeviceInfo(info, 200), "identify");
    sl_lidar_response_device_health_t health {};
    checked(driver_->getHealth(health, 200), "health");
    if (health.status != 0) {
      throw std::runtime_error("RPLIDAR device reports non-healthy status " +
              std::to_string(health.status));
    }
    motor_started_ = true;
    checked(driver_->setMotorSpeed(), "start motor");
    checked(driver_->startScan(false, false), "start standard scan");
    previous_start_ = 0;
  }
  void disconnect() override
  {
    if (!driver_) {channel_.reset(); return;}
    const auto stop_result = motor_started_ ? driver_->stop(200) : SL_RESULT_OK;
    const auto motor_result = motor_started_ ? driver_->setMotorSpeed(0) : SL_RESULT_OK;
    driver_->disconnect();
    driver_.reset();
    channel_.reset();
    motor_started_ = false;
    checked(stop_result, "stop scan");
    checked(motor_result, "stop motor");
  }
  std::vector<Observation<sensor_msgs::msg::LaserScan>> read(
    const rclcpp::Clock & clock) override
  {
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = 8192;
    sl_u64 start_us = 0;
    const auto result = driver_->grabScanDataHqWithTimeStamp(nodes, count, start_us, 100);
    if (result == SL_RESULT_OPERATION_TIMEOUT) {return {};}
    checked(result, "read scan");
    if (start_us == 0 || count < 2) {throw std::runtime_error("RPLIDAR missing scan timing/data");}
    // The pinned SDK's Linux rp_getus() uses CLOCK_MONOTONIC. Compare in that
    // domain, then map the measured age to ROS/steady clocks (including queuing).
    timespec host_time {};
    if (::clock_gettime(CLOCK_MONOTONIC, &host_time) != 0) {
      throw std::runtime_error("RPLIDAR monotonic clock read failed");
    }
    const uint64_t host_us = static_cast<uint64_t>(host_time.tv_sec) * 1000000 +
      static_cast<uint64_t>(host_time.tv_nsec) / 1000;
    if (start_us > host_us || (previous_start_ != 0 && start_us <= previous_start_)) {
      throw std::runtime_error("RPLIDAR invalid/non-increasing acquisition timestamp");
    }
    const auto steady_now = Steady::now();
    const auto ros_now = clock.now();
    const double duration = previous_start_ == 0 ? 0.0 :
      static_cast<double>(start_us - previous_start_) / 1e6;
    previous_start_ = start_us;
    if (duration == 0.0) {return {};}  // first revolution establishes timing
    if (duration > 1.0) {throw std::runtime_error("RPLIDAR scan duration exceeds one second");}
    const double age_s = static_cast<double>(host_us - start_us) / 1e6;
    const auto acquired = steady_now - std::chrono::duration_cast<Steady::duration>(
      std::chrono::duration<double>(age_s));
    std::vector<LidarPoint> points;
    points.reserve(count);
    for (size_t index = 0; index < count; ++index) {
      points.push_back({
            nodes[index].angle_z_q14 * pi / 32768.0,
            nodes[index].dist_mm_q2 / 4000.0,
            nodes[index].quality});
    }
    auto scan = make_scan(points, duration, ros_now - rclcpp::Duration::from_seconds(age_s),
          config_);
    return {{std::move(scan), acquired}};
  }

private:
  LidarConfig config_;
  std::unique_ptr<sl::ILidarDriver> driver_;
  std::unique_ptr<sl::IChannel> channel_;
  sl_u64 previous_start_ {0};
  bool motor_started_ {false};
};
}  // namespace

sensor_msgs::msg::LaserScan make_scan(
  const std::vector<LidarPoint> & points, double duration_s,
  const rclcpp::Time & first_ray, const LidarConfig & config)
{
  if (points.size() < 2 || !std::isfinite(duration_s) || duration_s <= 0.0 ||
    duration_s > 1.0 || !std::isfinite(points.front().angle_rad) ||
    points.front().angle_rad < 0.0 || points.front().angle_rad >= circle)
  {
    throw std::runtime_error("RPLIDAR invalid scan size or duration");
  }
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = first_ray;
  scan.header.frame_id = config.frame;
  // Keep acquisition order: Slamtec angles are clockwise, ROS angles CCW.
  // Negative increment preserves first-ray timestamps without reversing time.
  scan.angle_min = static_cast<float>(pi - points.front().angle_rad);
  scan.angle_increment = static_cast<float>(-circle / points.size());
  scan.angle_max = scan.angle_min + scan.angle_increment * (points.size() - 1);
  scan.scan_time = static_cast<float>(duration_s);
  scan.time_increment = static_cast<float>(duration_s / points.size());
  scan.range_min = static_cast<float>(config.range_min);
  scan.range_max = static_cast<float>(config.range_max);
  scan.ranges.assign(points.size(), std::numeric_limits<float>::quiet_NaN());
  scan.intensities.assign(points.size(), 0.0f);
  size_t valid = 0;
  for (const auto & point : points) {
    if (!std::isfinite(point.angle_rad) || point.angle_rad < 0.0 || point.angle_rad >= circle ||
      !std::isfinite(point.range_m) || point.quality == 0 ||
      point.range_m < config.range_min || point.range_m > config.range_max)
    {
      continue;  // unknown/invalid is NaN, never a fabricated clear (+Inf) ray
    }
    double angle = std::fmod(point.angle_rad - points.front().angle_rad + circle, circle);
    const size_t bin = static_cast<size_t>(std::llround(angle / circle * points.size())) %
      points.size();
    if (!std::isfinite(scan.ranges[bin]) || point.range_m < scan.ranges[bin]) {
      scan.ranges[bin] = static_cast<float>(point.range_m);
      scan.intensities[bin] = point.quality;
    }
    ++valid;
  }
  if (valid == 0) {throw std::runtime_error("RPLIDAR scan has no usable returns");}
  return scan;
}

std::unique_ptr<Source<sensor_msgs::msg::LaserScan>> slamtec_source(LidarConfig config)
{
  return std::make_unique<SlamtecSource>(std::move(config));
}
}  // namespace rov2_sensors
