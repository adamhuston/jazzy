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
#include <gtest/gtest.h>

#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>
#include <atomic>
#include <map>
#include <thread>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rov2_sensors/adxl345.hpp"
#include "rov2_sensors/managed_sensor.hpp"
#include "rov2_sensors/rplidar.hpp"

namespace rov2_sensors
{
class FakeRegisters : public RegisterIo
{
public:
  bool opened {false};
  bool fail {false};
  uint8_t identity {0xe5};
  uint8_t fifo {0};
  uint8_t interrupts {0};
  std::map<uint8_t, uint8_t> written;
  std::vector<uint8_t> acceleration {0, 0, 0, 0, 0, 1};
  void open() override {opened = true;}
  void close() override {opened = false;}
  void write(uint8_t reg, uint8_t value) override {written[reg] = value;}
  std::vector<uint8_t> read(uint8_t reg, size_t) override
  {
    if (fail) {throw std::runtime_error("injected I2C read failure");}
    if (reg == 0) {return {identity};}
    if (reg == 0x39) {return {fifo};}
    if (reg == 0x30) {return {interrupts};}
    return acceleration;
  }
};

TEST(Freshness, ExactBoundaryAndFutureTime)
{
  const auto now = Steady::now();
  EXPECT_TRUE(fresh(now - std::chrono::milliseconds(100), now, 100.0));
  EXPECT_FALSE(fresh(now - std::chrono::microseconds(100001), now, 100.0));
  EXPECT_FALSE(fresh(now + std::chrono::nanoseconds(1), now, 100.0));
}

TEST(Adxl, SignedScalingAndMalformedBytes)
{
  const auto acceleration = decode_acceleration({0, 1, 0, 255, 0, 0});
  EXPECT_NEAR(acceleration[0], 256 * 0.0039 * 9.80665, 1e-9);
  EXPECT_NEAR(acceleration[1], -256 * 0.0039 * 9.80665, 1e-9);
  EXPECT_DOUBLE_EQ(acceleration[2], 0);
  EXPECT_THROW(decode_acceleration({0}), std::runtime_error);
}

TEST(Adxl, IdentityConfigurationAndStandby)
{
  auto io = std::make_unique<FakeRegisters>();
  auto registers = io.get();
  AdxlSource source(std::move(io), {});
  source.connect();
  EXPECT_EQ(registers->written[0x31], 0x0b);
  EXPECT_EQ(registers->written[0x2c], 0x0a);
  EXPECT_EQ(registers->written[0x38], 0x80);
  EXPECT_EQ(registers->written[0x2d], 0x08);
  source.disconnect();
  EXPECT_FALSE(registers->opened);
  EXPECT_EQ(registers->written[0x2d], 0);
  registers->identity = 0;
  EXPECT_THROW(source.connect(), std::runtime_error);
  source.disconnect();
}

TEST(Adxl, FifoPreservesTransientAndFlagsUnavailableAxes)
{
  auto io = std::make_unique<FakeRegisters>();
  auto registers = io.get();
  AdxlSource source(std::move(io), {});
  source.connect();
  rclcpp::Clock clock;
  EXPECT_TRUE(source.read(clock).empty());
  registers->fifo = 3;
  const auto observations = source.read(clock);
  ASSERT_EQ(observations.size(), 3u);
  EXPECT_LT(observations[0].acquired, observations[1].acquired);
  EXPECT_LT(observations[1].acquired, observations[2].acquired);
  EXPECT_LT(observations[0].message.header.stamp.sec * 1e9 +
    observations[0].message.header.stamp.nanosec,
    observations[2].message.header.stamp.sec * 1e9 +
    observations[2].message.header.stamp.nanosec);
  EXPECT_DOUBLE_EQ(observations[0].message.orientation_covariance[0], -1);
  EXPECT_DOUBLE_EQ(observations[0].message.angular_velocity_covariance[0], -1);
  EXPECT_GT(observations[0].message.linear_acceleration_covariance[0], 0);
  registers->fifo = 32;
  EXPECT_THROW(source.read(clock), std::runtime_error);
  registers->fifo = 1;
  registers->interrupts = 1;
  EXPECT_THROW(source.read(clock), std::runtime_error);
  registers->interrupts = 0;
  registers->fail = true;
  EXPECT_THROW(source.read(clock), std::runtime_error);
  source.disconnect();
}

TEST(Adxl, InvalidConfiguration)
{
  AdxlConfig config;
  config.rate_hz = 123;
  EXPECT_THROW(AdxlSource(std::make_unique<FakeRegisters>(), config), std::runtime_error);
  EXPECT_THROW(linux_i2c("/dev/i2c-1", 0), std::runtime_error);
}

TEST(Adxl, SaturationRejected)
{
  auto io = std::make_unique<FakeRegisters>();
  auto registers = io.get();
  AdxlSource source(std::move(io), {});
  source.connect();
  registers->fifo = 1;
  registers->acceleration = {255, 15, 0, 0, 0, 0};  // full-resolution +16 g rail
  rclcpp::Clock clock;
  EXPECT_THROW(source.read(clock), std::runtime_error);
  source.disconnect();
}

TEST(Lidar, GeometryTimingAndUnknownReturns)
{
  LidarConfig config;
  const std::vector<LidarPoint> points {{0, 1, 10}, {1.57079632679, 2, 10},
    {3.14159265359, 0, 0}, {4.71238898038, 20, 10}};
  const auto scan = make_scan(points, 0.2, rclcpp::Time(1000000000LL), config);
  ASSERT_EQ(scan.ranges.size(), 4u);
  EXPECT_EQ(scan.header.stamp.sec, 1);
  EXPECT_LT(scan.angle_increment, 0);
  EXPECT_NEAR(scan.angle_min, 3.14159265359, 1e-6);
  EXPECT_NEAR(scan.scan_time, 0.2, 1e-6);
  EXPECT_NEAR(scan.time_increment, 0.05, 1e-6);
  EXPECT_FLOAT_EQ(scan.ranges[0], 1);
  EXPECT_FLOAT_EQ(scan.ranges[1], 2);
  EXPECT_TRUE(std::isnan(scan.ranges[2]));
  EXPECT_TRUE(std::isnan(scan.ranges[3]));
  EXPECT_THROW(make_scan({{0, 0, 0}, {1, 0, 0}}, 0.2, rclcpp::Time(0), config),
    std::runtime_error);
  EXPECT_THROW(make_scan(points, -1, rclcpp::Time(0), config), std::runtime_error);
}

TEST(Lidar, SdkRejectsUnhealthyFakeSerialDeviceBeforeScanning)
{
  const int master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
  ASSERT_GE(master, 0);
  ASSERT_EQ(::grantpt(master), 0);
  ASSERT_EQ(::unlockpt(master), 0);
  LidarConfig config;
  config.device = ::ptsname(master);
  std::atomic<bool> running {true};
  std::atomic<int> health_requests {0};
  std::atomic<int> scan_requests {0};
  std::thread emulator([&]() {
      std::vector<uint8_t> incoming;
      while (running) {
        pollfd descriptor {master, POLLIN, 0};
        if (::poll(&descriptor, 1, 10) <= 0) {continue;}
        uint8_t buffer[64];
        const auto received = ::read(master, buffer, sizeof(buffer));
        if (received <= 0) {continue;}
        incoming.insert(incoming.end(), buffer, buffer + received);
        while (incoming.size() >= 2) {
          if (incoming.front() != 0xa5) {
            incoming.erase(incoming.begin());
            continue;
          }
          const auto command = incoming[1];
          size_t length = 2;
          if (command & 0x80) {
            if (incoming.size() < 3) {break;}
            length = 4 + incoming[2];
            if (incoming.size() < length) {break;}
          }
          incoming.erase(incoming.begin(), incoming.begin() + length);
          std::vector<uint8_t> payload;
          uint8_t type = 0;
          if (command == 0x50) {
            payload.assign(20, 0);
            payload[0] = 1;
            payload[2] = 1;  // legacy 1.0 firmware; no motor-control negotiation
            type = 4;
          } else if (command == 0x52) {
            payload = {2, 0, 0};  // device health ERROR
            type = 6;
            ++health_requests;
          } else if (command == 0x20 || command == 0x21) {
            ++scan_requests;
          }
          if (!payload.empty()) {
            std::vector<uint8_t> response {0xa5, 0x5a,
              static_cast<uint8_t>(payload.size()), 0, 0, 0, type};
            response.insert(response.end(), payload.begin(), payload.end());
            ::write(master, response.data(), response.size());
          }
        }
      }
    });
  auto source = slamtec_source(config);
  EXPECT_THROW(source->connect(), std::runtime_error);
  EXPECT_EQ(health_requests, 1);
  EXPECT_EQ(scan_requests, 0);
  EXPECT_NO_THROW(source->disconnect());
  running = false;
  emulator.join();
  ::close(master);
}

class FakeSource : public Source<sensor_msgs::msg::Imu>
{
public:
  std::atomic<bool> emit {true};
  std::atomic<bool> fail {false};
  std::atomic<int> connects {0};
  std::atomic<int> disconnects {0};
  void connect() override {++connects;}
  void disconnect() override {++disconnects;}
  std::vector<Observation<sensor_msgs::msg::Imu>> read(const rclcpp::Clock & clock) override
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (fail) {throw std::runtime_error("injected transport failure");}
    if (!emit) {return {};}
    sensor_msgs::msg::Imu message;
    message.header.stamp = clock.now();
    return {{message, Steady::now()}};
  }
};

class FakePlugin : public ManagedSensor<sensor_msgs::msg::Imu>
{
public:
  FakePlugin()
  : ManagedSensor("test_sensor") {}
  bool on_configure() override
  {
    auto source = std::make_unique<FakeSource>();
    fake = source.get();
    return configure_source(std::move(source), "/test/accel", "fake", 50.0);
  }
  FakeSource * fake {};
};

TEST(Runtime, FreshStaleFaultRecoveryAndLifecycle)
{
  int argc = 0;
  rclcpp::init(argc, nullptr);
  auto host = std::make_shared<rclcpp_lifecycle::LifecycleNode>("sensor_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(host->get_node_base_interface());
  auto drain = [&]() {
      const auto deadline = Steady::now() + std::chrono::milliseconds(50);
      while (Steady::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    };
  size_t received = 0;
  std::vector<int64_t> stamps;
  std::string health_fresh;
  auto data_sub = host->create_subscription<sensor_msgs::msg::Imu>(
    "/test/accel", rclcpp::SensorDataQoS(),
    [&](sensor_msgs::msg::Imu::ConstSharedPtr message) {
      ++received;
      stamps.push_back(rclcpp::Time(message->header.stamp).nanoseconds());
    });
  auto health_sub = host->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/sensors/test_sensor/health", rclcpp::QoS(10).reliable(),
    [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
      for (const auto & field : message->status.front().values) {
        if (field.key == "fresh") {health_fresh = field.value;}
      }
    });
  FakePlugin plugin;
  ASSERT_TRUE(plugin.on_init(host, "fake"));
  ASSERT_TRUE(plugin.on_configure());
  ASSERT_TRUE(plugin.on_activate());
  const auto until = Steady::now() + std::chrono::milliseconds(150);
  while (Steady::now() < until) {
    drain();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  plugin.poll();
  EXPECT_EQ(plugin.state(), rov2_interfaces::msg::PluginStatus::ACTIVE);
  ASSERT_GT(received, 0u);
  EXPECT_TRUE(std::is_sorted(stamps.begin(), stamps.end()));
  EXPECT_EQ(std::adjacent_find(stamps.begin(), stamps.end()), stamps.end());
  EXPECT_EQ(health_fresh, "true");
  plugin.fake->emit = false;
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  plugin.poll();
  drain();
  EXPECT_EQ(plugin.state(), rov2_interfaces::msg::PluginStatus::DEGRADED);
  const size_t stale_received = received;
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  plugin.poll();
  drain();
  EXPECT_EQ(received, stale_received);
  EXPECT_EQ(health_fresh, "false");
  plugin.fake->fail = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  plugin.poll();
  EXPECT_EQ(plugin.state(), rov2_interfaces::msg::PluginStatus::FAULT);
  plugin.fake->fail = false;
  plugin.fake->emit = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  plugin.poll();
  EXPECT_EQ(plugin.state(), rov2_interfaces::msg::PluginStatus::ACTIVE);
  EXPECT_GT(plugin.fake->connects, 1);
  ASSERT_TRUE(plugin.on_deactivate());
  EXPECT_EQ(plugin.state(), rov2_interfaces::msg::PluginStatus::INACTIVE);
  drain();
  const size_t inactive_received = received;
  const int stopped = plugin.fake->disconnects;
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_EQ(plugin.fake->disconnects, stopped);
  plugin.poll();
  executor.spin_some();
  EXPECT_EQ(received, inactive_received);
  plugin.fake->emit = false;
  ASSERT_TRUE(plugin.on_activate());
  plugin.poll();
  EXPECT_NE(plugin.state(), rov2_interfaces::msg::PluginStatus::ACTIVE);
  ASSERT_TRUE(plugin.on_deactivate());
  ASSERT_TRUE(plugin.on_cleanup());
  host.reset();
  rclcpp::shutdown();
}
}  // namespace rov2_sensors
