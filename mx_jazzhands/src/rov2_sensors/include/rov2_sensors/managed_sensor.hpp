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
#ifndef ROV2_SENSORS__MANAGED_SENSOR_HPP_
#define ROV2_SENSORS__MANAGED_SENSOR_HPP_

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rov2_core/sensor_plugin.hpp"
#include "rov2_sensors/source.hpp"

namespace rov2_sensors
{

template<typename Message>
class ManagedSensor : public rov2_core::SensorPlugin
{
public:
  explicit ManagedSensor(std::string prefix)
  : prefix_(std::move(prefix)) {}
  ~ManagedSensor() override {stop_worker();}

  bool on_activate() override
  {
    if (!source_ || !pub_ || running_) {
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_.clear();
      have_sample_ = false;
      connected_ = false;
      error_.clear();
      valid_bins_ = 0;
      total_bins_ = 0;
    }
    pub_->on_activate();
    health_pub_->on_activate();
    diag_pub_->on_activate();
    state_ = PluginStatus::DEGRADED;
    status_message_ = "waiting for identified device and fresh observations";
    running_ = true;
    worker_ = std::thread([this]() {acquire();});
    timer_ = node()->create_wall_timer(std::chrono::milliseconds(10), [this]() {poll();});
    return true;
  }

  bool on_deactivate() override
  {
    if (timer_) {timer_->cancel();}
    timer_.reset();
    stop_worker();
    pub_->on_deactivate();
    health_pub_->on_deactivate();
    diag_pub_->on_deactivate();
    state_ = PluginStatus::INACTIVE;
    status_message_ = "inactive; acquisition stopped";
    return true;
  }

  bool on_cleanup() override
  {
    if (timer_) {timer_->cancel();}
    timer_.reset();
    stop_worker();
    source_.reset();
    pub_.reset();
    health_pub_.reset();
    diag_pub_.reset();
    state_ = PluginStatus::UNCONFIGURED;
    return true;
  }

  bool on_shutdown() override {return on_cleanup();}

  void poll() override
  {
    if (!pub_ || !pub_->is_activated()) {return;}
    using Diagnostic = diagnostic_msgs::msg::DiagnosticStatus;
    const auto now = Steady::now();
    Diagnostic diagnostic;
    diagnostic.name = "rov2_sensors/" + prefix_;
    diagnostic.hardware_id = device_id_;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      while (!pending_.empty()) {
        auto observation = std::move(pending_.front());
        pending_.pop_front();
        if (connected_ && fresh(observation.acquired, now, timeout_ms_)) {
          pub_->publish(observation.message);
        } else {
          ++dropped_;
        }
      }
      const bool is_fresh = connected_ && have_sample_ &&
        fresh(last_sample_, now, timeout_ms_);
      if (!error_.empty()) {
        state_ = PluginStatus::FAULT;
        diagnostic.level = Diagnostic::ERROR;
        status_message_ = error_;
      } else if (!is_fresh) {
        state_ = PluginStatus::DEGRADED;
        diagnostic.level = Diagnostic::STALE;
        status_message_ = have_sample_ ? "STALE" : "NO_DATA";
      } else {
        state_ = PluginStatus::ACTIVE;
        diagnostic.level = Diagnostic::OK;
        status_message_ = "fresh observations";
      }
      diagnostic.message = status_message_;
      auto add = [&](const std::string & key, const std::string & value) {
          diagnostic_msgs::msg::KeyValue field;
          field.key = key;
          field.value = value;
          diagnostic.values.push_back(field);
        };
      add("connected", connected_ ? "true" : "false");
      add("identified", connected_ ? "true" : "false");
      add("streaming", connected_ && have_sample_ ? "true" : "false");
      add("fresh", is_fresh ? "true" : "false");
      add("sample_age_ms", have_sample_ ? std::to_string(age_ms(last_sample_, now)) : "-1");
      add("max_staleness_ms", std::to_string(timeout_ms_));
      add("sequence", std::to_string(sequence_));
      add("reconnect_count", std::to_string(reconnects_ > 0 ? reconnects_ - 1 : 0));
      add("dropped_samples", std::to_string(dropped_));
      add("last_error", error_);
      if constexpr (std::is_same<Message, sensor_msgs::msg::LaserScan>::value) {
        add("valid_bins", std::to_string(valid_bins_));
        add("unknown_bins", std::to_string(total_bins_ - valid_bins_));
        add("valid_fraction", total_bins_ == 0 ? "0" :
          std::to_string(static_cast<double>(valid_bins_) / total_bins_));
        add("timestamp_basis", "SDK first ray / Linux CLOCK_MONOTONIC");
      } else {
        add("timestamp_basis", "FIFO / configured output rate (estimated)");
      }
    }
    if (diagnostic.message != last_report_ && diagnostic.level != Diagnostic::OK) {
      RCLCPP_WARN(logger(), "[%s] %s", prefix_.c_str(), diagnostic.message.c_str());
    }
    last_report_ = diagnostic.message;
    if (now - last_diagnostic_ >= std::chrono::milliseconds(100)) {
      diagnostic_msgs::msg::DiagnosticArray array;
      array.header.stamp = node()->now();
      array.status.push_back(std::move(diagnostic));
      health_pub_->publish(array);
      diag_pub_->publish(array);
      last_diagnostic_ = now;
    }
  }

protected:
  template<typename T>
  T param(const std::string & key, const T & fallback)
  {
    auto host = node();
    const auto name = prefix_ + "." + key;
    if (!host->has_parameter(name)) {
      host->declare_parameter(name, rclcpp::ParameterValue(fallback));
    }
    return host->get_parameter(name).template get_value<T>();
  }

  bool configure_source(
    std::unique_ptr<Source<Message>> source, const std::string & default_topic,
    const std::string & device_id, double default_timeout)
  {
    timeout_ms_ = param<double>("max_staleness_ms", default_timeout);
    const auto topic = param<std::string>("topic", default_topic);
    const auto health_topic = param<std::string>("health_topic", "/sensors/" + prefix_ + "/health");
    if (!std::isfinite(timeout_ms_) || timeout_ms_ <= 0.0 ||
      topic.empty() || health_topic.empty())
    {
      throw std::runtime_error("positive finite freshness timeout and nonempty topics required");
    }
    device_id_ = device_id;
    source_ = std::move(source);
    pub_ = node()->template create_publisher<Message>(topic, rclcpp::SensorDataQoS());
    health_pub_ = node()->template create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      health_topic, rclcpp::QoS(10).reliable());
    diag_pub_ = node()->template create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10).reliable());
    state_ = PluginStatus::INACTIVE;
    status_message_ = "configured; no device readiness claim";
    return true;
  }

  bool configuration_error(const std::runtime_error & error)
  {
    state_ = PluginStatus::FAULT;
    status_message_ = error.what();
    RCLCPP_ERROR(logger(), "[%s] configuration failed: %s", prefix_.c_str(), error.what());
    return false;
  }

private:
  void stop_worker()
  {
    running_ = false;
    wake_.notify_all();
    if (worker_.joinable()) {worker_.join();}
  }

  void disconnect()
  {
    try {
      source_->disconnect();
    } catch (const std::runtime_error & error) {
      std::lock_guard<std::mutex> lock(mutex_);
      error_ = std::string("disconnect failed: ") + error.what();
      RCLCPP_ERROR(logger(), "[%s] %s", prefix_.c_str(), error_.c_str());
    }
    std::lock_guard<std::mutex> lock(mutex_);
    connected_ = false;
    have_sample_ = false;
    pending_.clear();
    valid_bins_ = 0;
    total_bins_ = 0;
  }

  void acquire()
  {
    bool connected = false;
    auto last_received = Steady::now();
    while (running_) {
      try {
        if (!connected) {
          source_->connect();
          connected = true;
          last_received = Steady::now();
          std::lock_guard<std::mutex> lock(mutex_);
          connected_ = true;
          ++reconnects_;
          pending_.clear();
          have_sample_ = false;
        }
        auto observations = source_->read(*clock_);
        if (observations.empty() && age_ms(last_received, Steady::now()) >
          std::max(1000.0, timeout_ms_ * 4.0))
        {
          throw std::runtime_error("acquisition stalled; reconnecting");
        }
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto & observation : observations) {
          if (pending_.size() == 64) {
            ++dropped_;
            throw std::runtime_error("publication buffer overflow; observations lost");
          }
          error_.clear();
          last_received = Steady::now();
          if constexpr (std::is_same<Message, sensor_msgs::msg::LaserScan>::value) {
            total_bins_ = observation.message.ranges.size();
            valid_bins_ = static_cast<size_t>(std::count_if(
              observation.message.ranges.begin(), observation.message.ranges.end(),
                [](float range) {return std::isfinite(range);}));
          }
          last_sample_ = observation.acquired;
          have_sample_ = true;
          ++sequence_;
          pending_.push_back(std::move(observation));
        }
      } catch (const std::runtime_error & error) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          error_ = error.what();
        }
        disconnect();
        connected = false;
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait_for(lock, std::chrono::seconds(1), [this]() {return !running_;});
      }
    }
    disconnect();
  }

  const std::string prefix_;
  std::string device_id_;
  double timeout_ms_ {0.0};
  std::unique_ptr<Source<Message>> source_;
  typename rclcpp_lifecycle::LifecyclePublisher<Message>::SharedPtr pub_;
  using HealthPublisher =
    rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>;
  HealthPublisher::SharedPtr health_pub_;
  HealthPublisher::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<bool> running_ {false};
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::deque<Observation<Message>> pending_;
  bool connected_ {false};
  bool have_sample_ {false};
  Steady::time_point last_sample_ {};
  Steady::time_point last_diagnostic_ {};
  uint64_t sequence_ {0};
  uint64_t reconnects_ {0};
  uint64_t dropped_ {0};
  size_t valid_bins_ {0};
  size_t total_bins_ {0};
  std::string error_;
  std::string last_report_;
};
}  // namespace rov2_sensors
#endif  // ROV2_SENSORS__MANAGED_SENSOR_HPP_
