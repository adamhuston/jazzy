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
#ifndef ROV2_SENSORS__SOURCE_HPP_
#define ROV2_SENSORS__SOURCE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/clock.hpp"

namespace rov2_sensors
{
using Steady = std::chrono::steady_clock;

template<typename Message>
struct Observation
{
  Message message;
  Steady::time_point acquired;
};

template<typename Message>
class Source
{
public:
  virtual ~Source() = default;
  // connect verifies identity/health and starts acquisition. All methods are
  // called exclusively by one worker. Reads must have bounded waits.
  virtual void connect() = 0;
  virtual std::vector<Observation<Message>> read(const rclcpp::Clock & clock) = 0;
  virtual void disconnect() = 0;
};

inline double age_ms(Steady::time_point observation, Steady::time_point now)
{
  return std::chrono::duration<double, std::milli>(now - observation).count();
}

inline bool fresh(Steady::time_point observation, Steady::time_point now, double timeout_ms)
{
  const double age = age_ms(observation, now);
  return age >= 0.0 && age <= timeout_ms;
}
}  // namespace rov2_sensors
#endif  // ROV2_SENSORS__SOURCE_HPP_
