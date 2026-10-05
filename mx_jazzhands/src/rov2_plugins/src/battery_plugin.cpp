#include <limits>
#include <memory>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include "rov2_core/sensor_plugin.hpp"
#include "rov2_plugins/yahboom_board.hpp"

namespace rov2_plugins
{

// Battery sensor: reads pack voltage from the shared Yahboom board (reported in
// its speed/telemetry frame) and publishes sensor_msgs/BatteryState.
class Battery : public rov2_core::SensorPlugin
{
public:
  bool on_configure() override
  {
    device_ = param<std::string>("device", "/dev/myserial");
    topic_ = param<std::string>("topic", "/battery");

    board_ = YahboomBoard::instance(device_);
    if (!board_->open()) {
      RCLCPP_WARN(
        logger(), "[%s] could not open board on %s: %s",
        instance_name_.c_str(), device_.c_str(), board_->last_error().c_str());
    }
    pub_ = node()->create_publisher<sensor_msgs::msg::BatteryState>(topic_, rclcpp::SensorDataQoS());
    status_message_ = "configured";
    state_ = PluginStatus::INACTIVE;
    return true;
  }

  bool on_activate() override
  {
    pub_->on_activate();
    state_ = PluginStatus::ACTIVE;
    status_message_ = "active";
    return true;
  }

  bool on_deactivate() override
  {
    pub_->on_deactivate();
    state_ = PluginStatus::INACTIVE;
    status_message_ = "inactive";
    return true;
  }

  bool on_cleanup() override
  {
    pub_.reset();
    board_.reset();
    state_ = PluginStatus::UNCONFIGURED;
    return true;
  }

  void poll() override
  {
    if (!board_ || !pub_ || !pub_->is_activated()) {
      return;
    }
    const BatterySample s = board_->battery();
    if (!s.valid) {
      status_message_ = "no battery data yet";
      return;
    }
    const double nan = std::numeric_limits<double>::quiet_NaN();
    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = node()->now();
    msg.voltage = static_cast<float>(s.voltage);
    msg.current = static_cast<float>(nan);
    msg.charge = static_cast<float>(nan);
    msg.capacity = static_cast<float>(nan);
    msg.design_capacity = static_cast<float>(nan);
    msg.percentage = static_cast<float>(nan);
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    msg.power_supply_technology =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    msg.present = true;

    pub_->publish(msg);
    status_message_ = std::to_string(s.voltage) + " V";
  }

private:
  template<typename T>
  T param(const std::string & key, const T & def)
  {
    auto n = node();
    const std::string full = prefix_ + "." + key;
    if (!n->has_parameter(full)) {
      n->declare_parameter(full, rclcpp::ParameterValue(def));
    }
    return n->get_parameter(full).get_value<T>();
  }

  const std::string prefix_ {"battery"};
  std::string device_;
  std::string topic_;
  std::shared_ptr<YahboomBoard> board_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_;
};

}  // namespace rov2_plugins

PLUGINLIB_EXPORT_CLASS(rov2_plugins::Battery, rov2_core::SensorPlugin)
