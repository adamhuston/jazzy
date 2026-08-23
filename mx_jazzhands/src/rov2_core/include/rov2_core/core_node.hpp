#ifndef ROV2_CORE__CORE_NODE_HPP_
#define ROV2_CORE__CORE_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "pluginlib/class_loader.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rov2_interfaces/msg/mode_command.hpp"
#include "rov2_interfaces/msg/system_status.hpp"
#include "rov2_interfaces/srv/set_mode.hpp"

#include "rov2_core/actuator_plugin.hpp"
#include "rov2_core/brain_plugin.hpp"
#include "rov2_core/sensor_plugin.hpp"

namespace rov2_core
{

// Core runtime host: a managed (lifecycle) node that hosts pluginlib-loaded
// sensor/actuator/brain plugins, runs the alive loop, publishes framework
// status + diagnostics, and accepts mode/motion commands. The framework is a
// nervous system, not a brain (see skills/architecture-guardrails.md).
class CoreNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit CoreNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void alive_loop();
  void on_mode_command(const rov2_interfaces::msg::ModeCommand::SharedPtr msg);
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_set_mode(
    const std::shared_ptr<rov2_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<rov2_interfaces::srv::SetMode::Response> response);

  // Loads the plugin names listed in `param_name` using `loader`, running the
  // init/configure hooks. Missing optional plugins are non-fatal.
  template<typename PluginT>
  void load_plugins(
    const std::string & param_name,
    pluginlib::ClassLoader<PluginT> & loader,
    std::vector<std::shared_ptr<PluginT>> & out);

  bool apply_mode(uint8_t mode, const std::string & reason, std::string & message);
  rov2_interfaces::msg::SystemStatus build_status();
  void publish_diagnostics(const rov2_interfaces::msg::SystemStatus & status);

  // Plugin loaders (base package "rov2_core").
  std::unique_ptr<pluginlib::ClassLoader<SensorPlugin>> sensor_loader_;
  std::unique_ptr<pluginlib::ClassLoader<ActuatorPlugin>> actuator_loader_;
  std::unique_ptr<pluginlib::ClassLoader<BrainPlugin>> brain_loader_;

  std::vector<std::shared_ptr<SensorPlugin>> sensors_;
  std::vector<std::shared_ptr<ActuatorPlugin>> actuators_;
  std::vector<std::shared_ptr<BrainPlugin>> brains_;

  // Publishers / subscribers / services.
  rclcpp_lifecycle::LifecyclePublisher<rov2_interfaces::msg::SystemStatus>::SharedPtr
    status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diag_pub_;
  rclcpp::Subscription<rov2_interfaces::msg::ModeCommand>::SharedPtr mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Service<rov2_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
  rclcpp::TimerBase::SharedPtr alive_timer_;

  // Runtime state.
  double loop_rate_hz_ {10.0};
  double loop_period_ms_ {100.0};
  uint32_t loop_count_ {0};
  double last_jitter_ms_ {0.0};
  rclcpp::Time last_tick_;
  bool have_last_tick_ {false};

  uint8_t mode_ {rov2_interfaces::msg::ModeCommand::STANDBY};
  geometry_msgs::msg::Twist last_cmd_;
};

}  // namespace rov2_core

#endif  // ROV2_CORE__CORE_NODE_HPP_
