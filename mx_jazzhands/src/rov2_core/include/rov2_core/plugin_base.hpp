#ifndef ROV2_CORE__PLUGIN_BASE_HPP_
#define ROV2_CORE__PLUGIN_BASE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "rov2_interfaces/msg/plugin_status.hpp"

namespace rov2_core
{

// Base contract for all ROV2 framework plugins. Lifecycle-shaped to mirror
// ROS 2 managed-node semantics without forcing each plugin to be its own node.
// Plugins are loaded by the core host via pluginlib; concrete implementations
// live in separate packages that depend on rov2_core.
class PluginBase
{
public:
  using PluginStatus = rov2_interfaces::msg::PluginStatus;

  virtual ~PluginBase() = default;

  // Called once after construction, before any other lifecycle hook. The host
  // node is provided (as a weak pointer to avoid an ownership cycle) so plugins
  // can create their own publishers/subscribers on the framework node; logger
  // and clock are derived from it.
  virtual bool on_init(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
    const std::string & instance_name)
  {
    node_ = node;
    const auto locked = node_.lock();
    if (!locked) {
      return false;
    }
    logger_ = std::make_shared<rclcpp::Logger>(locked->get_logger());
    clock_ = locked->get_clock();
    instance_name_ = instance_name;
    state_ = PluginStatus::UNCONFIGURED;
    return true;
  }

  virtual bool on_configure() { state_ = PluginStatus::INACTIVE; return true; }
  virtual bool on_activate() { state_ = PluginStatus::ACTIVE; return true; }
  virtual bool on_deactivate() { state_ = PluginStatus::INACTIVE; return true; }
  virtual bool on_cleanup() { state_ = PluginStatus::UNCONFIGURED; return true; }
  virtual bool on_shutdown() { state_ = PluginStatus::UNKNOWN; return true; }

  // Category label: "sensor" | "actuator" | "brain".
  virtual std::string category() const = 0;

  // Health snapshot aggregated into SystemStatus each alive-loop tick.
  virtual PluginStatus get_status() const
  {
    PluginStatus s;
    s.name = instance_name_;
    s.type = plugin_type_;
    s.category = category();
    s.state = state_;
    s.message = status_message_;
    return s;
  }

  const std::string & instance_name() const { return instance_name_; }
  void set_plugin_type(const std::string & type) { plugin_type_ = type; }
  uint8_t state() const { return state_; }

protected:
  rclcpp::Logger logger() const
  {
    return logger_ ? *logger_ : rclcpp::get_logger("rov2_core.plugin");
  }

  // Host framework node, for plugins that create their own pub/sub. Returns
  // nullptr if the node has been destroyed; callers must null-check.
  rclcpp_lifecycle::LifecycleNode::SharedPtr node() const { return node_.lock(); }

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::string instance_name_;
  std::string plugin_type_;
  std::string status_message_;
  uint8_t state_ {rov2_interfaces::msg::PluginStatus::UNKNOWN};
};

}  // namespace rov2_core

#endif  // ROV2_CORE__PLUGIN_BASE_HPP_
