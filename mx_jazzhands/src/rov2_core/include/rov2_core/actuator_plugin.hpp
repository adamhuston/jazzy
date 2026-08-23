#ifndef ROV2_CORE__ACTUATOR_PLUGIN_HPP_
#define ROV2_CORE__ACTUATOR_PLUGIN_HPP_

#include <string>

#include "geometry_msgs/msg/twist.hpp"

#include "rov2_core/plugin_base.hpp"

namespace rov2_core
{

// Base class for actuator-facing plugins. Loaded under lookup base
// "rov2_core::ActuatorPlugin". Hardware-facing implementations should follow
// ros2_control-compatible conventions (see skills/ros2-control-integration.md).
class ActuatorPlugin : public PluginBase
{
public:
  std::string category() const override { return "actuator"; }

  // Apply the latest commanded motion. Twist is a phase-1 placeholder contract.
  virtual void apply_command(const geometry_msgs::msg::Twist & cmd) { (void)cmd; }
};

}  // namespace rov2_core

#endif  // ROV2_CORE__ACTUATOR_PLUGIN_HPP_
