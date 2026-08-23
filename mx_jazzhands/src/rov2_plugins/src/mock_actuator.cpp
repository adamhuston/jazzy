#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rov2_core/actuator_plugin.hpp"

namespace rov2_plugins
{

// Mock actuator: records the last commanded Twist into its status message so
// the command gateway path (mode + cmd_vel -> actuator) can be validated
// without real hardware.
class MockActuator : public rov2_core::ActuatorPlugin
{
public:
  bool on_activate() override
  {
    state_ = rov2_interfaces::msg::PluginStatus::ACTIVE;
    status_message_ = "mock actuator active";
    return true;
  }

  void apply_command(const geometry_msgs::msg::Twist & cmd) override
  {
    status_message_ = "lin.x=" + std::to_string(cmd.linear.x) +
      " ang.z=" + std::to_string(cmd.angular.z);
  }
};

}  // namespace rov2_plugins

PLUGINLIB_EXPORT_CLASS(rov2_plugins::MockActuator, rov2_core::ActuatorPlugin)
