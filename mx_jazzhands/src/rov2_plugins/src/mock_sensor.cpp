#include <cstdint>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "rov2_core/sensor_plugin.hpp"

namespace rov2_plugins
{

// Mock sensor: emits a monotonically increasing synthetic reading each tick so
// the alive loop and per-plugin status/diagnostics can be validated without
// real hardware.
class MockSensor : public rov2_core::SensorPlugin
{
public:
  bool on_activate() override
  {
    reading_ = 0;
    state_ = rov2_interfaces::msg::PluginStatus::ACTIVE;
    status_message_ = "mock sensor active";
    return true;
  }

  void poll() override
  {
    ++reading_;
    status_message_ = "reading #" + std::to_string(reading_);
  }

private:
  uint64_t reading_ {0};
};

}  // namespace rov2_plugins

PLUGINLIB_EXPORT_CLASS(rov2_plugins::MockSensor, rov2_core::SensorPlugin)
