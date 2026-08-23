#include "pluginlib/class_list_macros.hpp"
#include "rov2_core/brain_plugin.hpp"

namespace rov2_plugins
{

// No-op brain: does nothing on each tick. The framework is a nervous system,
// not a brain (see skills/architecture-guardrails.md); real decision logic
// belongs in dedicated brain plugins, never in core.
class NoopBrain : public rov2_core::BrainPlugin
{
public:
  bool on_activate() override
  {
    state_ = rov2_interfaces::msg::PluginStatus::ACTIVE;
    status_message_ = "idle (no-op)";
    return true;
  }

  void think() override {}
};

}  // namespace rov2_plugins

PLUGINLIB_EXPORT_CLASS(rov2_plugins::NoopBrain, rov2_core::BrainPlugin)
