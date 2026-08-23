#ifndef ROV2_CORE__BRAIN_PLUGIN_HPP_
#define ROV2_CORE__BRAIN_PLUGIN_HPP_

#include <string>

#include "rov2_core/plugin_base.hpp"

namespace rov2_core
{

// Base class for brain plugins. Loaded under lookup base "rov2_core::BrainPlugin".
// The framework is a nervous system, not a brain: decision logic stays here in
// plugins, never in core (see skills/architecture-guardrails.md).
class BrainPlugin : public PluginBase
{
public:
  std::string category() const override { return "brain"; }

  // Optional per-tick decision hook. No-op brains simply do nothing.
  virtual void think() {}
};

}  // namespace rov2_core

#endif  // ROV2_CORE__BRAIN_PLUGIN_HPP_
