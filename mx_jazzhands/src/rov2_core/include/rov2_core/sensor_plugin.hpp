#ifndef ROV2_CORE__SENSOR_PLUGIN_HPP_
#define ROV2_CORE__SENSOR_PLUGIN_HPP_

#include <string>

#include "rov2_core/plugin_base.hpp"

namespace rov2_core
{

// Base class for sensor-facing plugins. Loaded under lookup base
// "rov2_core::SensorPlugin".
class SensorPlugin : public PluginBase
{
public:
  std::string category() const override { return "sensor"; }

  // Called on each alive-loop tick while active; read and publish sensor data.
  virtual void poll() {}
};

}  // namespace rov2_core

#endif  // ROV2_CORE__SENSOR_PLUGIN_HPP_
