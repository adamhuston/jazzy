#ifndef ROV2_CORE__QOS_PROFILES_HPP_
#define ROV2_CORE__QOS_PROFILES_HPP_

#include "rclcpp/qos.hpp"

// Documented QoS profiles by topic class (see skills/qos-policy-and-validation.md).
// Cross-machine interop with a remote Isaac Sim requires matching ROS_DOMAIN_ID
// and RMW (repo default: rmw_fastrtps_cpp / Fast DDS).
namespace rov2_core
{
namespace qos
{

// Control: timely, deterministic, reliable command delivery for safety.
inline rclcpp::QoS control()
{
  rclcpp::QoS profile(rclcpp::KeepLast(10));
  profile.reliable();
  profile.durability_volatile();
  return profile;
}

// Status/health: stable observability in CLI/dashboards; non-brittle heartbeat.
inline rclcpp::QoS status()
{
  rclcpp::QoS profile(rclcpp::KeepLast(10));
  profile.reliable();
  profile.durability_volatile();
  return profile;
}

// Sensor: throughput-oriented default; tune per stream at the plugin boundary.
inline rclcpp::QoS sensor()
{
  rclcpp::QoS profile(rclcpp::KeepLast(5));
  profile.best_effort();
  profile.durability_volatile();
  return profile;
}

}  // namespace qos
}  // namespace rov2_core

#endif  // ROV2_CORE__QOS_PROFILES_HPP_
