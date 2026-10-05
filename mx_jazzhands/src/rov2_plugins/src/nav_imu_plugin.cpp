#include <cmath>
#include <memory>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "rov2_core/sensor_plugin.hpp"
#include "rov2_plugins/yahboom_board.hpp"

namespace rov2_plugins
{

// Navigation IMU sensor: reads the Yahboom onboard 9-axis IMU over the shared
// board link and publishes sensor_msgs/Imu. This is the NAVIGATION inertial
// source; the Safety Czar deliberately uses the independent ADXL345 instead, so
// this plugin is intentionally not a safety input.
class NavImu : public rov2_core::SensorPlugin
{
public:
  bool on_configure() override
  {
    device_ = param<std::string>("device", "/dev/myserial");
    frame_id_ = param<std::string>("frame_id", "imu_link");
    topic_ = param<std::string>("topic", "/imu/data");

    board_ = YahboomBoard::instance(device_);
    if (!board_->open()) {
      RCLCPP_WARN(
        logger(), "[%s] could not open board on %s: %s",
        instance_name_.c_str(), device_.c_str(), board_->last_error().c_str());
    }
    pub_ = node()->create_publisher<sensor_msgs::msg::Imu>(topic_, rclcpp::SensorDataQoS());
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
    const ImuSample s = board_->imu();
    if (!s.valid) {
      status_message_ = "no IMU data yet";
      return;
    }
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = node()->now();
    msg.header.frame_id = frame_id_;

    msg.linear_acceleration.x = s.ax;
    msg.linear_acceleration.y = s.ay;
    msg.linear_acceleration.z = s.az;
    msg.angular_velocity.x = s.gx;
    msg.angular_velocity.y = s.gy;
    msg.angular_velocity.z = s.gz;

    // No fused orientation is produced here; flag it per ROS convention.
    msg.orientation.w = 1.0;
    msg.orientation_covariance[0] = -1.0;

    pub_->publish(msg);
    status_message_ = "publishing";
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

  const std::string prefix_ {"nav_imu"};
  std::string device_;
  std::string frame_id_;
  std::string topic_;
  std::shared_ptr<YahboomBoard> board_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
};

}  // namespace rov2_plugins

PLUGINLIB_EXPORT_CLASS(rov2_plugins::NavImu, rov2_core::SensorPlugin)
