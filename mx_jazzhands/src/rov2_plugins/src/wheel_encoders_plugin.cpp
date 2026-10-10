#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rov2_core/sensor_plugin.hpp"
#include "rov2_plugins/wheel_odometry.hpp"
#include "rov2_plugins/yahboom_board.hpp"

namespace rov2_plugins
{

// Raw wheel-encoder feedback: converts the shared board's cumulative encoder
// counts into per-wheel angle + angular velocity and publishes sensor_msgs/
// JointState. This is the first, least-processed motion evidence. The scale
// (counts_per_rev) is a PROVISIONAL best guess until bench-measured, so while
// `provisional_calibration` is true the plugin reports DEGRADED: the data is
// structurally correct but its metric scale is not yet trustworthy.
class WheelEncoders : public rov2_core::SensorPlugin
{
public:
  bool on_configure() override
  {
    device_ = param<std::string>("device", "/dev/myserial");
    topic_ = param<std::string>("topic", "/wheel_states");
    counts_per_rev_ = param<double>("counts_per_rev", 1320.0);
    max_wheel_rev_per_s_ = param<double>("max_wheel_rev_per_s", 20.0);
    provisional_ = param<bool>("provisional_calibration", true);
    joint_names_ = param<std::vector<std::string>>(
      "joint_names", {"front_left", "front_right", "rear_left", "rear_right"});

    proc_ = std::make_unique<WheelEncoderProcessor>(counts_per_rev_, max_wheel_rev_per_s_);

    board_ = YahboomBoard::instance(device_);
    if (!board_->open()) {
      RCLCPP_WARN(
        logger(), "[%s] could not open board on %s: %s",
        instance_name_.c_str(), device_.c_str(), board_->last_error().c_str());
    }
    pub_ = node()->create_publisher<sensor_msgs::msg::JointState>(
      topic_, rclcpp::SensorDataQoS());
    status_message_ = provisional_ ? "configured (PROVISIONAL counts_per_rev)" : "configured";
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
    proc_.reset();
    state_ = PluginStatus::UNCONFIGURED;
    return true;
  }

  void poll() override
  {
    if (!board_ || !pub_ || !pub_->is_activated()) {
      return;
    }
    const EncoderSample s = board_->encoders();
    if (!s.valid) {
      status_message_ = "no encoder data yet";
      state_ = PluginStatus::DEGRADED;
      return;
    }
    if (s.seq == last_seq_) {
      return;  // no new board sample since last poll
    }
    last_seq_ = s.seq;

    const double stamp_s = std::chrono::duration<double>(s.stamp.time_since_epoch()).count();
    const WheelMotion m = proc_->update(s.counts, stamp_s);

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node()->now();
    msg.name = joint_names_;
    msg.position = {m.angle_rad[0], m.angle_rad[1], m.angle_rad[2], m.angle_rad[3]};
    msg.velocity = {m.omega_radps[0], m.omega_radps[1], m.omega_radps[2], m.omega_radps[3]};
    pub_->publish(msg);

    state_ = provisional_ ? PluginStatus::DEGRADED : PluginStatus::ACTIVE;
    status_message_ = provisional_ ? "publishing (provisional scale)" : "publishing";
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

  const std::string prefix_ {"wheel_encoders"};
  std::string device_;
  std::string topic_;
  double counts_per_rev_ {1320.0};
  double max_wheel_rev_per_s_ {20.0};
  bool provisional_ {true};
  std::vector<std::string> joint_names_;
  uint64_t last_seq_ {0};

  std::unique_ptr<WheelEncoderProcessor> proc_;
  std::shared_ptr<YahboomBoard> board_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
};

}  // namespace rov2_plugins

PLUGINLIB_EXPORT_CLASS(rov2_plugins::WheelEncoders, rov2_core::SensorPlugin)
