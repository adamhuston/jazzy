#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "rov2_core/sensor_plugin.hpp"
#include "rov2_plugins/wheel_odometry.hpp"
#include "rov2_plugins/yahboom_board.hpp"

namespace rov2_plugins
{

// Calibrated wheel odometry: integrates the shared board's encoders into a body
// pose and publishes nav_msgs/Odometry on /odom.
//
// Fail-closed by construction:
//   - invalid geometry (non-positive radius/track/counts_per_rev) => FAULT and
//     /odom is suppressed; it will not fabricate a pose from nonsense geometry.
//   - PROVISIONAL best-guess calibration => DEGRADED and the covariance is
//     inflated, so downstream consumers (and the Safety Czar) can see the pose
//     is not yet metrically trustworthy.
//
// Wheel odometry is NOT independent proof of chassis motion (wheels can slip or
// spin free) and shares the motor controller's failure domain; it is motion
// evidence, not a safety-grade position truth.
class Odometry : public rov2_core::SensorPlugin
{
public:
  bool on_configure() override
  {
    device_ = param<std::string>("device", "/dev/myserial");
    topic_ = param<std::string>("topic", "/odom");
    odom_frame_ = param<std::string>("odom_frame_id", "odom");
    base_frame_ = param<std::string>("base_frame_id", "base_link");
    counts_per_rev_ = param<double>("counts_per_rev", 1320.0);
    wheel_radius_m_ = param<double>("wheel_radius_m", 0.0325);
    track_width_m_ = param<double>("track_width_m", 0.167);
    max_wheel_rev_per_s_ = param<double>("max_wheel_rev_per_s", 20.0);
    provisional_ = param<bool>("provisional_calibration", true);

    const auto left_ids = param<std::vector<int64_t>>("left_motor_ids", {1, 3});
    const auto right_ids = param<std::vector<int64_t>>("right_motor_ids", {2, 4});
    left_idx_ = to_zero_based(left_ids);
    right_idx_ = to_zero_based(right_ids);

    geometry_valid_ =
      (counts_per_rev_ > 0.0) && (wheel_radius_m_ > 0.0) && (track_width_m_ > 0.0) &&
      !left_idx_.empty() && !right_idx_.empty();

    proc_ = std::make_unique<WheelEncoderProcessor>(counts_per_rev_, max_wheel_rev_per_s_);

    board_ = YahboomBoard::instance(device_);
    if (!board_->open()) {
      RCLCPP_WARN(
        logger(), "[%s] could not open board on %s: %s",
        instance_name_.c_str(), device_.c_str(), board_->last_error().c_str());
    }
    pub_ = node()->create_publisher<nav_msgs::msg::Odometry>(topic_, rclcpp::SensorDataQoS());

    if (!geometry_valid_) {
      RCLCPP_ERROR(
        logger(), "[%s] invalid odometry geometry; /odom suppressed",
        instance_name_.c_str());
      status_message_ = "invalid geometry; /odom suppressed";
    } else {
      status_message_ = provisional_ ? "configured (PROVISIONAL geometry)" : "configured";
    }
    state_ = PluginStatus::INACTIVE;
    return true;
  }

  bool on_activate() override
  {
    pub_->on_activate();
    state_ = geometry_valid_ ? PluginStatus::ACTIVE : PluginStatus::FAULT;
    status_message_ = geometry_valid_ ? "active" : "invalid geometry; /odom suppressed";
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
    if (!geometry_valid_) {
      state_ = PluginStatus::FAULT;  // never fabricate a pose from bad geometry
      status_message_ = "invalid geometry; /odom suppressed";
      return;
    }
    const EncoderSample s = board_->encoders();
    if (!s.valid) {
      status_message_ = "no encoder data yet";
      state_ = PluginStatus::DEGRADED;
      return;
    }
    if (s.seq == last_seq_) {
      return;
    }
    last_seq_ = s.seq;

    const double stamp_s = std::chrono::duration<double>(s.stamp.time_since_epoch()).count();
    const WheelMotion m = proc_->update(s.counts, stamp_s);
    if (!m.updated) {
      return;  // baseline sample or non-positive dt
    }

    const BodyTwist t = body_twist_from_wheels(
      m.omega_radps, left_idx_, right_idx_, wheel_radius_m_, track_width_m_);
    integrate_pose(pose_, t, m.dt_s);

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = node()->now();
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = base_frame_;
    msg.pose.pose.position.x = pose_.x;
    msg.pose.pose.position.y = pose_.y;
    msg.pose.pose.orientation.z = std::sin(pose_.theta * 0.5);
    msg.pose.pose.orientation.w = std::cos(pose_.theta * 0.5);
    msg.twist.twist.linear.x = t.vx;
    msg.twist.twist.angular.z = t.wz;

    // Inflate covariance heavily while calibration is provisional so consumers
    // never treat a best-guess pose as metrically trustworthy.
    const double pos_var = provisional_ ? 0.5 : 0.05;
    const double yaw_var = provisional_ ? 1.0 : 0.1;
    msg.pose.covariance[0] = pos_var;    // x
    msg.pose.covariance[7] = pos_var;    // y
    msg.pose.covariance[35] = yaw_var;   // yaw
    msg.twist.covariance[0] = pos_var;
    msg.twist.covariance[35] = yaw_var;

    pub_->publish(msg);
    state_ = provisional_ ? PluginStatus::DEGRADED : PluginStatus::ACTIVE;
    status_message_ = provisional_ ? "publishing (provisional calibration)" : "publishing";
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

  static std::vector<int> to_zero_based(const std::vector<int64_t> & ids)
  {
    std::vector<int> out;
    for (int64_t id : ids) {
      if (id >= 1 && id <= 4) {
        out.push_back(static_cast<int>(id - 1));
      }
    }
    return out;
  }

  const std::string prefix_ {"odometry"};
  std::string device_;
  std::string topic_;
  std::string odom_frame_;
  std::string base_frame_;
  double counts_per_rev_ {1320.0};
  double wheel_radius_m_ {0.0325};
  double track_width_m_ {0.167};
  double max_wheel_rev_per_s_ {20.0};
  bool provisional_ {true};
  bool geometry_valid_ {false};
  std::vector<int> left_idx_;
  std::vector<int> right_idx_;
  uint64_t last_seq_ {0};
  Pose2D pose_;

  std::unique_ptr<WheelEncoderProcessor> proc_;
  std::shared_ptr<YahboomBoard> board_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

}  // namespace rov2_plugins

PLUGINLIB_EXPORT_CLASS(rov2_plugins::Odometry, rov2_core::SensorPlugin)
