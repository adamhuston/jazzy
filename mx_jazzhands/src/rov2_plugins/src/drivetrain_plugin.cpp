#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_value.hpp"

#include "rov2_core/actuator_plugin.hpp"
#include "rov2_plugins/yahboom_board.hpp"

namespace rov2_plugins
{

// Drivetrain actuator: converts an approved body-level Twist (linear.x, angular.z)
// into four independent wheel speeds (skid-steer "wheel expansion") and sends
// them to the shared Yahboom board. Per the design, the Safety Czar/brain own
// the body-level command; this plugin owns the body->wheel kinematics.
//
// Phase 1: open-loop normalized mixing (lin_scale/ang_scale are uncalibrated
// gains). Phase 2 will replace these with calibrated kinematics once wheel
// geometry, encoder counts-per-rev, and max speeds are measured.
class Drivetrain : public rov2_core::ActuatorPlugin
{
public:
  bool on_configure() override
  {
    device_ = param<std::string>("device", "/dev/myserial");
    lin_scale_ = param<double>("lin_scale", 100.0);
    ang_scale_ = param<double>("ang_scale", 100.0);
    max_motor_ = static_cast<int>(param<int64_t>("max_motor", 100));
    invert_left_ = param<bool>("invert_left", false);
    invert_right_ = param<bool>("invert_right", false);
    left_ids_ = param<std::vector<int64_t>>("left_motor_ids", {1, 3});
    right_ids_ = param<std::vector<int64_t>>("right_motor_ids", {2, 4});

    board_ = YahboomBoard::instance(device_);
    if (!board_->open()) {
      RCLCPP_WARN(
        logger(), "[%s] could not open board on %s: %s (will no-op until available)",
        instance_name_.c_str(), device_.c_str(), board_->last_error().c_str());
      status_message_ = "board unavailable: " + board_->last_error();
    } else {
      status_message_ = "configured on " + device_;
    }
    state_ = PluginStatus::INACTIVE;
    return true;
  }

  bool on_activate() override
  {
    state_ = PluginStatus::ACTIVE;
    status_message_ = "active";
    return true;
  }

  bool on_deactivate() override
  {
    if (board_ && board_->is_open()) {
      board_->set_motor(0, 0, 0, 0);  // fail-safe stop
    }
    state_ = PluginStatus::INACTIVE;
    status_message_ = "inactive (stopped)";
    return true;
  }

  bool on_cleanup() override
  {
    if (board_ && board_->is_open()) {
      board_->set_motor(0, 0, 0, 0);
    }
    board_.reset();
    state_ = PluginStatus::UNCONFIGURED;
    return true;
  }

  bool on_shutdown() override
  {
    if (board_ && board_->is_open()) {
      board_->set_motor(0, 0, 0, 0);
    }
    board_.reset();
    state_ = PluginStatus::UNKNOWN;
    return true;
  }

  void apply_command(const geometry_msgs::msg::Twist & cmd) override
  {
    if (!board_ || !board_->is_open()) {
      status_message_ = "board not open";
      return;
    }
    // linear.y (lateral) is unsupported on a skid-steer chassis and ignored.
    const double v = cmd.linear.x;
    const double w = cmd.angular.z;

    double left = v * lin_scale_ - w * ang_scale_;
    double right = v * lin_scale_ + w * ang_scale_;
    if (invert_left_) {left = -left;}
    if (invert_right_) {right = -right;}

    const auto clamp_motor = [this](double x) -> int {
      const double lim = static_cast<double>(max_motor_);
      return static_cast<int>(std::max(-lim, std::min(lim, x)));
    };

    int m[4] = {0, 0, 0, 0};
    for (int64_t id : left_ids_) {
      if (id >= 1 && id <= 4) {m[id - 1] = clamp_motor(left);}
    }
    for (int64_t id : right_ids_) {
      if (id >= 1 && id <= 4) {m[id - 1] = clamp_motor(right);}
    }
    board_->set_motor(m[0], m[1], m[2], m[3]);
    status_message_ = "m=[" + std::to_string(m[0]) + "," + std::to_string(m[1]) + "," +
      std::to_string(m[2]) + "," + std::to_string(m[3]) + "]";
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

  const std::string prefix_ {"drivetrain"};
  std::string device_;
  double lin_scale_ {100.0};
  double ang_scale_ {100.0};
  int max_motor_ {100};
  bool invert_left_ {false};
  bool invert_right_ {false};
  std::vector<int64_t> left_ids_ {1, 3};
  std::vector<int64_t> right_ids_ {2, 4};

  std::shared_ptr<YahboomBoard> board_;
};

}  // namespace rov2_plugins

PLUGINLIB_EXPORT_CLASS(rov2_plugins::Drivetrain, rov2_core::ActuatorPlugin)
