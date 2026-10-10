#include <algorithm>
#include <array>
#include <cmath>
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
    require_board_ = param<bool>("require_board", true);
    require_feedback_ = param<bool>("require_feedback", true);
    feedback_timeout_ms_ = param<double>("feedback_timeout_ms", 250.0);

    std::string cfg_err;
    if (!validate_config(cfg_err)) {
      RCLCPP_ERROR(
        logger(), "[%s] invalid drivetrain config: %s",
        instance_name_.c_str(), cfg_err.c_str());
      status_message_ = "invalid config: " + cfg_err;
      state_ = PluginStatus::FAULT;
      return false;  // fail closed: a mis-configured drivetrain must not load
    }

    board_ = YahboomBoard::instance(device_);
    if (!board_->open()) {
      RCLCPP_WARN(
        logger(), "[%s] could not open board on %s: %s (motion inhibited until available)",
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
    fault_latched_ = false;
    state_ = PluginStatus::ACTIVE;
    status_message_ = "active";
    return true;
  }

  bool on_deactivate() override
  {
    const bool ok = stop_now();  // fail-safe stop
    state_ = PluginStatus::INACTIVE;
    status_message_ = ok ? "inactive (stopped)" : "inactive (STOP WRITE FAILED)";
    return true;
  }

  bool on_cleanup() override
  {
    stop_now();
    board_.reset();
    state_ = PluginStatus::UNCONFIGURED;
    return true;
  }

  bool on_shutdown() override
  {
    stop_now();
    board_.reset();
    state_ = PluginStatus::UNKNOWN;
    return true;
  }

  void apply_command(const geometry_msgs::msg::Twist & cmd) override
  {
    if (!board_) {
      state_ = PluginStatus::FAULT;
      status_message_ = "no board";
      return;
    }

    const double v = cmd.linear.x;
    const double w = cmd.angular.z;

    // Never translate NaN/Inf into motor units; treat it as a latched fault.
    if (!std::isfinite(v) || !std::isfinite(w)) {
      board_->set_motor(0, 0, 0, 0);
      fault_latched_ = true;
      state_ = PluginStatus::FAULT;
      status_message_ = "non-finite command rejected";
      return;
    }

    const bool want_zero = (std::abs(v) < kZeroEps) && (std::abs(w) < kZeroEps);

    // Readiness gate: "open" is not "responding" is not "feeding fresh feedback".
    const BoardHealth h = board_->health();
    bool ready = h.open;
    std::string why;
    if (!h.open) {
      why = "port closed";
    }
    if (ready && require_board_ && !h.responding) {
      ready = false;
      why = "board not responding";
    }
    if (ready && require_feedback_ &&
      !(h.encoder_age_ms >= 0.0 && h.encoder_age_ms <= feedback_timeout_ms_))
    {
      ready = false;
      why = (h.encoder_age_ms < 0.0) ? "no encoder feedback" : "encoder feedback stale";
    }

    if (!ready) {
      // Inhibit motion with a best-effort stop. Not latched: recovers
      // transparently, but a nonzero command is refused while untrustworthy.
      const bool ok = board_->set_motor(0, 0, 0, 0);
      state_ = PluginStatus::DEGRADED;
      status_message_ = "motion inhibited (" + why + ")" + (ok ? "" : "; STOP WRITE FAILED");
      return;
    }

    if (fault_latched_) {
      // After a write fault, require an explicit zero command to re-arm so a
      // recovering link never silently resumes a stale motion command.
      const bool ok = board_->set_motor(0, 0, 0, 0);
      if (want_zero && ok) {
        fault_latched_ = false;
        state_ = PluginStatus::ACTIVE;
        status_message_ = "recovered; awaiting fresh command";
      } else {
        state_ = PluginStatus::FAULT;
        status_message_ = ok ? "latched fault: send zero to re-arm" :
          "latched fault: STOP WRITE FAILED";
      }
      return;
    }

    // linear.y (lateral) is unsupported on a skid-steer chassis and ignored.
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

    const bool ok = board_->set_motor(m[0], m[1], m[2], m[3]);
    if (!ok) {
      fault_latched_ = true;
      state_ = PluginStatus::FAULT;
      status_message_ = "motor write FAILED: " + board_->last_error();
      return;
    }
    state_ = PluginStatus::ACTIVE;
    status_message_ = "m=[" + std::to_string(m[0]) + "," + std::to_string(m[1]) + "," +
      std::to_string(m[2]) + "," + std::to_string(m[3]) + "]";
  }

private:
  // Reject obviously-invalid configuration so a mis-wired mapping can never
  // reach the motors. Returns false and fills `err` on the first problem.
  bool validate_config(std::string & err)
  {
    if (!std::isfinite(lin_scale_) || !std::isfinite(ang_scale_)) {
      err = "lin_scale/ang_scale must be finite";
      return false;
    }
    if (max_motor_ <= 0 || max_motor_ > 100) {
      err = "max_motor must be in (0,100], got " + std::to_string(max_motor_);
      return false;
    }
    if (left_ids_.empty() || right_ids_.empty()) {
      err = "left_motor_ids and right_motor_ids must both be non-empty";
      return false;
    }
    std::array<int, 5> seen {{0, 0, 0, 0, 0}};  // index 1..4
    const auto check = [&](const std::vector<int64_t> & ids) -> bool {
        for (int64_t id : ids) {
          if (id < 1 || id > 4) {
            err = "motor id out of range [1,4]: " + std::to_string(id);
            return false;
          }
          if (seen[id]) {
            err = "motor id duplicated / assigned to both sides: " + std::to_string(id);
            return false;
          }
          seen[id] = 1;
        }
        return true;
      };
    if (!check(left_ids_)) {return false;}
    if (!check(right_ids_)) {return false;}
    return true;
  }

  // Best-effort fail-safe stop. Returns the transmit result when a link is
  // open; returns true when there is no open link (nothing is being driven).
  bool stop_now()
  {
    if (!board_ || !board_->is_open()) {
      return true;
    }
    return board_->set_motor(0, 0, 0, 0);
  }
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

  // Fail-safe gating.
  static constexpr double kZeroEps {1e-6};
  bool require_board_ {true};         // nonzero motion requires a responding board
  bool require_feedback_ {true};      // ...and fresh encoder telemetry
  double feedback_timeout_ms_ {250.0};
  bool fault_latched_ {false};        // set on a failed write; needs explicit re-arm

  std::shared_ptr<YahboomBoard> board_;
};

}  // namespace rov2_plugins

PLUGINLIB_EXPORT_CLASS(rov2_plugins::Drivetrain, rov2_core::ActuatorPlugin)
