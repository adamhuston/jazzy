#ifndef ROV2_PLUGINS__WHEEL_ODOMETRY_HPP_
#define ROV2_PLUGINS__WHEEL_ODOMETRY_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace rov2_plugins
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

// Wrap-correct signed delta between two cumulative int32 encoder readings.
// Modular subtraction in uint32 space then reinterpreting as int32 yields the
// shortest signed distance, so a 32-bit counter wrap does not produce a huge
// spurious jump.
inline int32_t encoder_delta(int32_t prev, int32_t curr)
{
  return static_cast<int32_t>(static_cast<uint32_t>(curr) - static_cast<uint32_t>(prev));
}

// Per-wheel result of processing one encoder sample.
struct WheelMotion
{
  std::array<double, 4> angle_rad {{0.0, 0.0, 0.0, 0.0}};    // accumulated wheel angle
  std::array<double, 4> omega_radps {{0.0, 0.0, 0.0, 0.0}};  // per-wheel angular rate
  std::array<double, 4> rejected {{0.0, 0.0, 0.0, 0.0}};     // 1.0 if delta rejected
  double dt_s {0.0};
  bool updated {false};  // false for first sample, duplicates, or non-positive dt
};

// Converts the shared board's cumulative encoder counts into per-wheel angle and
// angular velocity. Pure/stateful-but-ROS-free so it is directly unit-testable.
//
// Robustness rules:
//   - first sample only establishes a baseline (updated=false)
//   - non-increasing timestamps (duplicates / out-of-order) are ignored
//   - a per-wheel delta larger than max_wheel_rev_per_s implies a board reset or
//     serial glitch, so that wheel's delta is rejected (treated as 0) rather than
//     corrupting the accumulated angle
class WheelEncoderProcessor
{
public:
  WheelEncoderProcessor(double counts_per_rev, double max_wheel_rev_per_s)
  : counts_per_rev_(counts_per_rev), max_wheel_rev_per_s_(max_wheel_rev_per_s) {}

  WheelMotion update(const std::array<int32_t, 4> & counts, double stamp_s)
  {
    WheelMotion out;
    if (counts_per_rev_ <= 0.0) {
      return out;  // uncalibrated: refuse to fabricate motion
    }
    if (!have_prev_) {
      prev_counts_ = counts;
      prev_stamp_s_ = stamp_s;
      have_prev_ = true;
      out.angle_rad = angle_rad_;
      return out;
    }
    const double dt = stamp_s - prev_stamp_s_;
    if (dt <= 0.0) {
      out.angle_rad = angle_rad_;
      out.omega_radps = last_omega_;
      return out;  // duplicate / out-of-order sample
    }
    const double max_dcounts = max_wheel_rev_per_s_ * counts_per_rev_ * dt;
    for (int i = 0; i < 4; ++i) {
      double dc = static_cast<double>(encoder_delta(prev_counts_[i], counts[i]));
      if (max_dcounts > 0.0 && std::abs(dc) > max_dcounts) {
        dc = 0.0;  // implausible jump -> reject this wheel's delta
        out.rejected[i] = 1.0;
      }
      const double rev = dc / counts_per_rev_;
      angle_rad_[i] += rev * kTwoPi;
      out.omega_radps[i] = rev * kTwoPi / dt;
    }
    prev_counts_ = counts;
    prev_stamp_s_ = stamp_s;
    last_omega_ = out.omega_radps;
    out.angle_rad = angle_rad_;
    out.dt_s = dt;
    out.updated = true;
    return out;
  }

private:
  double counts_per_rev_;
  double max_wheel_rev_per_s_;
  bool have_prev_ {false};
  std::array<int32_t, 4> prev_counts_ {{0, 0, 0, 0}};
  double prev_stamp_s_ {0.0};
  std::array<double, 4> angle_rad_ {{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 4> last_omega_ {{0.0, 0.0, 0.0, 0.0}};
};

// Body twist from per-wheel angular velocities for a skid/differential chassis.
// left_idx / right_idx are 0-based wheel indices (into the 4-wheel arrays).
struct BodyTwist
{
  double vx {0.0};  // m/s forward
  double wz {0.0};  // rad/s yaw
};

inline double mean_of(const std::array<double, 4> & v, const std::vector<int> & idx)
{
  if (idx.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (int i : idx) {
    if (i >= 0 && i < 4) {
      sum += v[i];
    }
  }
  return sum / static_cast<double>(idx.size());
}

inline BodyTwist body_twist_from_wheels(
  const std::array<double, 4> & wheel_omega_radps,
  const std::vector<int> & left_idx, const std::vector<int> & right_idx,
  double wheel_radius_m, double track_width_m)
{
  BodyTwist t;
  if (wheel_radius_m <= 0.0 || track_width_m <= 0.0) {
    return t;  // invalid geometry -> zero (caller treats as fault)
  }
  const double v_left = mean_of(wheel_omega_radps, left_idx) * wheel_radius_m;
  const double v_right = mean_of(wheel_omega_radps, right_idx) * wheel_radius_m;
  t.vx = 0.5 * (v_left + v_right);
  t.wz = (v_right - v_left) / track_width_m;
  return t;
}

// 2D pose, integrated from a body twist over dt (exact arc for constant wz).
struct Pose2D
{
  double x {0.0};
  double y {0.0};
  double theta {0.0};
};

inline void integrate_pose(Pose2D & pose, const BodyTwist & t, double dt_s)
{
  if (dt_s <= 0.0) {
    return;
  }
  if (std::abs(t.wz) < 1e-9) {
    pose.x += t.vx * std::cos(pose.theta) * dt_s;
    pose.y += t.vx * std::sin(pose.theta) * dt_s;
  } else {
    const double dtheta = t.wz * dt_s;
    const double radius = t.vx / t.wz;
    const double new_theta = pose.theta + dtheta;
    pose.x += radius * (std::sin(new_theta) - std::sin(pose.theta));
    pose.y -= radius * (std::cos(new_theta) - std::cos(pose.theta));
    pose.theta = new_theta;
  }
  // Normalize theta to [-pi, pi].
  while (pose.theta > kPi) {pose.theta -= kTwoPi;}
  while (pose.theta < -kPi) {pose.theta += kTwoPi;}
}

}  // namespace rov2_plugins

#endif  // ROV2_PLUGINS__WHEEL_ODOMETRY_HPP_
