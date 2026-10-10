#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "rov2_plugins/wheel_odometry.hpp"

using rov2_plugins::WheelEncoderProcessor;
using rov2_plugins::WheelMotion;
using rov2_plugins::body_twist_from_wheels;
using rov2_plugins::encoder_delta;
using rov2_plugins::integrate_pose;
using rov2_plugins::BodyTwist;
using rov2_plugins::Pose2D;
using rov2_plugins::kPi;
using rov2_plugins::kTwoPi;

TEST(EncoderDelta, SimpleForwardAndBackward)
{
  EXPECT_EQ(encoder_delta(100, 150), 50);
  EXPECT_EQ(encoder_delta(150, 100), -50);
  EXPECT_EQ(encoder_delta(0, 0), 0);
}

TEST(EncoderDelta, WrapAroundIsShortestPath)
{
  // +1 across the 32-bit wrap should read as +1, not a huge negative jump.
  EXPECT_EQ(
    encoder_delta(
      std::numeric_limits<int32_t>::max(),
      std::numeric_limits<int32_t>::min()), 1);
  // -1 across the wrap.
  EXPECT_EQ(
    encoder_delta(
      std::numeric_limits<int32_t>::min(),
      std::numeric_limits<int32_t>::max()), -1);
  EXPECT_EQ(encoder_delta(-1, 0), 1);
  EXPECT_EQ(encoder_delta(0, -1), -1);
}

TEST(WheelEncoderProcessor, FirstSampleIsBaselineOnly)
{
  WheelEncoderProcessor p(1000.0, 20.0);
  const WheelMotion m = p.update({{0, 0, 0, 0}}, 0.0);
  EXPECT_FALSE(m.updated);
  for (double w : m.omega_radps) {
    EXPECT_DOUBLE_EQ(w, 0.0);
                                                           }
}

TEST(WheelEncoderProcessor, OneRevolutionPerSecond)
{
  WheelEncoderProcessor p(1000.0, 20.0);
  p.update({{0, 0, 0, 0}}, 0.0);
  const WheelMotion m = p.update({{1000, 1000, 1000, 1000}}, 1.0);
  ASSERT_TRUE(m.updated);
  EXPECT_NEAR(m.dt_s, 1.0, 1e-9);
  for (double w : m.omega_radps) {
    EXPECT_NEAR(w, kTwoPi, 1e-6);
                                                               }  // 1 rev/s
  for (double a : m.angle_rad) {
    EXPECT_NEAR(a, kTwoPi, 1e-6);
                                                             }
}

TEST(WheelEncoderProcessor, DuplicateTimestampIgnored)
{
  WheelEncoderProcessor p(1000.0, 20.0);
  p.update({{0, 0, 0, 0}}, 0.0);
  p.update({{500, 500, 500, 500}}, 1.0);
  const WheelMotion dup = p.update({{999, 999, 999, 999}}, 1.0);  // same stamp
  EXPECT_FALSE(dup.updated);
}

TEST(WheelEncoderProcessor, ImplausibleJumpRejected)
{
  // max 20 rev/s * 1000 cpr * 0.1 s = 2000 counts allowed per wheel.
  WheelEncoderProcessor p(1000.0, 20.0);
  p.update({{0, 0, 0, 0}}, 0.0);
  const WheelMotion m = p.update({{1000000, 10, 10, 10}}, 0.1);  // wheel 0 absurd
  ASSERT_TRUE(m.updated);
  EXPECT_DOUBLE_EQ(m.rejected[0], 1.0);
  EXPECT_NEAR(m.omega_radps[0], 0.0, 1e-9);     // rejected -> no motion
  EXPECT_GT(std::abs(m.omega_radps[1]), 0.0);   // others still counted
}

TEST(BodyTwist, StraightAheadHasNoYaw)
{
  const std::array<double, 4> omega {{10.0, 10.0, 10.0, 10.0}};
  const BodyTwist t = body_twist_from_wheels(omega, {0, 2}, {1, 3}, 0.03, 0.17);
  EXPECT_GT(t.vx, 0.0);
  EXPECT_NEAR(t.wz, 0.0, 1e-9);
}

TEST(BodyTwist, OppositeWheelsSpinInPlace)
{
  const std::array<double, 4> omega {{-10.0, 10.0, -10.0, 10.0}};
  const BodyTwist t = body_twist_from_wheels(omega, {0, 2}, {1, 3}, 0.03, 0.17);
  EXPECT_NEAR(t.vx, 0.0, 1e-9);
  EXPECT_GT(std::abs(t.wz), 0.0);
}

TEST(BodyTwist, InvalidGeometryYieldsZero)
{
  const std::array<double, 4> omega {{10.0, 10.0, 10.0, 10.0}};
  const BodyTwist t = body_twist_from_wheels(omega, {0, 2}, {1, 3}, 0.0, 0.17);
  EXPECT_DOUBLE_EQ(t.vx, 0.0);
  EXPECT_DOUBLE_EQ(t.wz, 0.0);
}

TEST(IntegratePose, StraightLine)
{
  Pose2D pose;
  BodyTwist t;
  t.vx = 1.0;
  t.wz = 0.0;
  integrate_pose(pose, t, 2.0);
  EXPECT_NEAR(pose.x, 2.0, 1e-9);
  EXPECT_NEAR(pose.y, 0.0, 1e-9);
  EXPECT_NEAR(pose.theta, 0.0, 1e-9);
}

TEST(IntegratePose, PureRotation)
{
  Pose2D pose;
  BodyTwist t;
  t.vx = 0.0;
  t.wz = kPi;  // half a turn per second
  integrate_pose(pose, t, 1.0);
  EXPECT_NEAR(std::abs(pose.theta), kPi, 1e-6);
  EXPECT_NEAR(pose.x, 0.0, 1e-9);
  EXPECT_NEAR(pose.y, 0.0, 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
