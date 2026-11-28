// Copyright 2026 ranger_ros2 contributors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the ranger_ros2 contributors nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Rule tests for the odometry integrator (synthetic sequences; the replay of real recordings is
// in test_replay_recordings.cpp).

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "ranger_base/kinematics_model.hpp"
#include "ranger_base/motion_logic.hpp"
#include "ranger_base/ranger_params.hpp"

namespace ml = westonrobot::motion_logic;

namespace
{

const ml::Limits kV3 = ml::LimitsFromParams<westonrobot::RangerMiniV3Params>(false);
constexpr double kDt = 0.02;

ml::FeedbackSample Sample(
  uint8_t mode, double linear, double steering, double angular, double wheel_angle,
  double wheel_speed)
{
  return ml::FeedbackSample{mode, false, linear, angular, steering, wheel_angle, wheel_speed};
}

template<typename Model>
std::vector<double> Direct(const Model & model, std::vector<double> state, double dt)
{
  boost::numeric::odeint::integrate_const(
    boost::numeric::odeint::runge_kutta4<std::vector<double>>(), model, state, 0.0, dt,
    dt / 10.0);
  return state;
}

}  // namespace

TEST(OdometryIntegrator, SteadyModesMatchThePreviousModelsExactly)
{
  ml::OdometryIntegrator odom(kV3);
  odom.Reset(1.0, 2.0, 0.3);
  const ml::OdometryStep a = odom.Step(Sample(0, 0.3, 0.2, 0.0, 0.2, 0.3), kDt);
  EXPECT_EQ(a.geometry, ml::Geometry::kAckermann);
  EXPECT_EQ(a.source, ml::InputSource::kMotionState);
  const double central = ml::InnerToCentralAngle(0.2, kV3);
  const auto expect_a = Direct(
    westonrobot::DualAckermanModel(
      kV3.wheelbase, westonrobot::DualAckermanModel::control_type{0.3, central}),
    {1.0, 2.0, 0.3}, kDt);
  EXPECT_EQ(a.x, expect_a[0]);
  EXPECT_EQ(a.y, expect_a[1]);
  EXPECT_EQ(a.theta, expect_a[2]);
  EXPECT_EQ(a.vx, 0.3);
  EXPECT_EQ(a.wz, 2.0 * 0.3 * std::sin(central) / kV3.wheelbase);

  const ml::OdometryStep p = odom.Step(Sample(1, 0.1, 1.57, 0.0, 1.57, 0.1), kDt);
  EXPECT_EQ(p.geometry, ml::Geometry::kOblique);
  const auto expect_p = Direct(
    westonrobot::ParallelModel(westonrobot::ParallelModel::control_type{0.1, 1.57}), expect_a,
    kDt);
  EXPECT_EQ(p.x, expect_p[0]);
  EXPECT_EQ(p.y, expect_p[1]);
  EXPECT_EQ(p.theta, expect_p[2]);
  EXPECT_EQ(p.vy, 0.1 * std::sin(1.57));

  const ml::OdometryStep s = odom.Step(Sample(2, 0.0, 0.0, 0.25, 0.935, 0.07), kDt);
  EXPECT_EQ(s.geometry, ml::Geometry::kSpin);
  EXPECT_EQ(s.source, ml::InputSource::kMotionState);
  const auto expect_s = Direct(
    westonrobot::SpinningModel(westonrobot::SpinningModel::control_type{0.25}), expect_p, kDt);
  EXPECT_EQ(s.theta, expect_s[2]);
  EXPECT_EQ(s.wz, 0.25);
}

// The 0.20 m/s sideways stop: the firmware still reports oblique while the zero command is in,
// then reports Ackermann with the wheels at 90 degrees while the base coasts sideways.
TEST(OdometryIntegrator, SidewaysStopHasNoYawAndCoastsSideways)
{
  ml::OdometryIntegrator odom(kV3);
  for (int i = 0; i < 50; ++i) {
    odom.Step(Sample(1, 0.2, 1.57, 0.0, 1.57, 0.2), kDt);
  }
  // still reported oblique after the zero (0.33 s in the recording): plain oblique
  for (int i = 0; i < 16; ++i) {
    const ml::OdometryStep s = odom.Step(Sample(1, 0.15, 1.57, 0.0, 1.57, 0.15), kDt);
    EXPECT_EQ(s.geometry, ml::Geometry::kOblique);
  }
  const double y_before = odom.Step(Sample(1, 0.12, 1.57, 0.0, 1.57, 0.12), kDt).y;
  // reported Ackermann, wheels swinging back from 90 degrees; the motion-state frame now claims
  // straight-ahead motion (steering 0), which must not be integrated.
  double angle = 1.57;
  ml::OdometryStep s{};
  while (angle > kV3.max_steer_angle_ackermann) {
    s = odom.Step(Sample(0, 0.1, 0.0, 0.0, angle, 0.1), kDt);
    EXPECT_EQ(s.geometry, ml::Geometry::kOblique);
    EXPECT_EQ(s.source, ml::InputSource::kWheels);
    EXPECT_EQ(s.theta, 0.0);
    EXPECT_GT(s.vy, 0.0);
    angle -= 0.05;
  }
  EXPECT_GT(s.y, y_before);
  // inside the Ackermann range: back to the reported mode with the motion-state values
  const ml::OdometryStep a = odom.Step(Sample(0, 0.0, 0.3, 0.0, 0.3, 0.01), kDt);
  EXPECT_EQ(a.geometry, ml::Geometry::kAckermann);
  EXPECT_EQ(a.source, ml::InputSource::kMotionState);
  EXPECT_EQ(a.theta, 0.0);
}

// A sideways swing-back passes through the spin angle; it stays oblique (history, not angle).
TEST(OdometryIntegrator, SidewaysSwingBackThroughSpinAngleStaysOblique)
{
  ml::OdometryIntegrator odom(kV3);
  odom.Step(Sample(1, 0.05, 1.57, 0.0, 1.57, 0.05), kDt);
  for (double angle : {1.4, 1.1, 0.95, 0.935, 0.9, 0.85, 0.8, 0.7}) {
    const ml::OdometryStep s = odom.Step(Sample(0, 0.05, 0.0, 0.0, angle, 0.05), kDt);
    EXPECT_EQ(s.geometry, ml::Geometry::kOblique) << angle;
    EXPECT_EQ(s.wz, 0.0);
  }
}

TEST(OdometryIntegrator, SpinRateFromWheelSpeedSignAndRadius)
{
  // Radius from ranger_params.hpp: hypot(wheelbase / 2, track / 2).
  EXPECT_DOUBLE_EQ(
    ml::SpinRadius(kV3),
    std::hypot(westonrobot::RangerMiniV3Params::wheelbase / 2.0,
    westonrobot::RangerMiniV3Params::track / 2.0));
  // Anticlockwise: positive wheel speed gives a positive spin rate (verified on the real base).
  EXPECT_GT(ml::SpinRateFromWheelSpeed(0.064, kV3), 0.0);
  EXPECT_DOUBLE_EQ(ml::SpinRateFromWheelSpeed(0.064, kV3), 0.064 / ml::SpinRadius(kV3));
  // Clockwise: UNVERIFIED on the real base (not yet measured); this only fixes the convention.
  EXPECT_LT(ml::SpinRateFromWheelSpeed(-0.064, kV3), 0.0);
}

// Spin stop: reported Ackermann at once, wheels still at the spin angle, body still turning.
TEST(OdometryIntegrator, SpinStopCoastsOnWheelSpeedThenReportedMode)
{
  ml::OdometryIntegrator odom(kV3);
  for (int i = 0; i < 20; ++i) {
    odom.Step(Sample(2, 0.0, 0.0, 0.2, 0.935, 0.0614), kDt);
  }
  const double theta0 = odom.Step(Sample(2, 0.0, 0.0, 0.2, 0.935, 0.0614), kDt).theta;
  // at the spin angle: spin from the wheel speed (motion state says linear 0)
  ml::OdometryStep s = odom.Step(Sample(0, 0.0, 0.0, 0.0, 0.935, 0.06), kDt);
  EXPECT_EQ(s.geometry, ml::Geometry::kSpin);
  EXPECT_EQ(s.source, ml::InputSource::kWheels);
  EXPECT_DOUBLE_EQ(s.wz, ml::SpinRateFromWheelSpeed(0.06, kV3));
  EXPECT_GT(s.theta, theta0);
  s = odom.Step(Sample(0, 0.0, 0.0, 0.0, 0.81, 0.05), kDt);
  EXPECT_EQ(s.geometry, ml::Geometry::kSpin);
  // between the spin band and the Ackermann range: the reported mode on the motion-state values
  const double x1 = s.x;
  s = odom.Step(Sample(0, 0.004, 0.0, 0.0, 0.72, 0.02), kDt);
  EXPECT_EQ(s.geometry, ml::Geometry::kAckermann);
  EXPECT_EQ(s.source, ml::InputSource::kMotionState);
  EXPECT_EQ(s.vx, 0.004);
  EXPECT_NE(s.x, x1);
  // the spin hold is not resumed within the same hold, even if the angle comes back to the band
  s = odom.Step(Sample(0, 0.0, 0.0, 0.0, 0.85, 0.02), kDt);
  EXPECT_EQ(s.geometry, ml::Geometry::kAckermann);
  EXPECT_EQ(s.source, ml::InputSource::kMotionState);
  // inside the Ackermann range: the reported mode with the motion-state values
  s = odom.Step(Sample(0, 0.002, 0.5, 0.0, 0.5, 0.01), kDt);
  EXPECT_EQ(s.geometry, ml::Geometry::kAckermann);
  EXPECT_EQ(s.source, ml::InputSource::kMotionState);
}

TEST(OdometryIntegrator, HoldEndsAfterTimeBound)
{
  ml::OdometryIntegrator odom(kV3);
  odom.Step(Sample(2, 0.0, 0.0, 0.2, 0.935, 0.06), kDt);
  // reported Ackermann with the wheels stuck at the spin angle
  int ticks = 0;
  ml::OdometryStep s{};
  do {
    s = odom.Step(Sample(0, 0.0, 0.0, 0.0, 0.935, 0.01), kDt);
    ++ticks;
  } while (s.geometry == ml::Geometry::kSpin && ticks < 1000);
  EXPECT_EQ(s.geometry, ml::Geometry::kAckermann);
  EXPECT_EQ(s.source, ml::InputSource::kMotionState);
  EXPECT_NEAR(ticks * kDt, ml::kMaxHoldSeconds, 1.5 * kDt);

  // an oblique hold with the wheels stuck at 90 degrees ends the same way
  ml::OdometryIntegrator odom2(kV3);
  odom2.Step(Sample(1, 0.05, 1.57, 0.0, 1.57, 0.05), kDt);
  ticks = 0;
  do {
    s = odom2.Step(Sample(0, 0.0, 0.0, 0.0, 1.57, 0.0), kDt);
    ++ticks;
  } while (s.geometry == ml::Geometry::kOblique && ticks < 1000);
  EXPECT_EQ(s.geometry, ml::Geometry::kAckermann);
  EXPECT_NEAR(ticks * kDt, ml::kMaxHoldSeconds, 1.5 * kDt);
}

TEST(OdometryIntegrator, NewReportedModeEndsTheHold)
{
  ml::OdometryIntegrator odom(kV3);
  odom.Step(Sample(2, 0.0, 0.0, 0.2, 0.935, 0.06), kDt);
  EXPECT_EQ(
    odom.Step(Sample(0, 0.0, 0.0, 0.0, 0.7, 0.0), kDt).geometry, ml::Geometry::kAckermann);
  // an oblique command arrives: oblique fits any angle, so the reported mode is used at once
  const ml::OdometryStep s = odom.Step(Sample(1, 0.01, 0.7, 0.0, 0.7, 0.01), kDt);
  EXPECT_EQ(s.geometry, ml::Geometry::kOblique);
  EXPECT_EQ(s.source, ml::InputSource::kMotionState);
}

// Mode changes commanded while the base is still rolling (in no recording; unit tests only).
TEST(OdometryIntegrator, ModeChangeWhileRolling)
{
  {
    // Ackermann -> oblique: oblique fits any wheel angle, used at once
    ml::OdometryIntegrator odom(kV3);
    odom.Step(Sample(0, 0.5, 0.3, 0.0, 0.3, 0.5), kDt);
    const ml::OdometryStep s = odom.Step(Sample(1, 0.5, 0.3, 0.0, 0.3, 0.5), kDt);
    EXPECT_EQ(s.geometry, ml::Geometry::kOblique);
    EXPECT_EQ(s.source, ml::InputSource::kMotionState);
  }
  {
    // Ackermann -> spin: Ackermann on the wheel values until the wheels leave its range, then
    // spin on the motion-state rate
    ml::OdometryIntegrator odom(kV3);
    odom.Step(Sample(0, 0.5, 0.3, 0.0, 0.3, 0.5), kDt);
    ml::OdometryStep s = odom.Step(Sample(2, 0.0, 0.0, 0.1, 0.4, 0.4), kDt);
    EXPECT_EQ(s.geometry, ml::Geometry::kAckermann);
    EXPECT_EQ(s.source, ml::InputSource::kWheels);
    EXPECT_EQ(s.vx, 0.4);
    s = odom.Step(Sample(2, 0.0, 0.0, 0.1, 0.7, 0.2), kDt);
    EXPECT_EQ(s.geometry, ml::Geometry::kSpin);
    EXPECT_EQ(s.source, ml::InputSource::kMotionState);
    s = odom.Step(Sample(2, 0.0, 0.0, 0.1, 0.9, 0.05), kDt);
    EXPECT_EQ(s.geometry, ml::Geometry::kSpin);
    EXPECT_EQ(s.source, ml::InputSource::kMotionState);
  }
  {
    // oblique at 90 degrees -> spin: oblique on the wheel values until the spin band
    ml::OdometryIntegrator odom(kV3);
    odom.Step(Sample(1, 0.1, 1.57, 0.0, 1.57, 0.1), kDt);
    ml::OdometryStep s = odom.Step(Sample(2, 0.0, 0.0, 0.0, 1.3, 0.08), kDt);
    EXPECT_EQ(s.geometry, ml::Geometry::kOblique);
    EXPECT_EQ(s.source, ml::InputSource::kWheels);
    s = odom.Step(Sample(2, 0.0, 0.0, 0.05, 1.0, 0.02), kDt);
    EXPECT_EQ(s.geometry, ml::Geometry::kSpin);
  }
  {
    // spin -> oblique while still turning: oblique at once on the motion-state values
    ml::OdometryIntegrator odom(kV3);
    odom.Step(Sample(2, 0.0, 0.0, 0.2, 0.935, 0.06), kDt);
    const ml::OdometryStep s = odom.Step(Sample(1, 0.02, 0.935, 0.0, 0.935, 0.06), kDt);
    EXPECT_EQ(s.geometry, ml::Geometry::kOblique);
  }
}

// Odometry never stops while the base reports a driving mode: for every previous geometry, every
// reported mode 0, 1, 2 and every wheel angle, something is integrated.
TEST(OdometryIntegrator, NeverStopsWhileAModeIsReported)
{
  const std::vector<ml::FeedbackSample> before = {
    Sample(0, 0.3, 0.2, 0.0, 0.2, 0.3), Sample(1, 0.1, 1.57, 0.0, 1.57, 0.1),
    Sample(2, 0.0, 0.0, 0.2, 0.935, 0.06)};
  for (const ml::FeedbackSample & first : before) {
    for (uint8_t mode = 0; mode <= 2; ++mode) {
      for (double angle = -1.6; angle <= 1.6; angle += 0.01) {
        ml::OdometryIntegrator odom(kV3);
        odom.Step(first, kDt);
        double previous_x = 0.0;
        double previous_y = 0.0;
        double previous_theta = 0.0;
        for (int k = 0; k < 100; ++k) {   // 2 s, past the hold time bound
          const ml::OdometryStep s = odom.Step(Sample(mode, 0.1, angle, 0.2, angle, 0.1), kDt);
          ASSERT_NE(s.geometry, ml::Geometry::kNone) << static_cast<int>(mode) << " " << angle;
          EXPECT_TRUE(s.x != previous_x || s.y != previous_y || s.theta != previous_theta);
          previous_x = s.x;
          previous_y = s.y;
          previous_theta = s.theta;
        }
      }
    }
  }
}

// The Ranger Mini 3.0's maximum Ackermann angle (0.698 rad) is above the driver's own limit; with
// the remote driving there, odometry keeps integrating every cycle.
TEST(OdometryIntegrator, AckermannBeyondTheDriverLimitStillIntegrates)
{
  ml::OdometryIntegrator odom(kV3);
  double previous_x = odom.Step(Sample(0, 0.3, 0.3, 0.0, 0.3, 0.3), kDt).x;
  for (int k = 0; k < 200; ++k) {
    const double angle = (k / 10) % 2 == 0 ? 0.65 : 0.59;   // crossing 0.601 back and forth
    const ml::OdometryStep s = odom.Step(Sample(0, 0.3, angle, 0.0, angle, 0.3), kDt);
    EXPECT_EQ(s.geometry, ml::Geometry::kAckermann);
    EXPECT_GT(s.vx, 0.0);
    EXPECT_NE(s.x, previous_x);
    previous_x = s.x;
  }
}

TEST(OdometryIntegrator, ParkIntegratesNothing)
{
  ml::OdometryIntegrator odom(kV3);
  const ml::OdometryStep s = odom.Step(Sample(3, 0.3, 1.57, 0.2, 1.57, 0.3), kDt);
  EXPECT_EQ(s.geometry, ml::Geometry::kNone);
  EXPECT_EQ(s.x, 0.0);
  EXPECT_EQ(s.y, 0.0);
  EXPECT_EQ(s.theta, 0.0);
  EXPECT_EQ(s.vy, 0.0);
  EXPECT_EQ(odom.Step(Sample(9, 0.3, 0.0, 0.0, 0.0, 0.3), kDt).geometry, ml::Geometry::kNone);
}

TEST(OdometryIntegrator, MiniV1SideSlipIsObliqueAtNinetyDegrees)
{
  const ml::Limits v1 = ml::LimitsFromParams<westonrobot::RangerMiniV1Params>(true);
  ml::OdometryIntegrator odom(v1);
  const ml::OdometryStep s = odom.Step(Sample(3, 0.2, 0.0, 0.0, 0.0, 0.2), kDt);
  EXPECT_EQ(s.geometry, ml::Geometry::kOblique);
  EXPECT_EQ(s.source, ml::InputSource::kMotionState);
  EXPECT_EQ(s.vy, 0.2 * std::sin(1.57079632679489661923));
  EXPECT_GT(s.y, 0.0);
  EXPECT_EQ(s.theta, 0.0);
  // On the Mini 3.0, 3 is park: nothing.
  ml::OdometryIntegrator v3(kV3);
  EXPECT_EQ(v3.Step(Sample(3, 0.2, 0.0, 0.0, 0.0, 0.2), kDt).geometry, ml::Geometry::kNone);
}

TEST(OdometryIntegrator, SwitchingFlagDoesNotChangeTheResult)
{
  ml::OdometryIntegrator a(kV3);
  ml::OdometryIntegrator b(kV3);
  const std::vector<ml::FeedbackSample> seq = {
    Sample(1, 0.1, 1.57, 0.0, 1.57, 0.1), Sample(0, 0.1, 0.0, 0.0, 1.5, 0.1),
    Sample(0, 0.05, 0.0, 0.0, 0.5, 0.05), Sample(2, 0.0, 0.0, 0.0, 0.2, 0.0)};
  for (ml::FeedbackSample s : seq) {
    const ml::OdometryStep ra = a.Step(s, kDt);
    s.mode_changing = true;
    const ml::OdometryStep rb = b.Step(s, kDt);
    EXPECT_EQ(ra.geometry, rb.geometry);
    EXPECT_EQ(ra.x, rb.x);
    EXPECT_EQ(ra.y, rb.y);
    EXPECT_EQ(ra.theta, rb.theta);
  }
}

TEST(OdometryIntegrator, NonPositiveDtOrNonFiniteInputsDoNotMoveThePose)
{
  ml::OdometryIntegrator odom(kV3);
  ml::OdometryStep s = odom.Step(Sample(0, 0.5, 0.2, 0.0, 0.2, 0.5), 0.0);
  EXPECT_EQ(s.x, 0.0);
  s = odom.Step(Sample(0, 0.5, 0.2, 0.0, 0.2, 0.5), -0.02);
  EXPECT_EQ(s.x, 0.0);
  s = odom.Step(Sample(0, std::nan(""), 0.2, 0.0, 0.2, 0.5), kDt);
  EXPECT_EQ(s.x, 0.0);
  EXPECT_EQ(s.vx, 0.0);
  s = odom.Step(Sample(1, 0.5, std::nan(""), 0.0, 0.2, 0.5), kDt);
  EXPECT_EQ(s.y, 0.0);
}

TEST(OdometryIntegrator, AngleFitsGeometry)
{
  EXPECT_TRUE(ml::AngleFitsGeometry(ml::Geometry::kAckermann, -0.601, kV3));
  EXPECT_FALSE(ml::AngleFitsGeometry(ml::Geometry::kAckermann, 0.62, kV3));
  EXPECT_TRUE(ml::AngleFitsGeometry(ml::Geometry::kOblique, 1.57, kV3));
  EXPECT_TRUE(ml::AngleFitsGeometry(ml::Geometry::kSpin, -0.935, kV3));
  EXPECT_TRUE(ml::AngleFitsGeometry(ml::Geometry::kSpin, 0.8, kV3));
  EXPECT_FALSE(ml::AngleFitsGeometry(ml::Geometry::kSpin, 0.79, kV3));
  EXPECT_FALSE(ml::AngleFitsGeometry(ml::Geometry::kSpin, 1.2, kV3));
  EXPECT_FALSE(ml::AngleFitsGeometry(ml::Geometry::kSpin, 1.57, kV3));
  EXPECT_FALSE(ml::AngleFitsGeometry(ml::Geometry::kNone, 0.0, kV3));
  EXPECT_FALSE(ml::AngleFitsGeometry(ml::Geometry::kOblique, std::nan(""), kV3));
}
