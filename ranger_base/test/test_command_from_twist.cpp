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

// INV-1, INV-4 (dispatch) and INV-6 for the twist-to-command function.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>
#include <vector>

#include "ranger_base/motion_logic.hpp"
#include "ranger_base/ranger_params.hpp"

namespace ml = westonrobot::motion_logic;

namespace
{

const ml::Limits kV3 = ml::LimitsFromParams<westonrobot::RangerMiniV3Params>(false);
const ml::ZeroCommandPolicy kDefaultZero{false, 0, 0.0};

ml::MotionCommand Cmd(double vx, double vy, double wz)
{
  return ml::CommandFromTwist(vx, vy, wz, kV3, kDefaultZero);
}

// The value as it lands in the CAN frame (the SDK multiplies the float by 1000 and casts).
int Encode(double value)
{
  return static_cast<int16_t>(static_cast<float>(value) * 1000.0f);
}

// Motion the base performs for a command: (forward, left, turn), from the kinematic models.
struct Motion
{
  double x;
  double y;
  double w;
};

Motion Realised(const ml::MotionCommand & c, const ml::Limits & limits)
{
  switch (c.mode) {
    case ml::kModeAckermann: {
        const double central = ml::InnerToCentralAngle(c.steering, limits);
        return Motion{c.speed * std::cos(central), 0.0,
          2.0 * c.speed * std::sin(central) / limits.wheelbase};
      }
    case ml::kModeOblique:
      return Motion{c.speed * std::cos(c.steering), c.speed * std::sin(c.steering), 0.0};
    case ml::kModeSpin:
      return Motion{0.0, 0.0, c.angular};
    default:
      return Motion{std::nan(""), std::nan(""), std::nan("")};
  }
}

int Sign(double v)
{
  return (v > 0.0) - (v < 0.0);
}

// One component of INV-1: never opposite; zero where the mode drops it. A pure sideways move
// keeps a forward residue of |speed| * cos(max_steer_angle_parallel) because the steering limit
// (1.570 rad) is 0.8 mrad short of 90 degrees; that residue is allowed and documented.
void ExpectComponent(
  double twist, double realised, bool dropped, double residual_allowed, const char * what,
  double vx, double vy, double wz)
{
  SCOPED_TRACE(::testing::Message() << what << " for twist (" << vx << ", " << vy << ", " << wz <<
    ")");
  if (dropped) {
    EXPECT_EQ(realised, 0.0);
    return;
  }
  if (twist == 0.0) {
    EXPECT_LE(std::abs(realised), residual_allowed);
    return;
  }
  EXPECT_EQ(Sign(realised), Sign(twist));
}

std::vector<double> GridValues()
{
  return {-1.0, -0.05, -1e-9, -0.0, 0.0, 1e-9, 0.05, 1.0};
}

}  // namespace

TEST(CommandFromTwist, SignsMatchTwistOverGrid)
{
  for (double vx : GridValues()) {
    for (double vy : GridValues()) {
      for (double wz : GridValues()) {
        const ml::MotionCommand c = Cmd(vx, vy, wz);
        ASSERT_TRUE(c.valid);
        SCOPED_TRACE(::testing::Message() << "(" << vx << ", " << vy << ", " << wz << ")");
        // The mode follows the case table, and exactly the components that mode cannot
        // realise are dropped: oblique only the turn, spin only the forward part, Ackermann only
        // a turn below kMinAngularForArc.
        uint8_t mode = ml::kModeAckermann;
        uint8_t dropped = ml::kDropNone;
        const bool any = vx != 0.0 || vy != 0.0 || wz != 0.0;
        if (vy != 0.0) {
          mode = ml::kModeOblique;
          dropped = wz != 0.0 ? ml::kDropAngularZ : ml::kDropNone;
        } else if (std::abs(wz) >= ml::kMinAngularForArc &&
          std::abs(vx) / std::abs(wz) < kV3.min_turn_radius)
        {
          mode = ml::kModeSpin;
          dropped = vx != 0.0 ? ml::kDropLinearX : ml::kDropNone;
        } else if (any && wz != 0.0 && std::abs(wz) < ml::kMinAngularForArc) {
          dropped = ml::kDropAngularZ;
        }
        EXPECT_EQ(c.mode, mode);
        EXPECT_EQ(c.dropped, dropped);
        const Motion m = Realised(c, kV3);
        const double residue = c.mode == ml::kModeOblique ?
          std::abs(c.speed) * std::cos(kV3.max_steer_angle_parallel) + 1e-15 : 0.0;
        ExpectComponent(vx, m.x, c.dropped & ml::kDropLinearX, residue, "forward", vx, vy, wz);
        ExpectComponent(vy, m.y, c.dropped & ml::kDropLinearY, 0.0, "sideways", vx, vy, wz);
        ExpectComponent(wz, m.w, c.dropped & ml::kDropAngularZ, 0.0, "turn", vx, vy, wz);
      }
    }
  }
}

TEST(CommandFromTwist, ZerosOfEitherSignGiveTheSameCommand)
{
  for (double vx : GridValues()) {
    for (double vy : GridValues()) {
      for (double wz : GridValues()) {
        const ml::MotionCommand a = Cmd(vx, vy, wz);
        const ml::MotionCommand b = Cmd(
          vx == 0.0 ? 0.0 : vx, vy == 0.0 ? 0.0 : vy, wz == 0.0 ? 0.0 : wz);
        EXPECT_EQ(a.mode, b.mode);
        EXPECT_EQ(a.speed, b.speed);
        EXPECT_EQ(a.steering, b.steering);
        EXPECT_EQ(a.angular, b.angular);
        EXPECT_EQ(a.dropped, b.dropped);
        EXPECT_EQ(Encode(a.speed), Encode(b.speed));
      }
    }
  }
}

TEST(CommandFromTwist, AllZeroTwistIsAckermannStop)
{
  for (double vx : {-0.0, 0.0}) {
    for (double vy : {-0.0, 0.0}) {
      for (double wz : {-0.0, 0.0}) {
        const ml::MotionCommand c = Cmd(vx, vy, wz);
        EXPECT_TRUE(c.valid);
        EXPECT_EQ(c.mode, ml::kModeAckermann);
        EXPECT_EQ(c.speed, 0.0);
        EXPECT_EQ(c.steering, 0.0);
        EXPECT_EQ(c.angular, 0.0);
        EXPECT_EQ(c.dropped, ml::kDropNone);
      }
    }
  }
}

TEST(CommandFromTwist, NoResultDependsOnEarlierCommands)
{
  const std::vector<std::tuple<double, double, double>> twists = {
    {-0.05, 0.05, 0.0}, {0.0, 0.05, 0.0}, {0.05, -0.05, 0.0}, {0.0, -0.05, 0.0},
    {0.0, 0.0, 0.2}, {-0.05, -0.05, 0.0}, {0.05, 0.0, 0.08}, {0.0, 0.0, 0.0}};
  std::vector<ml::MotionCommand> first;
  for (const auto & t : twists) {
    first.push_back(Cmd(std::get<0>(t), std::get<1>(t), std::get<2>(t)));
  }
  // Same twists again, in reverse order and interleaved with others: same results.
  for (size_t i = twists.size(); i-- > 0; ) {
    Cmd(-1.0, 1.0, 0.0);
    const auto & t = twists[i];
    const ml::MotionCommand c = Cmd(std::get<0>(t), std::get<1>(t), std::get<2>(t));
    EXPECT_EQ(c.mode, first[i].mode);
    EXPECT_EQ(c.speed, first[i].speed);
    EXPECT_EQ(c.steering, first[i].steering);
    EXPECT_EQ(c.angular, first[i].angular);
  }
}

// One row per sign case, as encoded in the frame. "seen" rows are frames that appear in a
// recording of the real base; the others are unverified until the hardware test list is run.
TEST(CommandFromTwist, SignCaseTable)
{
  struct Row
  {
    double vx, vy, wz;
    uint8_t mode;
    int steer_mrad, speed_mm_s, angular_mrad_s;
  };
  const std::vector<Row> rows = {
    {0.05, 0.0, 0.0, 0, 0, 50, 0},           // forward: seen (A)
    {-0.05, 0.0, 0.0, 0, 0, -50, 0},         // backward: seen (C)
    {0.05, 0.05, 0.0, 1, 785, 70, 0},        // forward-left: seen (D2)
    {-0.05, -0.05, 0.0, 1, 785, -70, 0},     // backward-right: the frame D sent, seen going there
    {-0.05, 0.05, 0.0, 1, -785, -70, 0},     // backward-left: unverified (negative angle)
    {0.05, -0.05, 0.0, 1, -785, 70, 0},      // forward-right: unverified (negative angle)
    {0.0, 0.05, 0.0, 1, 1570, 50, 0},        // pure left: seen (B, C, C2, y stops)
    {0.0, -0.05, 0.0, 1, 1570, -50, 0},      // pure right: unverified
    {-0.001, 0.05, 0.0, 1, -1550, -50, 0},   // near-sideways left, x of the other sign: unverified
    {0.001, -0.05, 0.0, 1, -1550, 50, 0},    // near-sideways right, x of the other sign: unverified
    {0.0, 0.0, 0.2, 2, 0, 0, 200},           // spin anticlockwise: seen (E)
    {0.0, 0.0, -0.2, 2, 0, 0, -200},         // spin clockwise: unverified
    {0.05, 0.0, 0.08, 0, 376, 50, 0},        // arc forward-left: unverified
    {0.05, 0.0, -0.08, 0, -376, 50, 0},      // arc forward-right: unverified
    {-0.05, 0.0, 0.08, 0, -376, -50, 0},     // arc backward, turning left: unverified
    {-0.05, 0.0, -0.08, 0, 376, -50, 0},     // arc backward, turning right: unverified
  };
  for (const Row & r : rows) {
    SCOPED_TRACE(::testing::Message() << "(" << r.vx << ", " << r.vy << ", " << r.wz << ")");
    const ml::MotionCommand c = Cmd(r.vx, r.vy, r.wz);
    EXPECT_EQ(c.mode, r.mode);
    EXPECT_EQ(Encode(c.steering), r.steer_mrad);
    EXPECT_EQ(Encode(c.speed), r.speed_mm_s);
    EXPECT_EQ(Encode(c.angular), r.angular_mrad_s);
  }
}

// The steering range wraps at pure sideways: a sideways twist with a tiny x of one sign or the
// other asks for wheels at +90 or -90 degrees (a half turn). No dead band hides it.
TEST(CommandFromTwist, SteeringWrapsAtPureSideways)
{
  EXPECT_EQ(Encode(Cmd(1e-6, 0.05, 0.0).steering), 1570);
  EXPECT_EQ(Encode(Cmd(0.0, 0.05, 0.0).steering), 1570);
  EXPECT_EQ(Encode(Cmd(-1e-6, 0.05, 0.0).steering), -1570);
  EXPECT_EQ(Encode(Cmd(-1e-6, -0.05, 0.0).steering), 1570);
  EXPECT_EQ(Encode(Cmd(0.0, -0.05, 0.0).steering), 1570);
  EXPECT_EQ(Encode(Cmd(1e-6, -0.05, 0.0).steering), -1570);
}

TEST(CommandFromTwist, DroppedComponentPerCase)
{
  // oblique: the turn is dropped
  EXPECT_EQ(Cmd(0.05, 0.05, 0.2).dropped, ml::kDropAngularZ);
  EXPECT_EQ(Cmd(0.0, 0.05, 0.3).dropped, ml::kDropAngularZ);
  // radius 0.25 m < 0.4764 m: spin, the forward part is dropped
  const ml::MotionCommand spin = Cmd(0.05, 0.0, 0.2);
  EXPECT_EQ(spin.mode, ml::kModeSpin);
  EXPECT_EQ(spin.dropped, ml::kDropLinearX);
  // |wz| below 1e-6: straight, the turn is dropped
  EXPECT_EQ(Cmd(0.05, 0.0, 1e-7).dropped, ml::kDropAngularZ);
  // arc at radius 0.625 m: nothing dropped
  EXPECT_EQ(Cmd(0.05, 0.0, 0.08).dropped, ml::kDropNone);
  EXPECT_EQ(Cmd(0.05, 0.05, 0.0).dropped, ml::kDropNone);
}

TEST(CommandFromTwist, AckermannClampNeverEngagesForTheMini3)
{
  // Below min_turn_radius the twist goes to spin, so the arc's inner angle never exceeds
  // atan((wheelbase / 2) / min_turn_radius) = 0.478 rad, under the 0.601 rad clamp.
  for (double radius = kV3.min_turn_radius * 1.000001; radius < 50.0; radius *= 1.01) {
    for (double v : {-0.5, 0.5}) {
      const ml::MotionCommand c = Cmd(v, 0.0, std::abs(v) / radius);
      ASSERT_EQ(c.mode, ml::kModeAckermann);
      EXPECT_FALSE(c.steering_clamped);
    }
  }
}

TEST(CommandFromTwist, ClampedArcKeepsItsSigns)
{
  ml::Limits narrow = kV3;
  narrow.max_steer_angle_ackermann = 0.2;
  for (double vx : {-0.5, 0.5}) {
    for (double wz : {-0.9, 0.9}) {
      const ml::MotionCommand c = ml::CommandFromTwist(vx, 0.0, wz, narrow, kDefaultZero);
      ASSERT_EQ(c.mode, ml::kModeAckermann);
      EXPECT_TRUE(c.steering_clamped);
      EXPECT_EQ(std::abs(c.steering), 0.2);
      EXPECT_EQ(c.dropped, ml::kDropNone);
      const Motion m = Realised(c, narrow);
      EXPECT_EQ(Sign(m.x), Sign(vx));
      EXPECT_EQ(Sign(m.w), Sign(wz));
    }
  }
}

TEST(CommandFromTwist, LimitsAreNotWidened)
{
  const std::vector<double> values = {-100.0, -5.0, -1.5, -0.3, -1e-9, 0.0, 1e-9, 0.3, 1.5, 5.0,
    100.0};
  for (double vx : values) {
    for (double vy : values) {
      for (double wz : values) {
        const ml::MotionCommand c = Cmd(vx, vy, wz);
        ASSERT_TRUE(c.valid);
        switch (c.mode) {
          case ml::kModeAckermann:
            EXPECT_LE(std::abs(c.steering), kV3.max_steer_angle_ackermann);
            EXPECT_EQ(c.angular, 0.0);
            EXPECT_EQ(c.speed, vx);
            break;
          case ml::kModeOblique:
            EXPECT_LE(std::abs(c.steering), kV3.max_steer_angle_parallel);
            EXPECT_LE(std::abs(c.speed), std::hypot(vx, vy));
            EXPECT_EQ(c.angular, 0.0);
            break;
          case ml::kModeSpin:
            EXPECT_LE(std::abs(c.angular), kV3.max_angular_speed);
            EXPECT_EQ(c.speed, 0.0);
            EXPECT_EQ(c.steering, 0.0);
            break;
          default:
            ADD_FAILURE() << "unexpected mode " << static_cast<int>(c.mode);
        }
      }
    }
  }
}

TEST(CommandFromTwist, NonFiniteTwistIsRejected)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (double bad : {nan, inf, -inf}) {
    EXPECT_FALSE(Cmd(bad, 0.0, 0.0).valid);
    EXPECT_FALSE(Cmd(0.0, bad, 0.0).valid);
    EXPECT_FALSE(Cmd(0.0, 0.0, bad).valid);
  }
}

TEST(CommandFromTwist, ZeroPolicyDefaultIsTodaysStop)
{
  for (uint8_t mode : {0, 1, 2, 3}) {
    const ml::MotionCommand c =
      ml::CommandFromTwist(0.0, 0.0, 0.0, kV3, ml::ZeroCommandPolicy{false, mode, 1.569});
    EXPECT_EQ(c.mode, ml::kModeAckermann);
    EXPECT_EQ(c.steering, 0.0);
  }
}

TEST(CommandFromTwist, ZeroPolicyKeepsReportedObliqueOrSpin)
{
  const ml::MotionCommand oblique =
    ml::CommandFromTwist(0.0, -0.0, 0.0, kV3, ml::ZeroCommandPolicy{true, 1, 1.569});
  EXPECT_EQ(oblique.mode, ml::kModeOblique);
  EXPECT_EQ(oblique.speed, 0.0);
  EXPECT_EQ(oblique.steering, 1.569);
  const ml::MotionCommand clamped =
    ml::CommandFromTwist(0.0, 0.0, 0.0, kV3, ml::ZeroCommandPolicy{true, 1, 1.6});
  EXPECT_EQ(clamped.steering, kV3.max_steer_angle_parallel);
  const ml::MotionCommand spin =
    ml::CommandFromTwist(0.0, 0.0, 0.0, kV3, ml::ZeroCommandPolicy{true, 2, 0.935});
  EXPECT_EQ(spin.mode, ml::kModeSpin);
  EXPECT_EQ(spin.angular, 0.0);
  for (uint8_t mode : {0, 3, 7}) {
    EXPECT_EQ(
      ml::CommandFromTwist(0.0, 0.0, 0.0, kV3, ml::ZeroCommandPolicy{true, mode, 0.3}).mode,
      ml::kModeAckermann);
  }
  const ml::MotionCommand no_angle = ml::CommandFromTwist(
    0.0, 0.0, 0.0, kV3, ml::ZeroCommandPolicy{true, 1, std::nan("")});
  EXPECT_EQ(no_angle.mode, ml::kModeAckermann);
  // A non-zero twist ignores the policy.
  const ml::MotionCommand moving =
    ml::CommandFromTwist(0.05, 0.0, 0.0, kV3, ml::ZeroCommandPolicy{true, 1, 1.569});
  EXPECT_EQ(moving.mode, ml::kModeAckermann);
  EXPECT_EQ(moving.steering, 0.0);
}

TEST(CommandFromTwist, MiniV1KeepsItsSideSlipBranch)
{
  const ml::Limits v1 = ml::LimitsFromParams<westonrobot::RangerMiniV1Params>(true);
  const ml::MotionCommand c = ml::CommandFromTwist(0.0, 0.3, 0.0, v1, kDefaultZero);
  EXPECT_EQ(c.mode, ml::kModeThree);
  EXPECT_EQ(c.speed, 0.0);
  EXPECT_EQ(c.angular, 0.3);
  EXPECT_EQ(ml::CommandFromTwist(0.1, 0.3, 0.0, v1, kDefaultZero).mode, ml::kModeOblique);
}

namespace
{

class RecordingSink : public ml::CommandSink
{
public:
  struct Call
  {
    char kind;
    double a, b, c;
  };
  void SetMotionMode(uint8_t mode) override
  {
    calls.push_back({'m', static_cast<double>(mode), 0.0, 0.0});
  }
  void SetMotionCommand(double linear, double steering, double angular) override
  {
    calls.push_back({'c', linear, steering, angular});
  }
  std::vector<Call> calls;
};

}  // namespace

TEST(DispatchCommand, OneTwistGivesOneModeFrameAndOneMotionFrame)
{
  RecordingSink sink;
  ml::DispatchCommand(Cmd(-0.05, 0.05, 0.0), sink);
  ASSERT_EQ(sink.calls.size(), 2u);
  EXPECT_EQ(sink.calls[0].kind, 'm');
  EXPECT_EQ(sink.calls[0].a, 1.0);
  EXPECT_EQ(sink.calls[1].kind, 'c');
  EXPECT_EQ(Encode(sink.calls[1].a), -70);
  EXPECT_EQ(Encode(sink.calls[1].b), -785);
  EXPECT_EQ(sink.calls[1].c, 0.0);
}

TEST(DispatchCommand, RejectedTwistSendsNothing)
{
  RecordingSink sink;
  ml::DispatchCommand(Cmd(std::nan(""), 0.0, 0.0), sink);
  EXPECT_TRUE(sink.calls.empty());
}

TEST(DispatchCommand, NothingIsRepeated)
{
  RecordingSink sink;
  for (int i = 0; i < 10; ++i) {
    ml::DispatchCommand(Cmd(0.0, 0.0, 0.0), sink);
  }
  EXPECT_EQ(sink.calls.size(), 20u);
}
