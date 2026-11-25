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

// Pure motion logic of the driver: what to send for a twist, and how to integrate odometry
// from what the base reports. No ROS and no CAN here; the node only wires these functions.

#ifndef RANGER_BASE__MOTION_LOGIC_HPP_
#define RANGER_BASE__MOTION_LOGIC_HPP_

#include <cstdint>

namespace westonrobot
{
namespace motion_logic
{

// Mode numbers as sent to the base and reported by it (the MotionState constants).
// 3 is park on the Ranger Mini 3.0 and side slip on the Ranger Mini 1.0.
constexpr uint8_t kModeAckermann = 0;
constexpr uint8_t kModeOblique = 1;
constexpr uint8_t kModeSpin = 2;
constexpr uint8_t kModeThree = 3;

// Below this |angular.z| a twist with no sideways part is driven straight (as before).
constexpr double kMinAngularForArc = 1e-6;
// Upper bound on the inner steering angle of an arc before the Ackermann clamp (as before).
constexpr double kMaxArcInnerAngle = 40.0 * 3.14159265358979323846 / 180.0;

// Geometry limits, all taken from ranger_params.hpp by the node.
struct Limits
{
  double wheelbase;
  double track;
  double max_linear_speed;           // used only by the Mini 1.0 side-slip branch, as before
  double max_angular_speed;
  double max_steer_angle_parallel;
  double max_steer_angle_ackermann;
  double min_turn_radius;
  double max_round_angle;            // wheel angle the base uses in spin mode
  bool side_slip_mode;               // true only for the Mini 1.0
};

// Limits from one of the parameter structs in ranger_params.hpp.
template<typename Params>
Limits LimitsFromParams(bool side_slip_mode)
{
  return Limits{
    Params::wheelbase, Params::track, Params::max_linear_speed, Params::max_angular_speed,
    Params::max_steer_angle_parallel, Params::max_steer_angle_ackermann, Params::min_turn_radius,
    Params::max_round_angle, side_slip_mode};
}

// ---------------------------------------------------------------------------------------------
// Command path
// ---------------------------------------------------------------------------------------------

// Twist components the selected mode cannot realise; they are commanded as zero.
enum DroppedComponent : uint8_t
{
  kDropNone = 0,
  kDropLinearX = 1,
  kDropLinearY = 2,
  kDropAngularZ = 4,
};

// What a zero twist sends. Default: Ackermann with zero speed and straight wheels (as before).
// With keep_reported_mode, a zero twist keeps the base's reported oblique or spin mode with zero
// speed and the measured wheel angle, so the wheels do not swing back at a stop. The inputs are
// the base's reported state, never an earlier command.
struct ZeroCommandPolicy
{
  bool keep_reported_mode;
  uint8_t reported_mode;
  double measured_wheel_angle;
};

// The arguments of SetMotionMode and SetMotionCommand for one twist.
struct MotionCommand
{
  bool valid;               // false: the twist is not finite and nothing is sent
  uint8_t mode;
  double speed;             // m/s, first argument of SetMotionCommand
  double steering;          // rad, second argument
  double angular;           // third argument: rad/s in spin mode (m/s in the Mini 1.0 side slip)
  uint8_t dropped;          // DroppedComponent bits
  bool steering_clamped;
};

// Stateless: the result depends on the twist, the limits and the zero policy only.
// Exact zeros of either sign are equal; tiny non-zero values are not rounded away.
//   vy != 0, vx != 0:  oblique, steering atan(vy / vx), speed sign(vx) * hypot(vx, vy);
//                      wz is dropped.
//   vy != 0, vx == 0:  oblique, steering +max_steer_angle_parallel, speed vy; wz is dropped.
//                      (The steering wraps here: a tiny x of the other sign asks for -90 degrees.)
//   vy == 0, |wz| < kMinAngularForArc:  Ackermann straight, speed vx; a non-zero wz is dropped.
//   vy == 0, |vx| / |wz| >= min_turn_radius:  Ackermann arc, inner angle atan((L / 2) / R)
//                      signed by wz * vx, clamped to max_steer_angle_ackermann; speed vx.
//   vy == 0, |vx| / |wz| < min_turn_radius:  spin at wz, clamped to max_angular_speed;
//                      vx is dropped.
//   all zero:  Ackermann with speed 0 and steering 0, unless the zero policy keeps the mode.
//   not finite:  rejected (valid == false), nothing is sent.
MotionCommand CommandFromTwist(
  double vx, double vy, double wz, const Limits & limits,
  const ZeroCommandPolicy & zero_policy);

// Where commands go. The node implements it with the SDK; tests use a recording fake.
class CommandSink
{
public:
  virtual ~CommandSink() = default;
  virtual void SetMotionMode(uint8_t mode) = 0;
  virtual void SetMotionCommand(double linear, double steering, double angular) = 0;
};

// A valid command gives exactly one mode frame and one motion frame; an invalid one gives none.
void DispatchCommand(const MotionCommand & command, CommandSink & sink);

// ---------------------------------------------------------------------------------------------
// Odometry
// ---------------------------------------------------------------------------------------------

// Geometry the odometry integrates with in one cycle.
enum class Geometry : uint8_t
{
  kAckermann = 0,
  kOblique = 1,
  kSpin = 2,
  kNone = 3,   // park or an unknown mode: nothing is integrated
};

// Which reported values a cycle used.
enum class InputSource : uint8_t
{
  kMotionState = 0,   // the base's motion-state frame (speed, steering, spin rate)
  kWheels = 1,        // the measured wheel angle and wheel speed (actuator frames)
  kNothing = 2,
};

// One feedback cycle as the base reports it.
struct FeedbackSample
{
  uint8_t reported_mode;
  bool mode_changing;          // published by the node; not used by the integration rule
  double linear_velocity;      // m/s, motion-state frame
  double angular_velocity;     // rad/s, motion-state frame
  double steering_angle;       // rad, motion-state frame (inner angle in Ackermann mode)
  double wheel_angle;          // rad, measured steering angle of one wheel
  double wheel_speed;          // m/s, measured speed of the same wheel
};

struct OdometryStep
{
  Geometry geometry;
  InputSource source;
  double x;
  double y;
  double theta;
  double vx;       // body-frame twist published on /odom
  double vy;
  double wz;
};

// A wheel is "at the spin angle" when |wheel angle| lies in a band centred on max_round_angle
// (about 0.936 rad) whose lower edge is kSpinAngleMin; the Ackermann limit is 0.601 rad, the
// oblique limit 1.570 rad. With these values the band is [0.8, 1.071] rad.
constexpr double kSpinAngleMin = 0.8;
// Longest a held geometry may be used while the reported mode disagrees with the wheels.
// The longest measured swing ended 0.94 s after the mode report.
constexpr double kMaxHoldSeconds = 1.5;

double SpinRadius(const Limits & limits);

// The one place that turns a measured wheel speed into a spin rate (spin coast after a stop).
// Sign verified for an anticlockwise spin only: the recorded wheel speed was positive while the
// gyro showed a positive yaw rate. The clockwise sign is UNVERIFIED (not yet measured).
double SpinRateFromWheelSpeed(double wheel_speed, const Limits & limits);

// Whether a measured wheel angle is possible in a geometry.
bool AngleFitsGeometry(Geometry geometry, double wheel_angle, const Limits & limits);

Geometry GeometryOfReportedMode(uint8_t reported_mode);

double InnerToCentralAngle(double inner, const Limits & limits);

// Integrates odometry from what the base reports, never from what was commanded.
//
// Each cycle, with R the reported mode and a the measured wheel angle:
//  1. R is park (3) or unknown: nothing is integrated. On the Ranger Mini 1.0 (side_slip_mode),
//     3 is side slip and is integrated as before: oblique at +90 degrees on the reported speed.
//  2. a fits R's geometry: integrate R's geometry with the motion-state values (the model used
//     before this change, so steady motion is unchanged).
//  3. Otherwise a hold applies. It starts with the geometry integrated in the previous cycle and
//     is tied to the reported mode at its start; a new reported mode ends it and starts a new one.
//     a) while a still fits the held geometry, for at most kMaxHoldSeconds: integrate it with the
//        measured wheel values;
//     b) otherwise: integrate R's geometry with the motion-state values (the model used before
//        this change), so odometry never stops while the base reports a mode. Once a has left the
//        held geometry, the held geometry is not resumed within the same hold.
class OdometryIntegrator
{
public:
  explicit OdometryIntegrator(const Limits & limits);

  void Reset(double x, double y, double theta);
  OdometryStep Step(const FeedbackSample & sample, double dt);

private:
  Limits limits_;
  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
  Geometry previous_geometry_ = Geometry::kAckermann;
  bool holding_ = false;
  Geometry held_geometry_ = Geometry::kNone;
  uint8_t hold_reported_mode_ = kModeAckermann;
  double hold_elapsed_ = 0.0;
};

}  // namespace motion_logic
}  // namespace westonrobot

#endif  // RANGER_BASE__MOTION_LOGIC_HPP_
