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

#include "ranger_base/motion_logic.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "ranger_base/kinematics_model.hpp"

namespace westonrobot
{
namespace motion_logic
{

namespace
{

double Clamp(double value, double limit, bool * clamped)
{
  if (value > limit) {
    *clamped = true;
    return limit;
  }
  if (value < -limit) {
    *clamped = true;
    return -limit;
  }
  return value;
}

bool AllFinite(double a, double b, double c)
{
  return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
}

MotionCommand ZeroCommand(const Limits & limits, const ZeroCommandPolicy & policy)
{
  MotionCommand command{true, kModeAckermann, 0.0, 0.0, 0.0, kDropNone, false};
  if (!policy.keep_reported_mode) {
    return command;
  }
  if (policy.reported_mode == kModeOblique && std::isfinite(policy.measured_wheel_angle)) {
    command.mode = kModeOblique;
    command.steering = Clamp(
      policy.measured_wheel_angle, limits.max_steer_angle_parallel, &command.steering_clamped);
  } else if (policy.reported_mode == kModeSpin) {
    command.mode = kModeSpin;
  }
  return command;
}

// Integrates one kinematic model over dt exactly as the driver always has:
// odeint runge_kutta4 with integrate_const and a step of dt / 10.
template<typename Model>
void Integrate(const Model & model, double dt, double * x, double * y, double * theta)
{
  std::vector<double> state = {*x, *y, *theta};
  boost::numeric::odeint::integrate_const(
    boost::numeric::odeint::runge_kutta4<std::vector<double>>(), model, state, 0.0, dt,
    dt / 10.0);
  *x = state[0];
  *y = state[1];
  *theta = state[2];
}

}  // namespace

MotionCommand CommandFromTwist(
  double vx, double vy, double wz, const Limits & limits,
  const ZeroCommandPolicy & zero_policy)
{
  MotionCommand command{false, kModeAckermann, 0.0, 0.0, 0.0, kDropNone, false};
  if (!AllFinite(vx, vy, wz)) {
    return command;
  }
  // Exact zeros of either sign compare equal to 0.0, so -0.0 and +0.0 behave the same everywhere.
  if (vx == 0.0 && vy == 0.0 && wz == 0.0) {
    return ZeroCommand(limits, zero_policy);
  }
  command.valid = true;

  if (vy != 0.0) {
    if (wz != 0.0) {
      command.dropped |= kDropAngularZ;
    }
    if (limits.side_slip_mode && vx == 0.0) {
      // Ranger Mini 1.0 only, unchanged: side-slip mode with the lateral speed in the third field.
      command.mode = kModeThree;
      command.angular = Clamp(vy, limits.max_linear_speed, &command.steering_clamped);
      return command;
    }
    command.mode = kModeOblique;
    if (vx != 0.0) {
      // Wheels along the direction of travel, angle in (-pi/2, pi/2); the speed carries the sign
      // of x. Motion = speed * (cos, sin)(steering) has the signs of (vx, vy).
      command.steering = std::atan(vy / vx);
      command.speed = std::copysign(std::hypot(vx, vy), vx);
    } else {
      // Pure sideways: wheels at +90 degrees (the parallel limit), speed = vy.
      command.steering = limits.max_steer_angle_parallel;
      command.speed = vy;
    }
    command.steering = Clamp(
      command.steering, limits.max_steer_angle_parallel, &command.steering_clamped);
    return command;
  }

  if (std::abs(wz) < kMinAngularForArc) {
    if (wz != 0.0) {
      command.dropped |= kDropAngularZ;
    }
    command.mode = kModeAckermann;
    command.speed = vx;
    return command;
  }

  const double radius = std::abs(vx) / std::abs(wz);
  if (radius < limits.min_turn_radius) {
    if (vx != 0.0) {
      command.dropped |= kDropLinearX;
    }
    command.mode = kModeSpin;
    command.angular = Clamp(wz, limits.max_angular_speed, &command.steering_clamped);
    return command;
  }

  // Ackermann arc, unchanged: inner wheel angle for the radius, signed by wz * vx so that the
  // turn has the sign of wz whether driving forward or backward.
  const double k = (wz * vx) >= 0.0 ? 1.0 : -1.0;
  double inner = std::atan((limits.wheelbase / 2.0) / radius);
  inner = std::min(inner, kMaxArcInnerAngle);
  command.mode = kModeAckermann;
  command.speed = vx;
  command.steering = Clamp(
    k * inner, limits.max_steer_angle_ackermann, &command.steering_clamped);
  return command;
}

void DispatchCommand(const MotionCommand & command, CommandSink & sink)
{
  if (!command.valid) {
    return;
  }
  sink.SetMotionMode(command.mode);
  sink.SetMotionCommand(command.speed, command.steering, command.angular);
}

double SpinRadius(const Limits & limits)
{
  return std::hypot(limits.wheelbase / 2.0, limits.track / 2.0);
}

double SpinRateFromWheelSpeed(double wheel_speed, const Limits & limits)
{
  // Positive wheel speed = anticlockwise (verified); clockwise unverified.
  return wheel_speed / SpinRadius(limits);
}

bool AngleFitsGeometry(Geometry geometry, double wheel_angle, const Limits & limits)
{
  if (!std::isfinite(wheel_angle)) {
    return false;
  }
  switch (geometry) {
    case Geometry::kAckermann:
      return std::abs(wheel_angle) <= limits.max_steer_angle_ackermann;
    case Geometry::kOblique:
      return true;
    case Geometry::kSpin:
      return std::abs(wheel_angle) >= kSpinAngleMin &&
             std::abs(wheel_angle) <= 2.0 * limits.max_round_angle - kSpinAngleMin;
    default:
      return false;
  }
}

Geometry GeometryOfReportedMode(uint8_t reported_mode)
{
  switch (reported_mode) {
    case kModeAckermann:
      return Geometry::kAckermann;
    case kModeOblique:
      return Geometry::kOblique;
    case kModeSpin:
      return Geometry::kSpin;
    default:
      return Geometry::kNone;   // 3 is park on the Ranger Mini 3.0: no motion
  }
}

double InnerToCentralAngle(double inner, const Limits & limits)
{
  const double phi_i = std::abs(inner);
  const double phi = std::atan(
    limits.wheelbase * std::sin(phi_i) /
    (limits.wheelbase * std::cos(phi_i) + limits.track * std::sin(phi_i)));
  return inner >= 0 ? phi : -phi;
}

OdometryIntegrator::OdometryIntegrator(const Limits & limits)
: limits_(limits)
{
}

void OdometryIntegrator::Reset(double x, double y, double theta)
{
  x_ = x;
  y_ = y;
  theta_ = theta;
  previous_geometry_ = Geometry::kAckermann;
  holding_ = false;
  held_geometry_ = Geometry::kNone;
  hold_reported_mode_ = kModeAckermann;
  hold_elapsed_ = 0.0;
}

OdometryStep OdometryIntegrator::Step(const FeedbackSample & sample, double dt)
{
  const Geometry reported = GeometryOfReportedMode(sample.reported_mode);
  Geometry geometry = Geometry::kNone;
  InputSource source = InputSource::kNothing;

  if (reported == Geometry::kNone) {
    holding_ = false;
  } else if (AngleFitsGeometry(reported, sample.wheel_angle, limits_)) {
    geometry = reported;
    source = InputSource::kMotionState;
    holding_ = false;
  } else {
    if (!holding_ || sample.reported_mode != hold_reported_mode_) {
      holding_ = true;
      held_geometry_ = previous_geometry_;
      hold_reported_mode_ = sample.reported_mode;
      hold_elapsed_ = 0.0;
    }
    if (dt > 0.0) {
      hold_elapsed_ += dt;
    }
    if (hold_elapsed_ <= kMaxHoldSeconds &&
      AngleFitsGeometry(held_geometry_, sample.wheel_angle, limits_))
    {
      geometry = held_geometry_;
      source = InputSource::kWheels;
    } else {
      // Time bound reached, or the wheels left the held geometry: integrate the reported mode on
      // the motion-state values, so odometry never stops while the base reports a mode.
      if (hold_elapsed_ <= kMaxHoldSeconds) {
        held_geometry_ = Geometry::kNone;
      }
      geometry = reported;
      source = InputSource::kMotionState;
    }
  }

  const bool wheels = source == InputSource::kWheels;
  const double speed = wheels ? sample.wheel_speed : sample.linear_velocity;
  const double angle = wheels ? sample.wheel_angle : sample.steering_angle;
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  const bool integrate = dt > 0.0;

  switch (geometry) {
    case Geometry::kAckermann: {
        const double central = InnerToCentralAngle(angle, limits_);
        if (std::isfinite(speed) && std::isfinite(central)) {
          if (integrate) {
            DualAckermanModel::control_type u{speed, central};
            Integrate(DualAckermanModel(limits_.wheelbase, u), dt, &x_, &y_, &theta_);
          }
          vx = speed;
          wz = 2.0 * speed * std::sin(central) / limits_.wheelbase;
        }
        break;
      }
    case Geometry::kOblique: {
        if (std::isfinite(speed) && std::isfinite(angle)) {
          if (integrate) {
            ParallelModel::control_type u{speed, angle};
            Integrate(ParallelModel(u), dt, &x_, &y_, &theta_);
          }
          vx = speed * std::cos(angle);
          vy = speed * std::sin(angle);
        }
        break;
      }
    case Geometry::kSpin: {
        const double rate = wheels ?
          SpinRateFromWheelSpeed(sample.wheel_speed, limits_) : sample.angular_velocity;
        if (std::isfinite(rate)) {
          if (integrate) {
            SpinningModel::control_type u{rate};
            Integrate(SpinningModel(u), dt, &x_, &y_, &theta_);
          }
          wz = rate;
        }
        break;
      }
    default:
      break;
  }

  previous_geometry_ = geometry;
  return OdometryStep{geometry, source, x_, y_, theta_, vx, vy, wz};
}

}  // namespace motion_logic
}  // namespace westonrobot
