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

// Reading the replay CSVs (feedback recovered from the real recordings) and running the
// odometry integrator over them. Shared by the replay test and the replay tool; test-only.

#ifndef REPLAY_IO_HPP_
#define REPLAY_IO_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "ranger_base/motion_logic.hpp"

namespace replay
{

namespace ml = westonrobot::motion_logic;

constexpr int kSubstLinear = 1;
constexpr int kSubstSteering = 2;
constexpr int kSubstAngular = 4;

struct Row
{
  int64_t t_ns;
  ml::FeedbackSample sample;
  int subst;
  double odom_yaw;
  double gyro_yaw;
  double gyro_rate;
  bool ref;
  double zero_t;
};

inline double ParseNumber(const std::string & text)
{
  return text.empty() ? std::nan("") : std::stod(text);
}

inline bool ReadCsv(const std::string & path, std::vector<Row> * rows, std::string * error)
{
  std::ifstream in(path);
  if (!in) {
    *error = "cannot open " + path;
    return false;
  }
  std::string line;
  // the CSVs may end lines with "\r\n"
  auto strip_cr = [](std::string * text) {
      if (!text->empty() && text->back() == '\r') {
        text->pop_back();
      }
    };
  std::getline(in, line);
  strip_cr(&line);
  std::map<std::string, size_t> col;
  {
    std::stringstream header(line);
    std::string name;
    while (std::getline(header, name, ',')) {
      const size_t index = col.size();
      col[name] = index;
    }
  }
  const char * needed[] = {"t_ns", "reported_mode", "linear", "steering", "angular",
    "wheel_angle", "wheel_speed", "subst", "odom_yaw", "gyro_yaw", "gyro_rate", "ref", "zero_t"};
  for (const char * name : needed) {
    if (col.count(name) == 0) {
      *error = path + ": missing column " + name;
      return false;
    }
  }
  while (std::getline(in, line)) {
    strip_cr(&line);
    std::vector<std::string> cells;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      cells.push_back(cell);
    }
    if (!line.empty() && line.back() == ',') {
      cells.push_back("");
    }
    if (cells.size() != col.size()) {
      *error = path + ": bad row: " + line;
      return false;
    }
    auto get = [&](const char * name) {return cells[col[name]];};
    Row r;
    r.t_ns = std::stoll(get("t_ns"));
    r.sample.reported_mode = static_cast<uint8_t>(std::stoi(get("reported_mode")));
    r.sample.mode_changing = false;   // not in the recordings
    r.sample.linear_velocity = ParseNumber(get("linear"));
    r.sample.angular_velocity = ParseNumber(get("angular"));
    r.sample.steering_angle = ParseNumber(get("steering"));
    r.sample.wheel_angle = ParseNumber(get("wheel_angle"));
    r.sample.wheel_speed = ParseNumber(get("wheel_speed"));
    r.subst = std::stoi(get("subst"));
    r.odom_yaw = ParseNumber(get("odom_yaw"));
    r.gyro_yaw = ParseNumber(get("gyro_yaw"));
    r.gyro_rate = ParseNumber(get("gyro_rate"));
    r.ref = std::stoi(get("ref")) != 0;
    r.zero_t = ParseNumber(get("zero_t"));
    rows->push_back(r);
  }
  return true;
}

struct Result
{
  std::vector<ml::OdometryStep> steps;
  // Ticks that consumed a substituted (not recorded) motion-state value at non-zero speed.
  int substituted_consumed = 0;
  double substituted_max_abs = 0.0;
  std::vector<double> substituted_times;
};

inline Result Run(const std::vector<Row> & rows, const ml::Limits & limits)
{
  Result result;
  ml::OdometryIntegrator odom(limits);
  odom.Reset(0.0, 0.0, rows.empty() ? 0.0 : rows.front().odom_yaw);
  for (size_t i = 0; i < rows.size(); ++i) {
    const double dt = i == 0 ? 0.0 : static_cast<double>(rows[i].t_ns - rows[i - 1].t_ns) * 1e-9;
    const ml::OdometryStep step = odom.Step(rows[i].sample, dt);
    result.steps.push_back(step);
    if (step.source != ml::InputSource::kMotionState || i == 0) {
      continue;
    }
    const ml::FeedbackSample & s = rows[i].sample;
    double consumed = 0.0;
    bool substituted = false;
    if (step.geometry == ml::Geometry::kSpin) {
      substituted = (rows[i].subst & kSubstAngular) != 0;
      consumed = s.angular_velocity;
    } else {
      substituted = (rows[i].subst & (kSubstLinear | kSubstSteering)) != 0;
      consumed = s.linear_velocity;
    }
    if (substituted && std::abs(consumed) > 1e-3) {
      ++result.substituted_consumed;
      result.substituted_max_abs = std::max(result.substituted_max_abs, std::abs(consumed));
      result.substituted_times.push_back(static_cast<double>(rows[i].t_ns) * 1e-9);
    }
  }
  return result;
}

// Heading tolerance against the recorded gyroscope, fixed before the integrator was written.
// The gyro columns are already bias-corrected and shifted by 35 ms.
struct Inv2
{
  double max_abs_err = 0.0;
  double worst_ratio = 0.0;
  bool t1_pass = true;
  std::vector<double> stop_steps;
  std::vector<double> stop_limits;
  bool t2_pass = true;
};

constexpr double kDeg = 3.14159265358979323846 / 180.0;

inline Inv2 CheckInv2(const std::vector<Row> & rows, const std::vector<double> & yaw)
{
  Inv2 out;
  size_t i0 = 0;
  for (size_t i = 0; i < rows.size(); ++i) {
    if (rows[i].ref) {
      i0 = i;
    }
  }
  std::vector<double> e(rows.size(), 0.0);
  for (size_t i = 0; i < rows.size(); ++i) {
    e[i] = (yaw[i] - yaw[i0]) - (rows[i].gyro_yaw - rows[i0].gyro_yaw);
  }
  double side = 0.0;
  double cum = 0.0;
  for (size_t i = i0; i < rows.size(); ++i) {
    if (i > i0) {
      const double dt = static_cast<double>(rows[i].t_ns - rows[i - 1].t_ns) * 1e-9;
      if (rows[i].sample.reported_mode == 1) {
        side += std::abs(rows[i].sample.wheel_speed) * dt;
      }
      cum += std::abs(rows[i].gyro_yaw - rows[i - 1].gyro_yaw);
    }
    const double tol = 1.0 * kDeg + 2.5 * kDeg * side + 0.05 * cum;
    out.max_abs_err = std::max(out.max_abs_err, std::abs(e[i]));
    out.worst_ratio = std::max(out.worst_ratio, std::abs(e[i]) / tol);
  }
  out.t1_pass = out.worst_ratio <= 1.0;
  for (size_t j = 0; j < rows.size(); ++j) {
    if (std::isnan(rows[j].zero_t)) {
      continue;
    }
    const double tz = rows[j].zero_t;
    double step = 0.0;
    for (size_t i = 0; i < rows.size(); ++i) {
      const double t = static_cast<double>(rows[i].t_ns) * 1e-9;
      if (t >= tz && t <= tz + 1.5) {
        step = std::max(step, std::abs(e[i] - e[j]));
      }
    }
    const double limit = 0.5 * kDeg + std::abs(rows[j].gyro_rate) * 0.035;
    out.stop_steps.push_back(step);
    out.stop_limits.push_back(limit);
    out.t2_pass = out.t2_pass && step <= limit;
  }
  return out;
}

}  // namespace replay

#endif  // REPLAY_IO_HPP_
