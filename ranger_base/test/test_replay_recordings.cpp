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

// INV-2 on the real recordings: the odometry integrator replayed over feedback recovered from
// eleven recordings of the real base, against the gyroscope, with the tolerance frozen before
// the new code ran. The old driver's /odom yaw is checked too, to show the test can fail.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "ranger_base/motion_logic.hpp"
#include "ranger_base/ranger_params.hpp"
#include "replay_io.hpp"

namespace
{

const char * const kRecordings[] = {
  "A_forward", "B_left_after_forward", "C_left_after_backward", "D_diagonal", "E_spin",
  "C2_left_after_diagonal", "D2_forward_left", "x_stop_10", "y_stop_10", "y_stop_20",
  "x_stop_20"};

std::vector<replay::Row> Load(const std::string & name)
{
  std::vector<replay::Row> rows;
  std::string error;
  const std::string path = std::string(REPLAY_FIXTURE_DIR) + "/" + name + ".csv";
  EXPECT_TRUE(replay::ReadCsv(path, &rows, &error)) << error;
  return rows;
}

}  // namespace

TEST(ReplayRecordings, NewOdometryFollowsTheGyro)
{
  const auto limits = replay::ml::LimitsFromParams<westonrobot::RangerMiniV3Params>(false);
  for (const char * name : kRecordings) {
    SCOPED_TRACE(name);
    const std::vector<replay::Row> rows = Load(name);
    ASSERT_FALSE(rows.empty());
    const replay::Result result = replay::Run(rows, limits);
    std::vector<double> yaw;
    for (const auto & s : result.steps) {
      yaw.push_back(s.theta);
    }
    const replay::Inv2 inv2 = replay::CheckInv2(rows, yaw);
    RecordProperty(std::string(name) + "_max_abs_err_deg", std::to_string(inv2.max_abs_err /
      replay::kDeg));
    RecordProperty(std::string(name) + "_substituted_consumed",
      std::to_string(result.substituted_consumed));
    EXPECT_TRUE(inv2.t1_pass) << "max |e| " << inv2.max_abs_err / replay::kDeg <<
      " deg, worst ratio " << inv2.worst_ratio;
    for (size_t k = 0; k < inv2.stop_steps.size(); ++k) {
      EXPECT_LE(inv2.stop_steps[k], inv2.stop_limits[k]) << "stop " << k << ": step " <<
        inv2.stop_steps[k] / replay::kDeg << " deg, limit " << inv2.stop_limits[k] / replay::kDeg;
    }
  }
}

TEST(ReplayRecordings, OldOdometryFailsWhereTheBaseFactsSayItIsWrong)
{
  for (const char * name : {"y_stop_20", "E_spin"}) {
    SCOPED_TRACE(name);
    const std::vector<replay::Row> rows = Load(name);
    std::vector<double> yaw;
    double previous = 0.0;
    double offset = 0.0;
    for (size_t i = 0; i < rows.size(); ++i) {
      // unwrap the recorded yaw
      if (i > 0) {
        const double d = rows[i].odom_yaw - previous;
        if (d > 3.14159265358979323846) {
          offset -= 2.0 * 3.14159265358979323846;
        } else if (d < -3.14159265358979323846) {
          offset += 2.0 * 3.14159265358979323846;
        }
      }
      previous = rows[i].odom_yaw;
      yaw.push_back(rows[i].odom_yaw + offset);
    }
    const replay::Inv2 inv2 = replay::CheckInv2(rows, yaw);
    EXPECT_FALSE(inv2.t2_pass);
  }
}
