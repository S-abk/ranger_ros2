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

// Test tool (not installed): replays the odometry integrator over feedback CSVs recovered from
// recordings of the real base and writes the integrated pose for comparison with a gyroscope.
//
// Usage: odom_replay <inputs_dir> <out_dir> <recording> [<recording> ...]
// Writes <out_dir>/<recording>.csv with t, yaw, x, y, vx, vy, wz, geometry, source.

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "ranger_base/motion_logic.hpp"
#include "ranger_base/ranger_params.hpp"
#include "replay_io.hpp"

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::fprintf(stderr, "usage: %s <inputs_dir> <out_dir> <recording>...\n", argv[0]);
    return 2;
  }
  const auto limits = replay::ml::LimitsFromParams<westonrobot::RangerMiniV3Params>(false);
  int status = 0;
  for (int k = 3; k < argc; ++k) {
    const std::string name = argv[k];
    std::vector<replay::Row> rows;
    std::string error;
    if (!replay::ReadCsv(std::string(argv[1]) + "/" + name + ".csv", &rows, &error)) {
      std::fprintf(stderr, "%s\n", error.c_str());
      status = 1;
      continue;
    }
    const replay::Result result = replay::Run(rows, limits);
    std::ofstream out(std::string(argv[2]) + "/" + name + ".csv");
    out << "t,yaw,x,y,vx,vy,wz,geometry,source\n";
    int counts[4] = {0, 0, 0, 0};
    int from_wheels = 0;
    char buffer[256];
    for (size_t i = 0; i < rows.size(); ++i) {
      const auto & s = result.steps[i];
      std::snprintf(
        buffer, sizeof(buffer), "%.9f,%.9f,%.9f,%.9f,%.6f,%.6f,%.6f,%d,%d\n",
        static_cast<double>(rows[i].t_ns) * 1e-9, s.theta, s.x, s.y, s.vx, s.vy, s.wz,
        static_cast<int>(s.geometry), static_cast<int>(s.source));
      out << buffer;
      counts[static_cast<int>(s.geometry)]++;
      from_wheels += s.source == replay::ml::InputSource::kWheels;
    }
    std::printf(
      "%-24s ticks=%zu ackermann=%d oblique=%d spin=%d none=%d from_wheels=%d "
      "substituted_consumed=%d (max |value| %.4f)\n",
      name.c_str(), rows.size(), counts[0], counts[1], counts[2], counts[3], from_wheels,
      result.substituted_consumed, result.substituted_max_abs);
    if (result.substituted_consumed > 0) {
      std::printf("  WARNING: substituted inputs consumed at t =");
      for (double t : result.substituted_times) {
        std::printf(" %.3f", t);
      }
      std::printf("\n");
    }
  }
  return status;
}
