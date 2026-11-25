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

// INV-3: topic names and message definitions keep what they had on the jazzy branch. Each
// installed .msg must start with its jazzy text (types only grow at the end).

#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>

#include "ranger_base/topic_names.hpp"

namespace
{

std::string ReadFile(const std::string & path)
{
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// Line endings and trailing blank lines are not part of a definition.
std::string Normalise(std::string text)
{
  std::string out;
  for (char c : text) {
    if (c != '\r') {
      out += c;
    }
  }
  while (!out.empty() && (out.back() == '\n' || out.back() == ' ')) {
    out.pop_back();
  }
  return out;
}

}  // namespace

TEST(MessageContract, InstalledDefinitionsStartWithTheJazzyText)
{
  for (const char * name : {"ActuatorState", "ActuatorStateArray", "DriverState", "MotionState",
      "MotorState", "SystemState"})
  {
    SCOPED_TRACE(name);
    const std::string jazzy = Normalise(
      ReadFile(std::string(MSG_FIXTURE_DIR) + "/" + name + ".msg"));
    const std::string installed = Normalise(
      ReadFile(std::string(RANGER_MSGS_MSG_DIR) + "/" + name + ".msg"));
    ASSERT_FALSE(jazzy.empty());
    ASSERT_FALSE(installed.empty());
    EXPECT_EQ(installed.compare(0, jazzy.size(), jazzy), 0) << "installed:\n" << installed;
  }
}

TEST(TopicContract, NamesAreUnchanged)
{
  namespace topics = westonrobot::topics;
  EXPECT_STREQ(topics::kCmdVel, "/cmd_vel");
  EXPECT_STREQ(topics::kSystemState, "/system_state");
  EXPECT_STREQ(topics::kMotionState, "/motion_state");
  EXPECT_STREQ(topics::kActuatorState, "/actuator_state");
  EXPECT_STREQ(topics::kBatteryState, "/battery_state");
  EXPECT_STREQ(topics::kDefaultOdom, "odom");
}
