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

// Topic names of the driver: a contract with other software (see test_contract.cpp).

#ifndef RANGER_BASE__TOPIC_NAMES_HPP_
#define RANGER_BASE__TOPIC_NAMES_HPP_

namespace westonrobot
{
namespace topics
{

constexpr char kCmdVel[] = "/cmd_vel";
constexpr char kSystemState[] = "/system_state";
constexpr char kMotionState[] = "/motion_state";
constexpr char kMotionFeedback[] = "/motion_feedback";
constexpr char kActuatorState[] = "/actuator_state";
constexpr char kBatteryState[] = "/battery_state";
constexpr char kDefaultOdom[] = "odom";   // default of the odom_topic_name parameter

}  // namespace topics
}  // namespace westonrobot

#endif  // RANGER_BASE__TOPIC_NAMES_HPP_
