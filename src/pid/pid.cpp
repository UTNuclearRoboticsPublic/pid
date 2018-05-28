///////////////////////////////////////////////////////////////////////////////
//      Title     : pid.cpp
//      Project   : pid
//      Created   : 5/28/2018
//      Author    : Andy Zelenak
//
// BSD 3-Clause License
//
// Copyright (c) 2018, Los Alamos National Security, LLC
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////

#include <pid/pid.h>

using namespace pid_ns;

PID::PID():
  Node("controller"), delta_t_(0, 0)
{
	// Callbacks for incoming state and setpoint messages
  auto state_callback =
    [this](const std_msgs::msg::Float64::SharedPtr msg) -> void
    {
      plant_state_ = msg-> data;
      RCLCPP_INFO(this->get_logger(), "State: [%f]", plant_state_)

      new_state_or_setpt_ = true;
    };

  auto setpoint_callback =
    [this](const std_msgs::msg::Float64::SharedPtr msg) -> void
    {
      setpoint_ = msg->data;
      RCLCPP_INFO(this->get_logger(), "Setpoint: [%f]", setpoint_)

      new_state_or_setpt_ = true;
    };

  state_sub_ = create_subscription<std_msgs::msg::Float64>("state", state_callback);
  setpoint_sub_ = create_subscription<std_msgs::msg::Float64>("setpoint", setpoint_callback);

  //while (rclcpp::Time(0) == rclcpp::Time::now())
  //{
  //  RCLCPP_INFO(this->get_logger(), "Spinning, waiting for time to become nonzero.");
  //  sleep(1);
  //} 
}
