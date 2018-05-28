///////////////////////////////////////////////////////////////////////////////
//      Title     : controller.cpp
//      Project   : pid
//      Created   : 5/25/2018
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

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "std_msgs/msg/float64.hpp"

#include "pid.h"

void print_usage()
{
  printf("Usage for controller node:\n");
  printf("controller [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("TODO: link to web docs\n");
}

// Create a Controller class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
// TODO: move this into pid.cpp
class Controller : public rclcpp::Node
{
public:
  explicit Controller()
  : Node("controller")
  {
    auto state_callback =
      [this](const std_msgs::msg::Float64::SharedPtr msg) -> void
      {
        state_ = msg-> data;
        RCLCPP_INFO(this->get_logger(), "State: [%f]", state_)
      };

    auto setpoint_callback =
      [this](const std_msgs::msg::Float64::SharedPtr msg) -> void
      {
        setpoint_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Setpoint: [%f]", setpoint_)
      };

    state_sub_ = create_subscription<std_msgs::msg::Float64>("state", state_callback);
    setpoint_sub_ = create_subscription<std_msgs::msg::Float64>("setpoint", setpoint_callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr state_sub_, setpoint_sub_;
  double state_, setpoint_;
};

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Display help?
  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  // Initialize any global resources needed by the middleware and the client library.
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  // Create a node.
  auto node = std::make_shared<Controller>();

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
