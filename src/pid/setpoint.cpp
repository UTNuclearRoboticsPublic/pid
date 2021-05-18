///////////////////////////////////////////////////////////////////////////////
//      Title     : setpoint.cpp
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

// Periodically publish a setpoint message.

#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "std_msgs/msg/float64.hpp"

void print_usage()
{
  printf("Usage for setpoint node:\n");
  printf("setpoint [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("TODO: link to web docs\n");
}

// Create a Setpoint class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Setpoint : public rclcpp::Node
{
public:
  explicit Setpoint()
  : Node("setpoint")
  {
    msg_ = std_msgs::msg::Float64();

    // Create a publisher with a custom Quality of Service profile.
    rclcpp::QoS custom_qos_profile(rclcpp::KeepLast(7), rmw_qos_profile_sensor_data);
    pub_ = this->create_publisher<std_msgs::msg::Float64>("setpoint", custom_qos_profile);

    // Negate the setpoint every 5 seconds
    rclcpp::Rate loop_rate(0.2);

    double setpoint = 1;

    while (rclcpp::ok())
    {
      setpoint = -setpoint;
      publish_message( setpoint );
      loop_rate.sleep();
    }
  }

  void publish_message(double& setpoint)
  {
      msg_.data = setpoint;
      pub_->publish(msg_);
  }

private:
  std_msgs::msg::Float64 msg_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
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
  auto node = std::make_shared<Setpoint>();

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
