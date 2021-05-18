///////////////////////////////////////////////////////////////////////////////
//      Title     : servo_sim.cpp
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

// Take control_effort messages, run a servo simulation, and publish state messages.

#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "std_msgs/msg/float64.hpp"
using std::placeholders::_1;

void print_usage()
{
  printf("Usage for servo_sim node:\n");
  printf("servo_sim [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
}

// Create a ServoSim class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class ServoSim : public rclcpp::Node
{
public:
  explicit ServoSim()
  : Node("servo_sim"), delta_t_(0, 0)
  {
    state_msg_ = std_msgs::msg::Float64();

    // Create a publisher with a custom Quality of Service profile.
    rclcpp::QoS custom_qos_profile(rclcpp::KeepLast(7), rmw_qos_profile_sensor_data);
    state_pub_ = this->create_publisher<std_msgs::msg::Float64>("state", custom_qos_profile);

    control_effort_sub_ = this->create_subscription<std_msgs::msg::Float64>("control_effort", 10, std::bind(&ServoSim::control_effort_callback, this, _1));

    prev_time_ = this->now();
  }

  void simulate()
  {
    delta_t_ = this->now() - prev_time_;
    prev_time_ = this->now();

    decel_force_ = -(speed_ * friction_);  // can be +ve or -ve. Linear with speed
    acceleration_ = ((Kv_ * (control_effort_ - (Kbackemf_ * speed_)) + decel_force_) / mass_);  // a = F/m
    speed_ = speed_ + (acceleration_ * delta_t_.nanoseconds()/1e9);
    displacement_ = displacement_ + speed_ * delta_t_.nanoseconds()/1e9;
    state_msg_.data = displacement_;

    state_pub_->publish(state_msg_);
  }

private:

  // Callback for incoming control_effort messages
  void control_effort_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
     control_effort_ = msg->data;
     RCLCPP_INFO(this->get_logger(), "control effort: [%f]", control_effort_);
  }

  std_msgs::msg::Float64 state_msg_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr control_effort_sub_;
  double control_effort_ = 0;
  rclcpp::Time prev_time_;
  rclcpp::Duration delta_t_;

  // Simulation parameters (servo-motor with load)
  double speed_ = 0;         // meters/sec
  double acceleration_ = 0;  // meters/sec^2
  double mass_ = 0.1;        // in kg
  double friction_ = 1.0;    // a decelerating force factor
  double Kv_ = 1;            // motor constant: force (newtons) / volt
  double Kbackemf_ = 0;      // Volts of back-emf per meter/sec of speed
  double decel_force_ = 0;   // decelerating force
  double displacement_ = 0;  // meters
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
  auto my_sim = std::make_shared<ServoSim>();

  // Simulate until shut down
  rclcpp::Rate loop_rate(100);
  while (rclcpp::ok())
  {
    rclcpp::spin_some( my_sim );

    my_sim->simulate();

    // Add a small sleep to avoid 100% CPU usage
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
