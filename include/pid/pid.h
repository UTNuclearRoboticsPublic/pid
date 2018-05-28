#ifndef PID_H
#define PID_H

#include "math.h"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <stdio.h>
#include <string>

namespace pid_ns
{
class PID : public rclcpp::Node
{
public:
  explicit PID();

  // Primary output variable
  double control_effort_ = 0;        // output of pid controller

private:
  void doCalcs();
  void printParameters();
  //bool validateParameters();

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr state_sub_, setpoint_sub_;

  // Primary PID controller input variables
  double plant_state_;               // current output of plant
  bool pid_enabled_ = true;          // PID is enabled to run
  bool new_state_or_setpt_ = false;  // Indicate that fresh calculations need to be run
  double setpoint_ = 0;              // desired output of plant

  rclcpp::Time prev_time_;
  rclcpp::Duration delta_t_;
  bool first_reconfig_ = true;

  double error_integral_ = 0;
  double proportional_ = 0;  // proportional term of output
  double integral_ = 0;      // integral term of output
  double derivative_ = 0;    // derivative term of output

  // PID gains
  double Kp_ = 0, Ki_ = 0, Kd_ = 0;

  // Parameters for error calc. with disconinuous input
  bool angle_error_ = false;
  double angle_wrap_ = 2.0 * 3.14159;

  // Cutoff frequency for the derivative calculation in Hz.
  // Negative -> Has not been set by the user yet, so use a default.
  double cutoff_frequency_ = -1;

  // Used in filter calculations. Default 1.0 corresponds to a cutoff frequency
  // at
  // 1/4 of the sample rate.
  double c_ = 1.;

  // Used to check for tan(0)==>NaN in the filter calculation
  double tan_filt_ = 1.;

  // Upper and lower saturation limits
  double upper_limit_ = 1000, lower_limit_ = -1000;

  // Anti-windup term. Limits the absolute value of the integral term.
  double windup_limit_ = 1000;

  // Initialize filter data with zeros
  std::vector<double> error_, filtered_error_, error_deriv_, filtered_error_deriv_;

  // Topic and node names and message objects
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_effort_pub_;

  std::string topic_from_controller_, topic_from_plant_, setpoint_topic_, pid_enable_topic_;
  std_msgs::msg::Float64 control_msg_, state_msg_;

  // Diagnostic objects
  double min_loop_frequency_ = 1, max_loop_frequency_ = 1000;
  int measurements_received_ = 0;
};
}  // end pid namespace

#endif