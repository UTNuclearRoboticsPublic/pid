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

// Perform PID calculations.

#include <pid/pid.h>

using namespace pid_ns;

PID::PID(double Kp, double Ki, double Kd):
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

  state_sub_ = create_subscription<std_msgs::msg::Float64>(topic_from_plant_, state_callback);
  setpoint_sub_ = create_subscription<std_msgs::msg::Float64>(setpoint_topic_, setpoint_callback);

  // Create a publisher with a custom Quality of Service profile.
  rclcpp::QoS custom_qos_profile(rclcpp::KeepLast(7), rmw_qos_profile_sensor_data);
  control_effort_pub_ = this->create_publisher<std_msgs::msg::Float64>(topic_from_controller_, custom_qos_profile);

  // TODO: get these parameters from server,
  // along with the other configurable params (cutoff_frequency_, etc.)
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  printParameters();

  if (not validateParameters())
    std::cout << "Error: invalid parameter\n";
}

void PID::printParameters()
{
  std::cout << std::endl << "PID PARAMETERS" << std::endl << "-----------------------------------------" << std::endl;
  std::cout << "Kp: " << Kp_ << ",  Ki: " << Ki_ << ",  Kd: " << Kd_ << std::endl;
  if (cutoff_frequency_ == -1)  // If the cutoff frequency was not specified by the user
    std::cout << "LPF cutoff frequency: 1/4 of sampling rate" << std::endl;
  else
    std::cout << "LPF cutoff frequency: " << cutoff_frequency_ << std::endl;
  std::cout << "pid node name: " << this->get_name() << std::endl;
  std::cout << "Name of topic from controller: " << topic_from_controller_ << std::endl;
  std::cout << "Name of topic from the plant: " << topic_from_plant_ << std::endl;
  std::cout << "Name of setpoint topic: " << setpoint_topic_ << std::endl;
  std::cout << "Integral-windup limit: " << windup_limit_ << std::endl;
  std::cout << "Saturation limits: " << upper_limit_ << "/" << lower_limit_ << std::endl;
  std::cout << "-----------------------------------------" << std::endl;

  return;
}

bool PID::validateParameters()
{
  if (lower_limit_ > upper_limit_)
  {
    RCLCPP_ERROR(this->get_logger(), "The lower saturation limit cannot be greater than the upper "
              "saturation limit.");
    return (false);
  }

  return true;
}

void PID::doCalcs()
{
  // Do fresh calcs if knowledge of the system has changed.
  if (new_state_or_setpt_)
  {
    if (!((Kp_ <= 0. && Ki_ <= 0. && Kd_ <= 0.) ||
          (Kp_ >= 0. && Ki_ >= 0. && Kd_ >= 0.)))  // All 3 gains should have the same sign
    {
      RCLCPP_WARN(this->get_logger(), "All three gains (Kp, Ki, Kd) should have the same sign for "
               "stability.");
    }

    error_[2] = error_[1];
    error_[1] = error_[0];
    error_[0] = setpoint_ - plant_state_;  // Current error goes to slot 0

    // If the angle_error param is true, then address discontinuity in error
    // calc.
    // For example, this maintains an angular error between -180:180.
    if (angle_error_)
    {
      while (error_[0] < -1.0 * angle_wrap_ / 2.0)
        error_[0] += angle_wrap_;
      while (error_[0] > angle_wrap_ / 2.0)
        error_[0] -= angle_wrap_;

      // The proportional error will flip sign, but the integral error
      // won't and the derivative error will be poorly defined. So,
      // reset them.
      error_[2] = 0.;
      error_[1] = 0.;
      error_integral_ = 0.;
    }

    // calculate delta_t
    if ( prev_time_.nanoseconds()!=0 )  // Not first time through the program
    {
      delta_t_ = this->now() - prev_time_;
      prev_time_ = this->now();
      if (0 == delta_t_.nanoseconds())
      {
        RCLCPP_ERROR(this->get_logger(), "delta_t is 0, skipping this loop. Possible overloaded CPU.");
        return;
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "prev_time is 0, doing nothing");
      prev_time_ = this->now();
      return;
    }

    // integrate the error
    error_integral_ += error_[0] * delta_t_.nanoseconds()/1e9;

    // Apply windup limit to limit the size of the integral term
    if (error_integral_ > fabsf(windup_limit_))
      error_integral_ = fabsf(windup_limit_);

    if (error_integral_ < -fabsf(windup_limit_))
      error_integral_ = -fabsf(windup_limit_);

    // My filter reference was Julius O. Smith III, Intro. to Digital Filters
    // With Audio Applications.
    if (cutoff_frequency_ != -1)
    {
      // Check if tan(_) is really small, could cause c = NaN
      tan_filt_ = tan((cutoff_frequency_ * 6.2832) * (delta_t_.nanoseconds()/1e9) / 2);

      // Avoid tan(0) ==> NaN
      if ((tan_filt_ <= 0.) && (tan_filt_ > -0.01))
        tan_filt_ = -0.01;
      if ((tan_filt_ >= 0.) && (tan_filt_ < 0.01))
        tan_filt_ = 0.01;

      c_ = 1 / tan_filt_;
    }

    filtered_error_[2] = filtered_error_[1];
    filtered_error_[1] = filtered_error_[0];
    filtered_error_[0] = (1 / (1 + c_ * c_ + 1.414 * c_)) * (error_[2] + 2 * error_[1] + error_[0] -
                                                                (c_ * c_ - 1.414 * c_ + 1) * filtered_error_[2] -
                                                                (-2 * c_ * c_ + 2) * filtered_error_[1]);

    // Take derivative of error
    // First the raw, unfiltered data:
    error_deriv_[2] = error_deriv_[1];
    error_deriv_[1] = error_deriv_[0];
    error_deriv_[0] = (error_[0] - error_[1]) / delta_t_.nanoseconds()/1e9;

    filtered_error_deriv_[2] = filtered_error_deriv_[1];
    filtered_error_deriv_[1] = filtered_error_deriv_[0];

    filtered_error_deriv_[0] =
        (1 / (1 + c_ * c_ + 1.414 * c_)) *
        (error_deriv_[2] + 2 * error_deriv_[1] + error_deriv_[0] -
         (c_ * c_ - 1.414 * c_ + 1) * filtered_error_deriv_[2] - (-2 * c_ * c_ + 2) * filtered_error_deriv_[1]);

    // calculate the control effort
    proportional_ = Kp_ * filtered_error_[0];
    integral_ = Ki_ * error_integral_;
    derivative_ = Kd_ * filtered_error_deriv_[0];
    control_effort_ = proportional_ + integral_ + derivative_;

    // Apply saturation limits
    if (control_effort_ > upper_limit_)
      control_effort_ = upper_limit_;
    else if (control_effort_ < lower_limit_)
      control_effort_ = lower_limit_;
  }

  // Publish the stabilizing control effort if the controller is enabled
  if (pid_enabled_)
  {
    control_msg_.data = control_effort_;
    control_effort_pub_->publish(control_msg_);
  }
  else
    error_integral_ = 0.0;

  new_state_or_setpt_ = false;
}