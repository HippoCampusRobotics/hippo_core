// Copyright (C) 2023 Thies Lennart Alff
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
// USA

#pragma once

#include <stdint.h>

#include <hippo_control_msgs/msg/actuator_setpoint.hpp>
#include <hippo_msgs/msg/newton_gripper_command.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace remote_control {
namespace joystick {

namespace axes {
static constexpr size_t kLeftStickLeftRight = 0;
static constexpr size_t kLeftStickUpDown = 1;
static constexpr size_t kLT = 2;
static constexpr size_t kRightStickLeftRight = 3;
static constexpr size_t kRightStickUpDown = 4;
static constexpr size_t kRT = 5;
static constexpr size_t kCrossLeftRight = 6;
static constexpr size_t kCrossUpDown = 7;

static constexpr size_t kNumAxes = 8;
};  // namespace axes

namespace buttons {
static constexpr size_t kA = 0;
static constexpr size_t kB = 1;
static constexpr size_t kX = 2;
static constexpr size_t kY = 3;
static constexpr size_t kLB = 4;
static constexpr size_t kRB = 5;

static constexpr size_t kNumButtons = 6;
};  // namespace buttons

class JoyStick : public rclcpp::Node {
 public:
  explicit JoyStick(rclcpp::NodeOptions const &_options);
  struct Params {
    struct Gains {
      struct Torque {
        double x{1.0};
        double y{1.0};
        double z{1.0};
      } torque;
      struct Thrust {
        double x{1.0};
        double y{1.0};
        double z{1.0};
      } thrust;
    } gains;
  };

 private:
  void DeclareParams();
  rcl_interfaces::msg::SetParametersResult OnGainParams(
      const std::vector<rclcpp::Parameter> _parameters);
  void InitPublishers();
  void InitSubscribers();

  std::array<double, 3> ComputeThrust(const std::vector<float> &_axes,
                                      const std::vector<int32_t> &_buttons);
  std::array<double, 3> ComputeTorque(const std::vector<float> &_axes,
                                      const std::vector<int32_t> &_buttons);
  hippo_msgs::msg::NewtonGripperCommand ComputeNewtonGripperCommand(
      const std::vector<float> &_axes, const std::vector<int32_t> &_buttons);

  void PublishThrust(const std::array<double, 3> &_thrust);
  void PublishTorque(const std::array<double, 3> &_torque);
  void PublishNewtonGripperCommand(
      const hippo_msgs::msg::NewtonGripperCommand &_command);

  //////////////////////////////////////////////////////////////////////////////
  // Callbacks
  //////////////////////////////////////////////////////////////////////////////
  void OnJoy(const sensor_msgs::msg::Joy::SharedPtr _msg);
  OnSetParametersCallbackHandle::SharedPtr gain_params_cb_handle_;

  //////////////////////////////////////////////////////////////////////////////
  // Publishers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<hippo_control_msgs::msg::ActuatorSetpoint>::SharedPtr
      thrust_pub_;
  rclcpp::Publisher<hippo_control_msgs::msg::ActuatorSetpoint>::SharedPtr
      torque_pub_;
  rclcpp::Publisher<hippo_msgs::msg::NewtonGripperCommand>::SharedPtr
      gripper_pub_;
  //////////////////////////////////////////////////////////////////////////////
  // Subscriptions
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  Params params_;
};
}  // namespace joystick
}  // namespace remote_control
