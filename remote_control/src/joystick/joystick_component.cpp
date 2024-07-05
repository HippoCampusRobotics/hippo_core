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

#include "joystick_component.hpp"

#include <hippo_common/tf2_utils.hpp>
#include <stdexcept>

namespace remote_control {
namespace joystick {
JoyStick::JoyStick(const rclcpp::NodeOptions &_options)
    : Node("joystick", _options) {
  DeclareParams();
  InitPublishers();
  InitSubscribers();
}

void JoyStick::InitPublishers() {
  std::string topic;

  topic = "thrust_setpoint";
  thrust_pub_ =
      create_publisher<hippo_control_msgs::msg::ActuatorSetpoint>(topic, 10);

  topic = "torque_setpoint";
  torque_pub_ =
      create_publisher<hippo_control_msgs::msg::ActuatorSetpoint>(topic, 10);

  topic = "gripper_command";
  gripper_pub_ =
      create_publisher<hippo_msgs::msg::NewtonGripperCommand>(topic, 10);
}

void JoyStick::OnJoy(const sensor_msgs::msg::Joy::SharedPtr _msg) {
  std::array<double, 3> thrust = ComputeThrust(_msg->axes, _msg->buttons);
  std::array<double, 3> torque = ComputeTorque(_msg->axes, _msg->buttons);
  hippo_msgs::msg::NewtonGripperCommand gripper_command =
      ComputeNewtonGripperCommand(_msg->axes, _msg->buttons);

  PublishThrust(thrust);
  PublishTorque(torque);
  PublishNewtonGripperCommand(gripper_command);
}

void JoyStick::InitSubscribers() {
  std::string topic;

  topic = "joy";
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      topic, 10, std::bind(&JoyStick::OnJoy, this, std::placeholders::_1));
}

std::array<double, 3> JoyStick::ComputeThrust(
    [[maybe_unused]] const std::vector<float> &_axes,
    [[maybe_unused]] const std::vector<int32_t> &_buttons) {
  std::array<double, 3> thrust;
  if (axes::kNumAxes > _axes.size()) {
    throw std::out_of_range("Axis index out of range.");
  }
  thrust[0] = (double)_axes.at(axes::kLeftStickUpDown) * params_.gains.thrust.x;
  thrust[1] =
      (double)_axes.at(axes::kLeftStickLeftRight) * params_.gains.thrust.y;
  thrust[2] =
      (double)_axes.at(axes::kRightStickUpDown) * params_.gains.thrust.z;
  return thrust;
}

std::array<double, 3> JoyStick::ComputeTorque(
    [[maybe_unused]] const std::vector<float> &_axes,
    [[maybe_unused]] const std::vector<int32_t> &_buttons) {
  std::array<double, 3> torque;
  if (axes::kNumAxes > _axes.size()) {
    throw std::out_of_range("Axis index out of range.");
  }
  torque[0] = 0.0 * params_.gains.torque.x;
  torque[1] = 0.0 * params_.gains.torque.y;
  // RT buttons are normally 1.0 and are -1.0 if pressed.
  torque[2] =
      (double)_axes.at(axes::kRightStickLeftRight) * params_.gains.torque.z;
  return torque;
}

hippo_msgs::msg::NewtonGripperCommand JoyStick::ComputeNewtonGripperCommand(
    [[maybe_unused]] const std::vector<float> &_axes,
    [[maybe_unused]] const std::vector<int32_t> &_buttons) {
  hippo_msgs::msg::NewtonGripperCommand msg;
  msg.header.stamp = now();

  // choose the appropriate action based on the buttons pressed.
  if (_buttons.at(buttons::kA)) {
    msg.action = msg.ACTION_CLOSE;
  } else if (_buttons.at(buttons::kX)) {
    msg.action = msg.ACTION_OPEN;
  } else {
    msg.action = msg.ACTION_NONE;
  }
  return msg;
}

void JoyStick::PublishThrust(const std::array<double, 3> &_thrust) {
  hippo_control_msgs::msg::ActuatorSetpoint msg;
  msg.header.stamp = now();
  msg.header.frame_id = hippo_common::tf2_utils::frame_id::BaseLink(this);
  msg.x = _thrust[0];
  msg.y = _thrust[1];
  msg.z = _thrust[2];
  if (!thrust_pub_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                          "Thrust publisher not initialized.");
    return;
  }
  thrust_pub_->publish(msg);
}

void JoyStick::PublishTorque(const std::array<double, 3> &_torque) {
  hippo_control_msgs::msg::ActuatorSetpoint msg;
  msg.header.stamp = now();
  msg.header.frame_id = hippo_common::tf2_utils::frame_id::BaseLink(this);
  msg.ignore_x = true;
  msg.ignore_y = true;
  msg.z = _torque[2];
  if (!torque_pub_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                          "Torque publisher not initialized.");
    return;
  }
  torque_pub_->publish(msg);
}

void JoyStick::PublishNewtonGripperCommand(
    const hippo_msgs::msg::NewtonGripperCommand &_command) {
  if (!gripper_pub_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                          "Gripper publisher not initialized.");
    return;
  }
  gripper_pub_->publish(_command);
}

}  // namespace joystick
}  // namespace remote_control

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(remote_control::joystick::JoyStick)
