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
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA

#include "interface_node.hpp"

namespace arift_interface {

InterfaceNode::InterfaceNode(rclcpp::NodeOptions const &_options)
    : Node("arift_interface", _options) {
  InitPublishers();
  InitSubscriptions();
}

void InterfaceNode::InitPublishers() {
  std::string topic;
  rclcpp::SensorDataQoS qos;

  topic = "thrust_setpoint";
  using hippo_msgs::msg::ActuatorSetpoint;
  thrust_pub_ = create_publisher<ActuatorSetpoint>(topic, qos);

  topic = "torque_setpoint";
  torque_pub_ = create_publisher<ActuatorSetpoint>(topic, qos);
}

void InterfaceNode::InitSubscriptions() {
  std::string topic;
  rclcpp::SensorDataQoS qos;

  topic = "force_torque";
  using geometry_msgs::msg::TwistStamped;
  force_torque_sub_ = create_subscription<TwistStamped>(
      topic, qos, [this](const TwistStamped::SharedPtr msg) { OnTwist(msg); });
}

void InterfaceNode::OnTwist(
    const geometry_msgs::msg::TwistStamped::SharedPtr _msg) {
  hippo_msgs::msg::ActuatorSetpoint thrust_msg, torque_msg;

  thrust_msg.header.stamp = _msg->header.stamp;
  thrust_msg.x = _msg->twist.linear.x;
  thrust_msg.y = _msg->twist.linear.y;
  thrust_msg.z = _msg->twist.linear.z;

  torque_msg.header.stamp = _msg->header.stamp;
  torque_msg.x = _msg->twist.angular.x;
  torque_msg.y = _msg->twist.angular.y;
  torque_msg.z = _msg->twist.angular.z;

  thrust_pub_->publish(thrust_msg);
  torque_pub_->publish(torque_msg);
}

}  // namespace arift_interface

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(arift_interface::InterfaceNode)
