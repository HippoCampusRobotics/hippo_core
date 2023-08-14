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

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hippo_msgs/msg/actuator_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>

namespace arift_interface {

class InterfaceNode : public rclcpp::Node {
 public:
  explicit InterfaceNode(rclcpp::NodeOptions const &_options);

 private:
  void InitPublishers();
  void InitSubscriptions();

  void OnTwist(const geometry_msgs::msg::TwistStamped::SharedPtr);
  rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr thrust_pub_;
  rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr torque_pub_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      force_torque_sub_;
};

}  // namespace arift_interface
