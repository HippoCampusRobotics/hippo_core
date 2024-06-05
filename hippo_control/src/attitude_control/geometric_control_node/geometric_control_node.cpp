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

#include "geometric_control_node.hpp"

#include <chrono>
#include <hippo_common/tf2_utils.hpp>
#include <hippo_control/attitude_control/geometric_controller.hpp>

namespace hippo_control {
namespace attitude_control {
GeometricControlNode::GeometricControlNode(const rclcpp::NodeOptions &_options)
    : Node("attitude_controller", _options) {
  DeclareParams();
  InitPublishers();
  InitTimers();
  InitSubscriptions();
}

void GeometricControlNode::InitTimers() {
  setpoint_timeout_timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(kSetpointTimeoutMs),
      [this]() { OnSetpointTimeout(); });
}

void GeometricControlNode::InitPublishers() {
  using geometry_msgs::msg::QuaternionStamped;
  using hippo_control_msgs::msg::ActuatorSetpoint;

  std::string topic;
  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);

  topic = "torque_setpoint";
  torque_pub_ = create_publisher<ActuatorSetpoint>(topic, qos);

  topic = "~/current_setpoint";
  current_setpoint_pub_ = create_publisher<QuaternionStamped>(topic, qos);
}

void GeometricControlNode::InitSubscriptions() {
  using geometry_msgs::msg::Vector3Stamped;
  using hippo_control_msgs::msg::RollTarget;
  using nav_msgs::msg::Odometry;

  std::string topic;
  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);

  topic = "heading_target";
  heading_target_sub_ = create_subscription<Vector3Stamped>(
      topic, qos,
      [this](const Vector3Stamped::SharedPtr msg) { OnHeadingTarget(msg); });

  topic = "odometry";
  odometry_sub_ = create_subscription<Odometry>(
      topic, qos, [this](const Odometry::SharedPtr msg) { OnOdometry(msg); });

  topic = "roll_target";
  roll_target_sub_ = create_subscription<RollTarget>(
      topic, qos,
      [this](const RollTarget::SharedPtr msg) { OnRollTarget(msg); });
}

hippo_control_msgs::msg::ActuatorSetpoint GeometricControlNode::ZeroMsg(
    const rclcpp::Time &_now) const {
  hippo_control_msgs::msg::ActuatorSetpoint msg;
  msg.header.stamp = _now;
  msg.x = 0.0;
  msg.y = 0.0;
  msg.z = 0.0;
  return msg;
}

void GeometricControlNode::PublishZeroActuatorSetpoints(
    const rclcpp::Time &_now) {
  torque_pub_->publish(ZeroMsg(_now));
}

void GeometricControlNode::SetControllerGains() {
  std::array<double, 3> p_gains = {params_.gain.roll.p, params_.gain.pitch.p,
                                   params_.gain.yaw.p};
  controller_.SetPgains(p_gains);
  std::array<double, 3> d_gains = {params_.gain.roll.d, params_.gain.pitch.d,
                                   params_.gain.yaw.d};
  controller_.SetDgains(d_gains);
}

void GeometricControlNode::PublishCurrentSetpoint(
    const rclcpp::Time &_now, const Eigen::Quaterniond &_attitude) {
  using hippo_common::convert::EigenToRos;
  using hippo_common::tf2_utils::frame_id::InertialFrame;
  geometry_msgs::msg::QuaternionStamped msg;
  EigenToRos(_attitude, msg.quaternion);
  msg.header.stamp = _now;
  msg.header.frame_id = InertialFrame();
  current_setpoint_pub_->publish(msg);
}

void GeometricControlNode::OnSetpointTimeout() {
  if (setpoint_timed_out_) {
    return;
  }
  RCLCPP_INFO(get_logger(), "Setpoint timed out. Sending zero commands.");
  setpoint_timed_out_ = true;
  PublishZeroActuatorSetpoints(now());
}

void GeometricControlNode::OnHeadingTarget(
    const geometry_msgs::msg::Vector3Stamped::SharedPtr _msg) {
  setpoint_timeout_timer_->reset();
  if (setpoint_timed_out_) {
    RCLCPP_INFO(get_logger(),
                "Received setpoint, Setpoint not timed out anymore.");
  }
  setpoint_timed_out_ = false;

  if (_msg->header.frame_id !=
      hippo_common::tf2_utils::frame_id::InertialFrame()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Heading target frame is [%s] but only [%s] is handled. Ignoring...",
        _msg->header.frame_id.c_str(),
        hippo_common::tf2_utils::frame_id::InertialFrame().c_str());
    return;
  }
  hippo_common::convert::RosToEigen(_msg->vector, heading_target_);
  controller_.SetAngularVelocityTarget(0.0, 0.0, 0.0);
  using hippo_common::tf2_utils::QuaternionFromHeading;
  Eigen::Quaterniond attitude =
      QuaternionFromHeading(heading_target_, roll_target_);
  controller_.SetOrientationTarget(attitude);

  PublishCurrentSetpoint(_msg->header.stamp, attitude);
}

void GeometricControlNode::OnOdometry(
    const nav_msgs::msg::Odometry::SharedPtr _msg) {
  if (setpoint_timed_out_) {
    PublishZeroActuatorSetpoints(now());
    return;
  }

  const rclcpp::Time t_now = _msg->header.stamp;
  using hippo_common::tf2_utils::frame_id::BaseLink;

  hippo_control_msgs::msg::ActuatorSetpoint torque_msg;
  torque_msg.header.stamp = t_now;
  torque_msg.header.frame_id = BaseLink(this);
  Eigen::Quaterniond orientation;
  hippo_common::convert::RosToEigen(_msg->pose.pose.orientation, orientation);
  Eigen::Vector3d body_rates;
  hippo_common::convert::RosToEigen(_msg->twist.twist.angular, body_rates);
  Eigen::Vector3d torque = controller_.Update(orientation, body_rates);
  using hippo_common::convert::EigenToRos;
  EigenToRos(torque, torque_msg);
  torque_pub_->publish(torque_msg);
}

}  // namespace attitude_control
}  // namespace hippo_control

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
    hippo_control::attitude_control::GeometricControlNode)
