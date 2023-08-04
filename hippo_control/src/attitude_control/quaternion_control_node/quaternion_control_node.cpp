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

#include "quaternion_control_node.hpp"

#include <rmw/qos_profiles.h>

#include <hippo_common/convert.hpp>

namespace hippo_control {
namespace attitude_control {

QuaternionControlNode::QuaternionControlNode(
    rclcpp::NodeOptions const &_options)
    : Node("attitude_controller", _options) {
  DeclareParams();
  InitPublishers();
  InitSubscriptions();
}

void QuaternionControlNode::InitPublishers() {
  std::string topic;
  rclcpp::QoS qos =
      rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

  topic = "angular_velocity_setpoint";
  angular_velocity_setpoint_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(topic, qos);

  topic = "~/attitude_target_debug";
  attitude_target_debug_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(topic, qos);
}

void QuaternionControlNode::InitSubscriptions() {
  std::string topic;
  rclcpp::QoS qos =
      rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

  // topic = "heading_target";
  // using hippo_msgs::msg::HeadingTarget;
  // heading_target_sub_ = create_subscription<HeadingTarget>(
  //     topic, qos,
  //     [this](const HeadingTarget::SharedPtr msg) { OnHeadingTarget(msg); });
  topic = "heading_target";
  using geometry_msgs::msg::Vector3Stamped;
  heading_target_sub_ = create_subscription<Vector3Stamped>(
      topic, qos,
      [this](const Vector3Stamped::SharedPtr msg) { OnHeadingTarget(msg); });

  topic = "odometry";
  using nav_msgs::msg::Odometry;
  odometry_sub_ = create_subscription<Odometry>(
      topic, qos, [this](const Odometry::SharedPtr msg) { OnOdometry(msg); });
}

void QuaternionControlNode::PublishAngularVelocitySetpoint(
    const rclcpp::Time &_now, const Eigen::Vector3d &_velocity) {
  geometry_msgs::msg::Vector3Stamped msg;
  msg.header.stamp = _now;
  msg.header.frame_id = hippo_common::tf2_utils::frame_id::BaseLink(this);
  hippo_common::convert::EigenToRos(_velocity, msg.vector);
  angular_velocity_setpoint_pub_->publish(msg);
}

void QuaternionControlNode::PublishAttitudeTargetDebug(
    const rclcpp::Time &_now, const geometry_msgs::msg::Pose &_pose) {
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = _now;
  msg.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  msg.pose = _pose;
  attitude_target_debug_pub_->publish(msg);
}

// void QuaternionControlNode::OnHeadingTarget(
//     const hippo_msgs::msg::HeadingTarget::SharedPtr _msg) {
//   hippo_common::convert::RosToEigen(_msg->heading, target_heading_);
//   hippo_common::convert::RosToEigen(_msg->current_orientation, orientation_);
//   const Eigen::Vector3d desired_body_rates =
//       attitude_controller_.Update(target_heading_, 0.0, orientation_);
// }

void QuaternionControlNode::OnHeadingTarget(
    const geometry_msgs::msg::Vector3Stamped::SharedPtr _msg) {
  hippo_common::convert::RosToEigen(_msg->vector, target_heading_);
}

void QuaternionControlNode::OnOdometry(
    const nav_msgs::msg::Odometry::SharedPtr _msg) {
  hippo_common::convert::RosToEigen(_msg->pose.pose.orientation, orientation_);
  const Eigen::Vector3d desired_body_rates =
      attitude_controller_.Update(target_heading_, 0.0, orientation_);

  const rclcpp::Time t_now = _msg->header.stamp;
  PublishAngularVelocitySetpoint(t_now, desired_body_rates);

  Eigen::Quaterniond q = hippo_common::tf2_utils::QuaternionFromHeading(target_heading_, 0.0);
  geometry_msgs::msg::Pose pose;
  pose.position = _msg->pose.pose.position;
  hippo_common::convert::EigenToRos(q, pose.orientation);
  PublishAttitudeTargetDebug(t_now, pose);

}
}  // namespace attitude_control
}  // namespace hippo_control

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
    hippo_control::attitude_control::QuaternionControlNode)
