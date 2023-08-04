#include "control_node.hpp"

#include <hippo_common/convert.hpp>

namespace hippo_control {
namespace motor_failure {

ControlNode::ControlNode(rclcpp::NodeOptions const &_options)
    : Node("motor_failure_control", _options) {
  InitPublishers();
  InitSubscriptions();
}

void ControlNode::InitPublishers() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);

  topic = "thruster_command";
  using hippo_msgs::msg::ActuatorControls;
  actuator_controls_pub_ = create_publisher<ActuatorControls>(topic, qos);
}

void ControlNode::InitSubscriptions() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);

  topic = "thrust_setpoint";
  using hippo_msgs::msg::ActuatorSetpoint;
  thrust_sub_ = create_subscription<ActuatorSetpoint>(
      topic, qos,
      [this](const ActuatorSetpoint::SharedPtr msg) { OnThrustSetpoint(msg); });

  topic = "angular_velocity_setpoint";
  using geometry_msgs::msg::Vector3Stamped;
  angular_velocity_setpoint_sub_ = create_subscription<Vector3Stamped>(
      topic, qos, [this](const Vector3Stamped::SharedPtr msg) {
        OnAngularVelocitySetpoint(msg);
      });

  topic = "odometry";
  using nav_msgs::msg::Odometry;
  odometry_sub_ = create_subscription<Odometry>(
      topic, qos, [this](const Odometry::SharedPtr msg) { OnOdometry(msg); });
}

void ControlNode::OnAngularVelocitySetpoint(
    const geometry_msgs::msg::Vector3Stamped::SharedPtr _msg) {
  hippo_common::convert::RosToEigen(_msg->vector, angular_velocity_setpoint_);
}

void ControlNode::OnThrustSetpoint(
    const hippo_msgs::msg::ActuatorSetpoint::SharedPtr _msg) {
  hippo_common::convert::RosToEigen(*_msg, thrust_setpoint_);
}

void ControlNode::OnOdometry(const nav_msgs::msg::Odometry::SharedPtr _msg) {
  using hippo_common::convert::RosToEigen;
  RosToEigen(_msg->twist.twist.angular, angular_velocity_);
  RosToEigen(_msg->twist.twist.linear, linear_velocity_);
  RosToEigen(_msg->pose.pose.orientation, orientation_);
  controller_.SetTarget(angular_velocity_setpoint_.y(),
                        angular_velocity_setpoint_.z(), thrust_setpoint_.x());
  Eigen::Vector4d thrusts =
      controller_.Update(angular_velocity_.y(), angular_velocity_.z(),
                         0.0, orientation_);
  for (int i = 0; i < 4; ++i) {
    thrusts(i) = thruster_model_.ThrustToEscCommand(thrusts(i));
    thrusts(i) = std::min(1.0, std::max(-1.0, thrusts(i)));
    // TODO: think about scaling in case we have values outside [-1, 1]
  }
  PublishThrusterCommand(_msg->header.stamp, thrusts);
}

void ControlNode::PublishThrusterCommand(const rclcpp::Time &_now,
                                         Eigen::Vector4d &_cmds) {
  hippo_msgs::msg::ActuatorControls msg;
  msg.header.stamp = _now;
  for (int i = 0; i < 4; ++i) {
    msg.control[i] = _cmds(i);
  }
  actuator_controls_pub_->publish(msg);
}
}  // namespace motor_failure
}  // namespace hippo_control

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hippo_control::motor_failure::ControlNode)
