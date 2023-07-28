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
  using hippo_msgs::msg::ActuatorSetpoint;
  using hippo_msgs::msg::AttitudeTarget;

  std::string topic;
  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);

  topic = "thrust_setpoint";
  thrust_pub_ = create_publisher<ActuatorSetpoint>(topic, qos);

  topic = "torque_setpoint";
  torque_pub_ = create_publisher<ActuatorSetpoint>(topic, qos);

  topic = "~/current_setpoint";
  current_setpoint_pub_ = create_publisher<AttitudeTarget>(topic, qos);
}

void GeometricControlNode::InitSubscriptions() {
  using hippo_msgs::msg::AttitudeTarget;
  using nav_msgs::msg::Odometry;

  std::string topic;
  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);

  topic = "attitude_target";
  attitude_target_sub_ = create_subscription<AttitudeTarget>(
      topic, qos,
      [this](const AttitudeTarget::SharedPtr msg) { OnAttitudeTarget(msg); });

  topic = "odometry";
  odometry_sub_ = create_subscription<Odometry>(
      topic, qos, [this](const Odometry::SharedPtr msg) { OnOdometry(msg); });
}

hippo_msgs::msg::ActuatorSetpoint GeometricControlNode::ZeroMsg(
    const rclcpp::Time &_now) const {
  hippo_msgs::msg::ActuatorSetpoint msg;
  msg.header.stamp = _now;
  msg.x = 0.0;
  msg.y = 0.0;
  msg.z = 0.0;
  return msg;
}

void GeometricControlNode::PublishZeroActuatorSetpoints(
    const rclcpp::Time &_now) {
  using hippo_msgs::msg::ActuatorSetpoint;
  ActuatorSetpoint thrust_msg = ZeroMsg(_now);
  ActuatorSetpoint torque_msg = ZeroMsg(_now);
  thrust_pub_->publish(thrust_msg);
  torque_pub_->publish(torque_msg);
}

void GeometricControlNode::PublishInFeedthroughMode() {
  const rclcpp::Time t_now = now();
  if (setpoint_timed_out_) {
    PublishZeroActuatorSetpoints(t_now);
    return;
  }
  using hippo_common::tf2_utils::frame_id::BaseLink;

  hippo_msgs::msg::ActuatorSetpoint thrust_msg;
  thrust_msg.header.stamp = t_now;
  thrust_msg.header.frame_id = BaseLink(this);
  thrust_msg.x = attitude_target_.thrust;

  hippo_msgs::msg::ActuatorSetpoint torque_msg;
  torque_msg.header.stamp = t_now;
  torque_msg.header.frame_id = BaseLink(this);
  torque_msg.x = attitude_target_.body_rate.x;
  torque_msg.y = attitude_target_.body_rate.y;
  torque_msg.z = attitude_target_.body_rate.z;

  thrust_pub_->publish(thrust_msg);
  torque_pub_->publish(torque_msg);
}

void GeometricControlNode::SetControllerGains() {
  std::array<double, 3> p_gains = {params_.gain.roll.p, params_.gain.pitch.p,
                                   params_.gain.yaw.p};
  controller_.SetPgains(p_gains);
  std::array<double, 3> d_gains = {params_.gain.roll.d, params_.gain.pitch.d,
                                   params_.gain.yaw.d};
  controller_.SetDgains(d_gains);
}

void GeometricControlNode::OnSetpointTimeout() {
  if (setpoint_timed_out_) {
    return;
  }
  RCLCPP_INFO(get_logger(), "Setpoint timed out. Sending zero commands.");
  setpoint_timed_out_ = true;
  using hippo_msgs::msg::ActuatorSetpoint;
  auto t_now = now();
  ActuatorSetpoint thrust_msg = ZeroMsg(t_now);
  ActuatorSetpoint torque_msg = ZeroMsg(t_now);
  thrust_pub_->publish(thrust_msg);
  torque_pub_->publish(torque_msg);
}

void GeometricControlNode::OnAttitudeTarget(
    const hippo_msgs::msg::AttitudeTarget::SharedPtr _msg) {
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
        "AttitudeTarget frame is [%s] but only [%s] is handled. Ignoring...",
        _msg->header.frame_id.c_str(),
        hippo_common::tf2_utils::frame_id::InertialFrame().c_str());
    return;
  }
  if (!(_msg->mask & _msg->IGNORE_ATTITUDE)) {
    attitude_target_.attitude = _msg->attitude;
  }
  if (!(_msg->mask & _msg->IGNORE_ROLL_RATE)) {
    attitude_target_.body_rate.x = _msg->body_rate.x;
  }
  if (!(_msg->mask & _msg->IGNORE_PITCH_RATE)) {
    attitude_target_.body_rate.y = _msg->body_rate.y;
  }
  if (!(_msg->mask & _msg->IGNORE_YAW_RATE)) {
    attitude_target_.body_rate.z = _msg->body_rate.z;
  }
  controller_.SetAngularVelocityTarget(attitude_target_.body_rate.x,
                                       attitude_target_.body_rate.y,
                                       attitude_target_.body_rate.z);
  if (!(_msg->mask & _msg->IGNORE_THRUST)) {
    attitude_target_.thrust = _msg->thrust;
  }
  Eigen::Quaterniond attitude;
  hippo_common::convert::RosToEigen(attitude_target_.attitude, attitude);
  controller_.SetOrientationTarget(attitude);

  current_setpoint_pub_->publish(attitude_target_);
  if (params_.feedthrough) {
    PublishInFeedthroughMode();
  }
}

void GeometricControlNode::OnOdometry(
    const nav_msgs::msg::Odometry::SharedPtr _msg) {
  if (setpoint_timed_out_) {
    PublishZeroActuatorSetpoints(now());
    return;
  }
  if (params_.feedthrough) {
    return;
  }

  const rclcpp::Time t_now = now();
  using hippo_common::tf2_utils::frame_id::BaseLink;

  hippo_msgs::msg::ActuatorSetpoint thrust_msg;
  thrust_msg.header.stamp = t_now;
  thrust_msg.header.frame_id = BaseLink(this);
  thrust_msg.x = attitude_target_.thrust;

  hippo_msgs::msg::ActuatorSetpoint torque_msg;
  torque_msg.header.stamp = t_now;
  torque_msg.header.frame_id = BaseLink(this);
  Eigen::Quaterniond orientation;
  hippo_common::convert::RosToEigen(_msg->pose.pose.orientation, orientation);
  Eigen::Vector3d body_rates;
  hippo_common::convert::RosToEigen(_msg->twist.twist.angular, body_rates);
  auto torque = controller_.Update(orientation, body_rates);
  torque_msg.x = torque.x();
  torque_msg.y = torque.y();
  torque_msg.z = torque.z();

  thrust_pub_->publish(thrust_msg);
  torque_pub_->publish(torque_msg);
}

}  // namespace attitude_control
}  // namespace hippo_control

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
    hippo_control::attitude_control::GeometricControlNode)
