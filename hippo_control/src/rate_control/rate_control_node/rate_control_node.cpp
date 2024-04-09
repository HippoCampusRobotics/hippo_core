#include "rate_control_node.hpp"

#include <hippo_common/convert.hpp>
#include <hippo_common/param_utils.hpp>

namespace hippo_control {
namespace rate_control {
RateControlNode::RateControlNode(rclcpp::NodeOptions const &_options)
    : Node("rate_controller", _options) {
  DeclareParams();
  InitController();
  t_last_update_ = t_last_setpoint_ = now();

  torque_pub_ = create_publisher<hippo_control_msgs::msg::ActuatorSetpoint>(
      "torque_setpoint", rclcpp::SensorDataQoS());

  rates_debug_pub_ = create_publisher<hippo_control_msgs::msg::RatesDebug>(
      "~/rates_debug", rclcpp::SensorDataQoS());

  std::string topic;
  rclcpp::SensorDataQoS qos;
  qos.keep_last(1);

  topic = "odometry";
  using nav_msgs::msg::Odometry;
  odometry_sub_ = create_subscription<Odometry>(
      topic, qos, [this](const Odometry::SharedPtr msg) { OnOdometry(msg); });

  topic = "angular_velocity_setpoint";
  using geometry_msgs::msg::Vector3Stamped;
  angular_velocity_setpoint_sub_ = create_subscription<Vector3Stamped>(
      topic, rclcpp::SystemDefaultsQoS(),
      [this](const Vector3Stamped::SharedPtr msg) {
        OnAngularVelocitySetpoint(msg);
      });
}

void RateControlNode::InitController() { UpdateAllControllerParams(); }

void RateControlNode::UpdateAllControllerParams() {
  controller_.SetRollGainP(params_.gains.roll.p);
  controller_.SetRollGainI(params_.gains.roll.i);
  controller_.SetRollGainD(params_.gains.roll.d);
  controller_.SetRollFeedForwardGain(params_.gains.roll.feed_forward);

  controller_.SetPitchGainP(params_.gains.pitch.p);
  controller_.SetPitchGainI(params_.gains.pitch.i);
  controller_.SetPitchGainD(params_.gains.pitch.d);
  controller_.SetPitchFeedForwardGain(params_.gains.pitch.feed_forward);

  controller_.SetYawGainP(params_.gains.yaw.p);
  controller_.SetYawGainI(params_.gains.yaw.i);
  controller_.SetYawGainD(params_.gains.yaw.d);
  controller_.SetYawFeedForwardGain(params_.gains.yaw.feed_forward);

  controller_.SetRollIntegralLimit(params_.integral_limits.roll);
  controller_.SetPitchIntegralLimit(params_.integral_limits.pitch);
  controller_.SetYawIntegralLimit(params_.integral_limits.yaw);

  controller_.SetZeroIntegralThreshold(params_.zero_integral_threshold);
}

void RateControlNode::PublishTorqueOutput(const rclcpp::Time &_now,
                                          const Eigen::Vector3d &_torque) {
  hippo_control_msgs::msg::ActuatorSetpoint msg;
  msg.header.stamp = _now;
  hippo_common::convert::EigenToRos(_torque, msg);
  torque_pub_->publish(msg);
}

void RateControlNode::OnOdometry(
    const nav_msgs::msg::Odometry::SharedPtr _msg) {
  Eigen::Vector3d angular_velocity;
  hippo_common::convert::RosToEigen(_msg->twist.twist.angular,
                                    angular_velocity);
  // do not use angular acceleration for now by setting current and desired
  // value to zero.
  Eigen::Vector3d angular_acceleration{0.0, 0.0, 0.0};

  rclcpp::Time t_now = now();
  double dt = (t_now - t_last_update_).nanoseconds() * 1e-9;
  dt = std::clamp(dt, 1e-3, 0.02);
  t_last_update_ = t_now;

  double dt_setpoint = (t_now - t_last_setpoint_).nanoseconds() * 1e-9;
  if (dt_setpoint > 0.3) {
    controller_.ResetIntegral();
    PublishTorqueOutput(_msg->header.stamp, Eigen::Vector3d::Zero());
  } else {
    const Eigen::Vector3d u_rpy = controller_.Update(
        angular_velocity, body_rates_setpoint_, angular_acceleration, dt);
    PublishTorqueOutput(_msg->header.stamp, u_rpy);
  }

  hippo_control_msgs::msg::RatesDebug debug_msg;
  debug_msg.header.stamp = _msg->header.stamp;
  hippo_common::convert::EigenToRos(body_rates_setpoint_,
                                    debug_msg.rates_desired);
  hippo_common::convert::EigenToRos(angular_velocity, debug_msg.rates);
  rates_debug_pub_->publish(debug_msg);
}

void RateControlNode::OnAngularVelocitySetpoint(
    const geometry_msgs::msg::Vector3Stamped::SharedPtr _msg) {
  t_last_setpoint_ = now();
  hippo_common::convert::RosToEigen(_msg->vector, body_rates_setpoint_);
}

}  // namespace rate_control
}  // namespace hippo_control

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hippo_control::rate_control::RateControlNode)
