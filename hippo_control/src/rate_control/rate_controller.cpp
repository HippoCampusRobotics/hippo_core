#include <hippo_common/param_utils.hpp>
#include <hippo_control/rate_control/rate_controller.hpp>

namespace hippo_control {
namespace rate_control {
RateController::RateController(rclcpp::NodeOptions const &_options)
    : Node("rate_controller", _options) {
  DeclareParams();
  InitController();
  t_last_update_ = now();

  actuator_controls_pub_ = create_publisher<hippo_msgs::msg::ActuatorControls>(
      "actuator_control", rclcpp::SystemDefaultsQoS());

  rclcpp::SensorDataQoS qos;

  angular_velocity_sub_ = create_subscription<hippo_msgs::msg::AngularVelocity>(
      "angular_velocity", qos,
      std::bind(&RateController::OnAngularVelocity, this,
                std::placeholders::_1));

  body_rates_setpoint_sub_ = create_subscription<hippo_msgs::msg::RatesTarget>(
      "rates_setpoint", rclcpp::SystemDefaultsQoS(),
      std::bind(&RateController::OnRatesSetpoint, this, std::placeholders::_1));
}

void RateController::InitController() { UpdateAllControllerParams(); }

void RateController::UpdateAllControllerParams() {
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

void RateController::OnAngularVelocity(
    hippo_msgs::msg::AngularVelocity::ConstSharedPtr _msg) {
  if (params_.updated) {
    UpdateAllControllerParams();
    params_.updated = false;
  }

  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d angular_acceleration;

  for (int i = 0; i < 3; ++i) {
    angular_velocity(i) = _msg->body_rates[i];
    angular_acceleration(i) = _msg->body_rates_derivative[i];
  }

  auto t_now = now();
  double dt = (t_now - t_last_update_).nanoseconds() * 1e-9;
  dt = std::clamp(dt, 1e-3, 0.02);
  t_last_update_ = t_now;
  Eigen::Vector3d u_rpy = controller_.Update(
      angular_velocity, body_rates_setpoint_, angular_acceleration, dt);

  hippo_msgs::msg::ActuatorControls msg;
  msg.header.stamp = t_now;
  msg.control[msg.INDEX_ROLL] = u_rpy.x();
  msg.control[msg.INDEX_PITCH] = u_rpy.y();
  msg.control[msg.INDEX_YAW] = u_rpy.z();
  actuator_controls_pub_->publish(msg);
}

void RateController::OnRatesSetpoint(
    hippo_msgs::msg::RatesTarget::ConstSharedPtr _msg) {
  body_rates_setpoint_.x() = _msg->roll;
  body_rates_setpoint_.y() = _msg->pitch;
  body_rates_setpoint_.z() = _msg->yaw;
  if (_msg->reset_integral) {
    controller_.ResetIntegral();
  }
}

}  // namespace rate_control
}  // namespace hippo_control

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hippo_control::rate_control::RateController)
