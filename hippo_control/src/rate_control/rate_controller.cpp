#include <hippo_common/param_utils.hpp>
#include <hippo_control/rate_control/rate_controller.hpp>

namespace hippo_control {
namespace rate_control {
RateController::RateController(rclcpp::NodeOptions const &_options)
    : Node("rate_controller", _options) {
  DeclareParams();
  InitController();
  t_last_update_ = t_last_setpoint_ = now();

  torque_pub_ = create_publisher<hippo_msgs::msg::ActuatorSetpoint>(
      "torque_setpoint", rclcpp::SensorDataQoS());

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

  hippo_msgs::msg::ActuatorSetpoint msg;
  msg.header.stamp = t_now;

  double dt_setpoint = (t_now - t_last_setpoint_).nanoseconds() * 1e-9;
  if (dt_setpoint > 0.3) {
    controller_.ResetIntegral();
    msg.x = 0;
    msg.y = 0;
    msg.z = 0;
  } else {
    Eigen::Vector3d u_rpy = controller_.Update(
        angular_velocity, body_rates_setpoint_, angular_acceleration, dt);

    msg.x = u_rpy.x();
    msg.y = u_rpy.y();
    msg.z = u_rpy.z();
  }
  torque_pub_->publish(msg);
}

void RateController::OnRatesSetpoint(
    hippo_msgs::msg::RatesTarget::ConstSharedPtr _msg) {
  t_last_setpoint_ = now();
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
