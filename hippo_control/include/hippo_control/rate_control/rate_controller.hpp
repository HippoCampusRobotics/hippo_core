#pragma once

#include <hippo_control/rate_control/rate_control.hpp>
#include <hippo_msgs/msg/actuator_setpoint.hpp>
#include <hippo_msgs/msg/angular_velocity.hpp>
#include <hippo_msgs/msg/rates_target.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hippo_control {
namespace rate_control {
class RateController : public rclcpp::Node {
 public:
  explicit RateController(rclcpp::NodeOptions const &_options);

 private:
  void DeclareParams();
  void DeclareGainParams();
  void DeclareIntegralLimitParams();
  void InitController();
  void UpdateAllControllerParams();
  void OnAngularVelocity(hippo_msgs::msg::AngularVelocity::ConstSharedPtr _msg);
  void OnRatesSetpoint(hippo_msgs::msg::RatesTarget::ConstSharedPtr _msg);
  rcl_interfaces::msg::SetParametersResult OnGainParams(
      const std::vector<rclcpp::Parameter> &_parameters);
  rcl_interfaces::msg::SetParametersResult OnIntegralLimitParams(
      const std::vector<rclcpp::Parameter> &_parameters);
  rcl_interfaces::msg::SetParametersResult OnParams(
      const std::vector<rclcpp::Parameter> &_parameters);

  struct PidGains {
    double p{2.0};
    double i{0.5};
    double d{0.001};
    double feed_forward{0.0};
  };
  struct Params {
    struct Gains {
      PidGains roll;
      PidGains pitch;
      PidGains yaw;
    } gains;
    struct IntegralLimits {
      double roll{0.5};
      double pitch{0.5};
      double yaw{0.5};
    } integral_limits;
    double zero_integral_threshold{2.0};
    bool updated{false};
  } params_;

  rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr torque_pub_;

  rclcpp::Subscription<hippo_msgs::msg::AngularVelocity>::SharedPtr
      angular_velocity_sub_;
  rclcpp::Subscription<hippo_msgs::msg::RatesTarget>::SharedPtr
      body_rates_setpoint_sub_;

  Eigen::Vector3d body_rates_setpoint_{0.0, 0.0, 0.0};
  rclcpp::Time t_last_update_;
  rclcpp::Time t_last_setpoint_;

  hippo_control::rate_control::Controller controller_;

  OnSetParametersCallbackHandle::SharedPtr gains_cb_handle_;
  OnSetParametersCallbackHandle::SharedPtr integral_limits_cb_handle_;
  OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;
};
}  // namespace rate_control
}  // namespace hippo_control
