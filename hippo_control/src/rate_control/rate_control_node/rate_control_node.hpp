#pragma once

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <hippo_control/rate_control/rate_controller.hpp>
#include <hippo_control_msgs/msg/actuator_setpoint.hpp>
#include <hippo_control_msgs/msg/rates_debug.hpp>
#include <hippo_control_msgs/msg/rates_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hippo_control {
namespace rate_control {
class RateControlNode : public rclcpp::Node {
 public:
  explicit RateControlNode(rclcpp::NodeOptions const &_options);

 private:
  void DeclareParams();
  void DeclareGainParams();
  void DeclareIntegralLimitParams();
  void InitController();
  void UpdateAllControllerParams();
  void PublishTorqueOutput(const rclcpp::Time &now,
                           const Eigen::Vector3d &torque);

  void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr);
  void OnAngularVelocitySetpoint(
      const geometry_msgs::msg::Vector3Stamped::SharedPtr);
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

  rclcpp::Publisher<hippo_control_msgs::msg::ActuatorSetpoint>::SharedPtr
      torque_pub_;
  rclcpp::Publisher<hippo_control_msgs::msg::RatesDebug>::SharedPtr
      rates_debug_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      angular_velocity_setpoint_sub_;

  Eigen::Vector3d body_rates_setpoint_{0.0, 0.0, 0.0};
  rclcpp::Time t_last_update_;
  rclcpp::Time t_last_setpoint_;

  hippo_control::rate_control::RateController controller_;

  OnSetParametersCallbackHandle::SharedPtr gains_cb_handle_;
  OnSetParametersCallbackHandle::SharedPtr integral_limits_cb_handle_;
  OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;
};
}  // namespace rate_control
}  // namespace hippo_control
