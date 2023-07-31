#pragma once

#include <hippo_msgs/msg/actuator_setpoint.hpp>
#include <hippo_msgs/msg/attitude_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hippo_control/attitude_control/geometric_controller.hpp"

namespace hippo_control {
namespace attitude_control {
class GeometricControlNode : public rclcpp::Node {
 public:
  GeometricControlNode(const rclcpp::NodeOptions &options);

 private:
  static constexpr int kSetpointTimeoutMs = 500;
  struct PDGains {
    double p{1.0};
    double d{0.1};
  };
  struct Params {
    bool updated{false};
    bool feedthrough{false};
    struct {
      PDGains roll;
      PDGains pitch;
      PDGains yaw;
    } gain;
  };
  void DeclareParams();
  void InitPublishers();
  void InitTimers();
  void InitSubscriptions();
  hippo_msgs::msg::ActuatorSetpoint ZeroMsg(const rclcpp::Time &_now) const;
  void PublishInFeedthroughMode();
  void PublishZeroActuatorSetpoints(const rclcpp::Time &now);

  void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void OnAttitudeTarget(const hippo_msgs::msg::AttitudeTarget::SharedPtr msg);
  void SetControllerGains();

  void OnSetpointTimeout();

  rcl_interfaces::msg::SetParametersResult OnGainParameters(
      const std::vector<rclcpp::Parameter> &parameters);

  //////////////////////////////////////////////////////////////////////////////
  // publishers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr thrust_pub_;
  rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr torque_pub_;
  rclcpp::Publisher<hippo_msgs::msg::AttitudeTarget>::SharedPtr
      current_setpoint_pub_;

  //////////////////////////////////////////////////////////////////////////////
  // subscriptions
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Subscription<hippo_msgs::msg::AttitudeTarget>::SharedPtr
      attitude_target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  OnSetParametersCallbackHandle::SharedPtr gain_params_cb_handle_;

  bool setpoint_timed_out_{false};
  rclcpp::TimerBase::SharedPtr setpoint_timeout_timer_;

  hippo_msgs::msg::AttitudeTarget attitude_target_;

  GeometricController controller_;

  Params params_;
};
}  // namespace attitude_control
}  // namespace hippo_control
