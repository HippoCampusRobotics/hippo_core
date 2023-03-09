#pragma once
#include <hippo_common/param_utils.hpp>
#include <hippo_msgs/msg/actuator_controls.hpp>
#include <hippo_msgs/msg/actuator_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hippo_control/mixer/simple_mixer.hpp"

namespace hippo_control {
namespace mixer {
class ActuatorMixerNode : public rclcpp::Node {
 public:
  explicit ActuatorMixerNode(rclcpp::NodeOptions const &_options);
  void OnThrustSetpoint(
      const hippo_msgs::msg::ActuatorSetpoint::SharedPtr _msg);
  void OnTorqueSetpoint(
      const hippo_msgs::msg::ActuatorSetpoint::SharedPtr _msg);
  rcl_interfaces::msg::SetParametersResult OnThrustParams(
      const std::vector<rclcpp::Parameter> &_parameters);

 private:
  void DeclareParams();
  void WatchdogTimeout();
  void ResetThrust();
  void ResetTorque();
  void PublishActuatorCommand(const rclcpp::Time &_now);
  mixer::SimpleMixer mixer_;
  rclcpp::Subscription<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr
      torque_setpoint_sub_;
  rclcpp::Subscription<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr
      thrust_setpoint_sub_;
  rclcpp::Publisher<hippo_msgs::msg::ActuatorControls>::SharedPtr
      actuator_controls_pub_;

  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::array<double, mixer::InputChannels::kCount> inputs_;
  rclcpp::Time t_last_thrust_setpoint_;
  rclcpp::Time t_last_torque_setpoint_;
};
}  // namespace mixer
}  // namespace hippo_control
