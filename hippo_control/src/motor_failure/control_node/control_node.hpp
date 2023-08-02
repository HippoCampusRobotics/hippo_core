#pragma once

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <hippo_control/motor_failure/motor_failure.hpp>
#include <hippo_control/thruster_model.hpp>
#include <hippo_msgs/msg/actuator_controls.hpp>
#include <hippo_msgs/msg/actuator_setpoint.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hippo_control {
namespace motor_failure {

class ControlNode : public rclcpp::Node {
 public:
  explicit ControlNode(const rclcpp::NodeOptions &_options);

 private:
  void DeclareParams();
  void InitPublishers();
  void InitSubscriptions();
  void PublishThrusterCommand(const rclcpp::Time &now, Eigen::Vector4d &cmds);

  void OnThrustSetpoint(const hippo_msgs::msg::ActuatorSetpoint::SharedPtr);
  void OnAngularVelocitySetpoint(const geometry_msgs::msg::Vector3Stamped::SharedPtr);
  void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr);

  //////////////////////////////////////////////////////////////////////////////
  // Publishers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<hippo_msgs::msg::ActuatorControls>::SharedPtr
      actuator_controls_pub_;
  //////////////////////////////////////////////////////////////////////////////
  // Subscriptions
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Subscription<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr
      thrust_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      angular_velocity_setpoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  Eigen::Vector3d angular_velocity_{0.0, 0.0, 0.0};
  Eigen::Vector3d linear_velocity_{0.0, 0.0, 0.0};
  Eigen::Vector3d angular_velocity_setpoint_{0.0, 0.0, 0.0};
  Eigen::Vector3d thrust_setpoint_{0.0, 0.0, 0.0};
  Eigen::Quaterniond orientation_{1.0, 0.0, 0.0, 0.0};

  MotorFailure controller_;
  hippo_control::ThrusterModel thruster_model_;
};

}  // namespace motor_failure
}  // namespace hippo_control
