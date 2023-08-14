#pragma once

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <hippo_control/motor_failure/motor_failure.hpp>
#include <hippo_control/thruster_model.hpp>
#include <hippo_msgs/msg/actuator_controls.hpp>
#include <hippo_msgs/msg/actuator_setpoint.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <hippo_msgs/msg/failure_control_debug.hpp>

namespace hippo_control {
namespace motor_failure {

class ControlNode : public rclcpp::Node {
 public:
  explicit ControlNode(const rclcpp::NodeOptions &_options);

 private:
  struct DOFs {
    double surge;
    double pitch;
    double yaw;
  };
  struct Model {
    struct {
      DOFs linear;
    } damping;
    DOFs inertia;
  };
  struct Gains {
    DOFs p;
  };
  struct Params {
    Gains gains;
    Model model;
  };
  void DeclareParams();
  void InitPublishers();
  void InitSubscriptions();
  void PublishThrusterCommand(const rclcpp::Time &now, Eigen::Vector4d &cmds);
  void SetControllerGains();
  void SetControllerModel();
  void PublishDebugMsg(const rclcpp::Time &now, double controllability_scaler);

  void OnThrustSetpoint(const hippo_msgs::msg::ActuatorSetpoint::SharedPtr);
  void OnAngularVelocitySetpoint(
      const geometry_msgs::msg::Vector3Stamped::SharedPtr);
  void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr);

  rcl_interfaces::msg::SetParametersResult OnParameters(
      const std::vector<rclcpp::Parameter> &parameters);

  //////////////////////////////////////////////////////////////////////////////
  // Publishers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<hippo_msgs::msg::ActuatorControls>::SharedPtr
      actuator_controls_pub_;
  rclcpp::Publisher<hippo_msgs::msg::FailureControlDebug>::SharedPtr debug_pub_;
  //////////////////////////////////////////////////////////////////////////////
  // Subscriptions
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Subscription<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr
      thrust_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      angular_velocity_setpoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;

  Params params_;
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
