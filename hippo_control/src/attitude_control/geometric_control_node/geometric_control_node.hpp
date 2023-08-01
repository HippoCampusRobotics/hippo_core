#pragma once

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <hippo_msgs/msg/actuator_setpoint.hpp>
#include <hippo_msgs/msg/float64_stamped.hpp>
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
  void PublishZeroActuatorSetpoints(const rclcpp::Time &now);
  void PublishCurrentSetpoint(const rclcpp::Time &now,
                              const Eigen::Quaterniond &attitude);

  void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void OnHeadingTarget(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void OnRollTarget(const hippo_msgs::msg::Float64Stamped::SharedPtr msg) {
    roll_target_ = msg->data;
  }
  void SetControllerGains();

  void OnSetpointTimeout();

  rcl_interfaces::msg::SetParametersResult OnGainParameters(
      const std::vector<rclcpp::Parameter> &parameters);

  //////////////////////////////////////////////////////////////////////////////
  // publishers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr torque_pub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr
      current_setpoint_pub_;

  //////////////////////////////////////////////////////////////////////////////
  // subscriptions
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      heading_target_sub_;
  rclcpp::Subscription<hippo_msgs::msg::Float64Stamped>::SharedPtr
      roll_target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  OnSetParametersCallbackHandle::SharedPtr gain_params_cb_handle_;

  bool setpoint_timed_out_{false};
  rclcpp::TimerBase::SharedPtr setpoint_timeout_timer_;

  Eigen::Vector3d heading_target_{0.0, 0.0, 0.0};
  double roll_target_{0.0};

  GeometricController controller_;

  Params params_;
};
}  // namespace attitude_control
}  // namespace hippo_control
