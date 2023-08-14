#pragma once
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <hippo_control/attitude_control/quaternion_controller.hpp>
#include <hippo_msgs/msg/actuator_controls.hpp>
// #include <hippo_msgs/msg/heading_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hippo_control {
namespace attitude_control {

class QuaternionControlNode : public rclcpp::Node {
 public:
  explicit QuaternionControlNode(rclcpp::NodeOptions const &_options);

 private:
  struct Params {
    bool updated_{false};
    double gain;
    double roll_weight;
  };
  void DeclareParams();
  void InitPublishers();
  void InitSubscriptions();
  inline void SetControllerGains() {
    attitude_controller_.gain() = params_.gain;
    attitude_controller_.roll_weight() = params_.roll_weight;
  }
  void PublishAngularVelocitySetpoint(const rclcpp::Time &now,
                                      const Eigen::Vector3d &velocity);

  //////////////////////////////////////////////////////////////////////////////
  // Callbacks
  //////////////////////////////////////////////////////////////////////////////
  //   void OnHeadingTarget(const hippo_msgs::msg::HeadingTarget::SharedPtr);
  void OnHeadingTarget(const geometry_msgs::msg::Vector3Stamped::SharedPtr);
  void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr);

  rcl_interfaces::msg::SetParametersResult OnParameters(
      const std::vector<rclcpp::Parameter> &parameters);

  //////////////////////////////////////////////////////////////////////////////
  // Publishers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      angular_velocity_setpoint_pub_;

  //////////////////////////////////////////////////////////////////////////////
  // Subscriptions
  //////////////////////////////////////////////////////////////////////////////
  //   rclcpp::Subscription<hippo_msgs::msg::HeadingTarget>::SharedPtr
  //       heading_target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      heading_target_sub_;

  Eigen::Quaterniond orientation_{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d target_heading_{1.0, 0.0, 0.0};
  QuaternionController attitude_controller_;

  OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;

  Params params_;
};

}  // namespace attitude_control
}  // namespace hippo_control