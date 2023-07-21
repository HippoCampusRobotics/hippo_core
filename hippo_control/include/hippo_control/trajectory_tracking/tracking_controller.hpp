#pragma once

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <hippo_common/convert.hpp>
#include <hippo_common/param_utils.hpp>
#include <hippo_common/tf2_utils.hpp>
#include <hippo_msgs/msg/attitude_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "hippo_control/trajectory_tracking/tracking_control.hpp"

namespace hippo_control {
namespace trajectory_tracking {
class TrackingControllerNode : public rclcpp::Node {
 public:
  explicit TrackingControllerNode(rclcpp::NodeOptions const &_options);

 private:
  struct Params {
    bool updated{false};
    double position_gain{1.0};
    double velocity_gain{1.0};
  };
  void DeclareParams();
  void InitPublishers();
  void InitSubscriptions();
  void UpdateControllerParams();
  void PublishAttitudeTarget(const Eigen::Quaterniond &_attitude,
                             double _thrust, const rclcpp::Time &_now);

  void OnRollTarget(const std_msgs::msg::Float64::SharedPtr _msg) {
    roll_target_ = _msg->data;
  }
  void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr _msg);
  void OnPositionTarget(
      const geometry_msgs::msg::PointStamped::SharedPtr _msg) {
    hippo_common::convert::RosToEigen(_msg->point, position_target_);
  }

  rcl_interfaces::msg::SetParametersResult OnParams(
      const std::vector<rclcpp::Parameter> &_parameters);
  void OnVelocityTarget(
      const geometry_msgs::msg::Vector3Stamped::SharedPtr _msg) {
    hippo_common::convert::RosToEigen(_msg->vector, velocity_target_);
  }

  OnSetParametersCallbackHandle::SharedPtr params_cb_handle_{nullptr};

  rclcpp::Publisher<hippo_msgs::msg::AttitudeTarget>::SharedPtr
      attitude_target_pub_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr roll_target_sub_{
      nullptr};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_{
      nullptr};
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      velocity_target_sub_{nullptr};
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      position_target_sub_{nullptr};
  Params params_;
  TrackingController controller_;

  Eigen::Vector3d position_{0.0, 0.0, 0.0};
  Eigen::Vector3d velocity_{0.0, 0.0, 0.0};
  Eigen::Quaterniond attitude_{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d position_target_{0.0, 0.0, 0.0};
  Eigen::Vector3d velocity_target_{0.0, 0.0, 0.0};
  Eigen::Quaterniond attitude_target_{1.0, 0.0, 0.0, 0.0};
  double thrust_{0.0};
  double roll_target_{0.0};
};
}  // namespace trajectory_tracking
}  // namespace hippo_control
