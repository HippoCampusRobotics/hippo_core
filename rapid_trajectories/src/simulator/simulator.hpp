#pragma once
#include <builtin_interfaces/msg/time.hpp>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <hippo_msgs/msg/actuator_setpoint.hpp>
#include <hippo_msgs/msg/rates_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace rapid_trajectories {
namespace simulator {
class Simulator : public rclcpp::Node {
 public:
  explicit Simulator(rclcpp::NodeOptions const &_options);

 private:
  struct Params {
    double damping{5.4};
    double mass{2.6};
    int timestep_ms{1};
    double speed_factor{1.0};
  } params_;
  void DeclareParams();

  void InitPublishers();
  void InitSubscriptions();

  //////////////////////////////////////////////////////////////////////////////
  // message callbacks
  //////////////////////////////////////////////////////////////////////////////
  void OnThrustSetpoint(
      const hippo_msgs::msg::ActuatorSetpoint::SharedPtr _msg);
  void OnRatesSetpoint(const hippo_msgs::msg::RatesTarget::SharedPtr _msg);

  rcl_interfaces::msg::SetParametersResult OnSetParams(
      const std::vector<rclcpp::Parameter> &_parameters);

  void CreateUpdateTimer();
  void PublishState();
  void PublishOdometry();
  void PublishLinearAcceleration();
  void Update();
  void UpdateState(double _dt);
  std::chrono::microseconds UpdatePeriod();
  Eigen::Quaterniond DeltaRotation(const Eigen::Vector3d &_v, double _dt);
  //////////////////////////////////////////////////////////////////////////////
  // publishers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      linear_acceleration_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  //////////////////////////////////////////////////////////////////////////////
  // subscriptions
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Subscription<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr
      thrust_setpoint_sub_;
  rclcpp::Subscription<hippo_msgs::msg::RatesTarget>::SharedPtr
      rates_setpoint_sub_;

  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::Time t_now_;

  OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;

  Eigen::Vector3d position_{0.0, 0.0, -0.5};
  Eigen::Vector3d velocity_{0.5, 1.0, 0.0};
  Eigen::Vector3d acceleration_{1.0, 1.0, 0.0};

  Eigen::Vector3d body_rates_{0.0, 0.0, 0.0};
  Eigen::Vector3d thrust_local_{0.0, 0.0, 0.0};
  Eigen::Quaterniond orientation_{1.0, 0.0, 0.0, 0.0};
};
}  // namespace simulator
}  // namespace rapid_trajectories
