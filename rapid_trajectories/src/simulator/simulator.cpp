#include "simulator.hpp"

#include <chrono>
#include <hippo_common/convert.hpp>
#include <hippo_common/tf2_utils.hpp>
namespace rapid_trajectories {
namespace simulator {
Simulator::Simulator(rclcpp::NodeOptions const &_options)
    : Node("simulator", _options) {
  t_now_ = rclcpp::Time(0);
  DeclareParams();

  InitPublishers();
  CreateUpdateTimer();
  InitSubscriptions();

  RCLCPP_INFO(get_logger(), "Simulator started");
}
void Simulator::InitPublishers() {
  clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 20);
  linear_acceleration_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(
          "uuv00/acceleration", rclcpp::SensorDataQoS());
  odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "uuv00/odometry", rclcpp::SensorDataQoS());
}
void Simulator::InitSubscriptions() {
  std::string topic;

  topic = "/uuv00/thrust_setpoint";
  thrust_setpoint_sub_ = create_subscription<hippo_msgs::msg::ActuatorSetpoint>(
      topic, rclcpp::SensorDataQoS(),
      std::bind(&Simulator::OnThrustSetpoint, this, std::placeholders::_1));

  topic = "/uuv00/rates_setpoint";
  rates_setpoint_sub_ = create_subscription<hippo_msgs::msg::RatesTarget>(
      topic, rclcpp::SensorDataQoS(),
      std::bind(&Simulator::OnRatesSetpoint, this, std::placeholders::_1));
}

void Simulator::OnThrustSetpoint(
    const hippo_msgs::msg::ActuatorSetpoint::SharedPtr _msg) {
  thrust_local_.x() = _msg->x;
  thrust_local_.y() = _msg->y;
  thrust_local_.z() = _msg->z;
}

void Simulator::OnRatesSetpoint(
    const hippo_msgs::msg::RatesTarget::SharedPtr _msg) {
  body_rates_.x() = _msg->roll;
  body_rates_.y() = _msg->pitch;
  body_rates_.z() = _msg->yaw;
}

void Simulator::PublishState() {
  PublishOdometry();
  PublishLinearAcceleration();
}

void Simulator::PublishOdometry() {
  nav_msgs::msg::Odometry odometry_msg;
  odometry_msg.header.stamp = t_now_;
  odometry_msg.header.frame_id =
      hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(position_, odometry_msg.pose.pose.position);
  hippo_common::convert::EigenToRos(orientation_,
                                    odometry_msg.pose.pose.orientation);
  hippo_common::convert::EigenToRos(velocity_, odometry_msg.twist.twist.linear);
  hippo_common::convert::EigenToRos(body_rates_,
                                    odometry_msg.twist.twist.angular);

  odometry_pub_->publish(odometry_msg);
}

void Simulator::PublishLinearAcceleration() {
  geometry_msgs::msg::Vector3Stamped accel_msg;
  accel_msg.header.stamp = t_now_;
  accel_msg.header.frame_id =
      hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(acceleration_, accel_msg.vector);

  linear_acceleration_pub_->publish(accel_msg);
}

void Simulator::CreateUpdateTimer() {
  if (update_timer_) {
    update_timer_->cancel();
  }
  update_timer_ =
      create_wall_timer(UpdatePeriod(), std::bind(&Simulator::Update, this));
}

void Simulator::Update() {
  CreateUpdateTimer();
  if (params_.paused) {
    return;
  }
  rclcpp::Duration dt(std::chrono::milliseconds(params_.timestep_ms));
  t_now_ += dt;
  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock = t_now_;
  clock_pub_->publish(clock_msg);
  UpdateState(dt.nanoseconds() * 1e-9);
  if (t_now_ - t_last_odometry_ >=
      rclcpp::Duration(std::chrono::milliseconds(4))) {
    t_last_odometry_ = t_now_;
    PublishState();
  }
}

void Simulator::UpdateState(double _dt) {
  orientation_ = orientation_ * DeltaRotation(body_rates_, _dt);
  orientation_.normalize();

  Eigen::Vector3d gravity{params_.gravity.x, params_.gravity.y,
                          params_.gravity.z};
  Eigen::Vector3d force_ = -params_.damping * velocity_ +
                           orientation_ * thrust_local_ +
                           gravity * params_.mass;
  acceleration_ = force_ / params_.mass;

  velocity_ += acceleration_ * _dt;

  position_ += 0.5 * acceleration_ * _dt * _dt + velocity_ * _dt;
}

Eigen::Quaterniond Simulator::DeltaRotation(const Eigen::Vector3d &_v,
                                            double _dt) {
  Eigen::Vector3d half = _v * _dt * 0.5;
  double l = half.norm();
  if (l > 0) {
    half *= sin(l) / l;
    return Eigen::Quaterniond(cos(l), half.x(), half.y(), half.z());
  } else {
    return Eigen::Quaterniond(1.0, half.x(), half.y(), half.z());
  }
}

std::chrono::microseconds Simulator::UpdatePeriod() {
  uint64_t update_period_us = (uint64_t)(params_.timestep_ms * 1000);
  update_period_us /= params_.speed_factor;
  return std::chrono::microseconds(update_period_us);
}
}  // namespace simulator
}  // namespace rapid_trajectories

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rapid_trajectories::simulator::Simulator)
