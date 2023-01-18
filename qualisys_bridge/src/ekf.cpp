#include "qualisys_bridge/ekf.hpp"

#include <eigen3/Eigen/SVD>
#include <rclcpp/rclcpp.hpp>

namespace qualisys_bridge {
Ekf::Ekf()
    : orientation_(Eigen::Quaterniond::Identity()),
      position_(Eigen::Vector3d::Zero()),
      velocity_angular_(Eigen::Vector3d::Zero()),
      velocity_linear_(Eigen::Vector3d::Zero()),
      acceleration_linear_(Eigen::Vector3d::Zero()),
      state_covariance_(Matrix15d::Zero()),
      process_covariance_(Matrix15d::Zero()),
      measurement_covariance_(Matrix6d::Zero()),
      process_jacobian_(Matrix15d::Identity()),
      measurement_jacobian_(Matrix6_15d::Zero()),
      process_noise_jacobian_(Matrix15d::Identity()),
      measurement_noise_jacobian_(Matrix6d::Identity()),
      time_last_(0.0),
      expected_update_interval_(0.01) {
  measurement_jacobian_.leftCols<6>() = Matrix6d::Identity();
}

bool Ekf::Init(const Matrix15d &_process_covariance,
               const Matrix6d &_measurement_covariance, int _frequency) {
  bool is_okay{true};
  Eigen::JacobiSVD<Matrix15d> process_svd{_process_covariance};
  Eigen::JacobiSVD<Matrix6d> measurement_svd{_measurement_covariance};
  Vector15d process_sv{process_svd.singularValues()};
  Vector6d measurement_sv{measurement_svd.singularValues()};
  // singular values are in decreasing order. last element is the smallest.
  const double smallest_process_sv = process_sv.col(0).tail<1>()[0];
  if (smallest_process_sv < 1e-10) {
    is_okay = false;
    RCLCPP_ERROR(rclcpp::get_logger("qualisys_bridge"),
                 "Process covariance is almost singular (%lf)",
                 smallest_process_sv);
  } else {
    process_covariance_ = _process_covariance;
  }

  const double smallest_measurement_sv = measurement_sv.col(0).tail<1>()[0];
  if (smallest_measurement_sv < 1e-10) {
    is_okay = false;
    RCLCPP_ERROR(rclcpp::get_logger("qualisys_bridge"),
                 "Measurement covariance is almost singular (%lf)",
                 smallest_measurement_sv);
  } else {
    measurement_covariance_ = _measurement_covariance;
  }

  if (_frequency <= 0) {
    is_okay = false;
    RCLCPP_ERROR(rclcpp::get_logger("qualisys_bridge"),
                 "Invalid frequency for Ekf %d", _frequency);
  } else {
    expected_update_interval_ = 1.0 / static_cast<double>(_frequency);
  }
  return is_okay;
}

bool Ekf::SetInitialCondition(
    double _time, const Eigen::Quaterniond &_orientation_measurement,
    const Eigen::Vector3d &_position_measurement) {
  double dt = std::max(_time - time_last_, 0.9 * expected_update_interval_);
  time_last_ = _time;

  // compute delta quaternion and its representation as angle-axis. compute
  // angular velocity by difference quotient.
  Eigen::Quaterniond delta_q =
      _orientation_measurement * orientation_.inverse();
  Eigen::AngleAxisd delta_angle_axis{delta_q};
  Eigen::Vector3d v_ang_current =
      delta_angle_axis.axis() * delta_angle_axis.angle() / dt;

  // compute linear acceleratino by applying a differential quotient.
  Eigen::Vector3d delta_position = _position_measurement - position_;
  Eigen::Vector3d v_lin_current = delta_position / dt;
  switch (state_) {
    case State::INIT_POSE:
      orientation_ = _orientation_measurement;
      position_ = _position_measurement;
      state_covariance_ = Matrix15d::Identity();
      state_ = State::INIT_TWIST;
      return true;
    case State::INIT_TWIST:
      orientation_ = _orientation_measurement;
      position_ = _position_measurement;
      velocity_angular_ = v_ang_current;
      velocity_linear_ = v_lin_current;
      state_covariance_ = Matrix15d::Identity();
      state_ = State::INIT_ACCEL;
      return true;
    case State::INIT_ACCEL:
      acceleration_linear_ = (v_lin_current - velocity_linear_) / dt;
      orientation_ = _orientation_measurement;
      position_ = _position_measurement;
      velocity_angular_ = v_ang_current;
      velocity_linear_ = v_lin_current;
      state_covariance_ = Matrix15d::Identity();
      state_ = State::READY;
      return true;
    case State::READY:
      return false;
    default:
      return false;
  }
  return false;
}

void Ekf::Predict(double _time) {
  double dt = std::max(_time - time_last_, 0.9 * expected_update_interval_);
  time_last_ = _time;

  Eigen::Vector3d v_lin_old = velocity_linear_;
  velocity_linear_ += acceleration_linear_ * dt;
  position_ += 0.5 * (v_lin_old + velocity_linear_) * dt;

  // predict orientation by rotating by delta quaternion
  Eigen::Vector3d delta_omega = velocity_angular_ * dt;
  double delta_angle = delta_omega.norm();
  Eigen::Vector3d axis = delta_omega / delta_angle;
  Eigen::Quaterniond delta_q{Eigen::AngleAxisd{delta_angle, axis}};
  orientation_ = delta_q * orientation_;

  // jacobian for the error angles is the cross product matrix of delta_omega
  process_jacobian_(0, 1) = delta_omega(2);
  process_jacobian_(0, 2) = -delta_omega(1);
  process_jacobian_(1, 0) = -delta_omega(2);
  process_jacobian_(1, 2) = delta_omega(0);
  process_jacobian_(2, 0) = delta_omega(1);
  process_jacobian_(2, 1) = -delta_omega(0);
  // delta_omega = omega * dt
  process_jacobian_(0, 6) = dt;
  process_jacobian_(1, 7) = dt;
  process_jacobian_(2, 8) = dt;
  // p_new = p + v * dt * 0.5 * a * dt^2
  process_jacobian_(3, 9) = dt;
  process_jacobian_(3, 12) = 0.5 * dt * dt;
  process_jacobian_(4, 10) = dt;
  process_jacobian_(4, 13) = 0.5 * dt * dt;
  process_jacobian_(5, 11) = dt;
  process_jacobian_(5, 14) = 0.5 * dt * dt;
  // v_new = v + dt * a
  process_jacobian_(9, 12) = dt;
  process_jacobian_(10, 13) = dt;
  process_jacobian_(11, 14) = dt;

  // P = F * P * F_transposed + Noise Covariance
  state_covariance_ =
      process_jacobian_ * state_covariance_ * process_jacobian_.transpose() +
      process_covariance_;
}

void Ekf::Update(const Eigen::Quaterniond &_orientation_measurement,
                 const Eigen::Vector3d &_position_measurement) {
  Eigen::Quaterniond residual_q =
      _orientation_measurement * orientation_.inverse();
  Eigen::AngleAxisd residual_angle_axis{residual_q};

  double angle = residual_angle_axis.angle();
  // choose shortest rotation
  if (std::abs(angle) > std::abs(2 * M_PI - angle)) {
    residual_angle_axis.angle() = 2 * M_PI - angle;
    residual_angle_axis.axis() = -residual_angle_axis.axis();
  }

  residual_q = Eigen::Quaterniond{residual_angle_axis};
  Eigen::Vector3d residual_theta{residual_q.vec() * 2.0};
  Eigen::Vector3d residual_position = _position_measurement - position_;
  Vector6d residual;
  residual.head<3>() = residual_theta;
  residual.tail<3>() = residual_position;

  // error states are assumed to be measured directly. The jacobian is therefore
  // identity.
  Matrix6d S =
      measurement_covariance_ + state_covariance_.topLeftCorner<6, 6>();
  Matrix15_6d K = state_covariance_.leftCols<6>() * S.inverse();

  Vector15d correction = K * residual;

  Eigen::Vector3d theta_correction{correction.head<3>() * 0.5};
  Eigen::Quaterniond orientation_correction{
      1.0, theta_correction.x(), theta_correction.y(), theta_correction.z()};
  orientation_correction.normalize();

  orientation_ = orientation_correction * orientation_;
  position_ += correction.segment<3>(3);
  velocity_angular_ += correction.segment<3>(6);
  velocity_linear_ += correction.segment<3>(9);
  acceleration_linear_ += correction.segment<3>(12);

  state_covariance_ =
      (Matrix15d::Identity() - K * measurement_jacobian_) * state_covariance_;
}

}  // namespace qualisys_bridge
