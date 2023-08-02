#include "hippo_control/motor_failure/motor_failure.hpp"

namespace hippo_control {
namespace motor_failure {

Eigen::Vector4d MotorFailure::Update(double _pitch_rate, double _yaw_rate,
                                     double _surge_velocity,
                                     const Eigen::Quaterniond &_orientation) {
  double surge_accel =
      surge_p_gain_ * (surge_velocity_target_ - _surge_velocity);

  double pitch_accel = pitch_p_gain_ * (pitch_rate_target_ - _pitch_rate);
  double yaw_accel = yaw_p_gain_ * (yaw_rate_target_ - _yaw_rate);

  Eigen::Vector3d thrusts =
      ComputeThrusts(_surge_velocity, surge_accel, _pitch_rate, pitch_accel,
                     _yaw_rate, yaw_accel);
  return AllocateThrust(thrusts);
}

void MotorFailure::SetTarget(double pitch_rate, double yaw_rate,
                             double surge_velocity) {
  pitch_rate_target_ = pitch_rate;
  yaw_rate_target_ = yaw_rate;
  surge_velocity_target_ = surge_velocity;
}

Eigen::Vector3d MotorFailure::ComputeThrusts(
    double _surge_velocity, double _surge_accel, double _pitch_velocity,
    double _pitch_accel, double _yaw_velocity, double _yaw_accel) {
  Eigen::Vector3d thrusts;
  thrusts(0) = mass_ * _surge_accel + surge_damping_linear_ * _surge_velocity;
  thrusts(1) =
      pitch_inertia_ * _pitch_accel + pitch_damping_linear_ * _pitch_velocity;
  thrusts(2) = yaw_inertia_ * _yaw_accel + yaw_damping_linear_ * _yaw_velocity;
  return thrusts;
}

Eigen::Vector4d MotorFailure::AllocateThrust(const Eigen::Vector3d &_thrust) {
  Eigen::Matrix3d A;
  A << 1.0, 1.0, 1.0, kMotorOffset, kMotorOffset, -kMotorOffset, -kMotorOffset,
      kMotorOffset, kMotorOffset;
  Eigen::Vector3d thrusts = A.colPivHouseholderQr().solve(_thrust);
  Eigen::Vector4d result;
  result(0) = thrusts(0);
  result(1) = thrusts(1);
  result(2) = thrusts(2);
  result(3) = 0.0;
  return result;
}
}  // namespace motor_failure
}  // namespace hippo_control
