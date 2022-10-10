#include "hippo_control/attitude_control/geometric_attitude_control.hpp"

#include <cmath>

namespace hippo_control {
namespace attitude_control {
Eigen::Vector3d GeometricAttitudeControl::Update(
    const Eigen::Quaterniond &_orientation,
    const Eigen::Vector3d &_angular_velocity, bool _scale) {
  Eigen::Matrix3d R = _orientation.toRotationMatrix();
  Eigen::Matrix3d R_desired = orientation_target_.toRotationMatrix();
  Eigen::Matrix3d R_error =
      0.5 * (R_desired.transpose() * R - R.transpose() * R_desired);

  // use array instead of vector to simplify coefficient-wise operations
  Eigen::Array3d R_error_vector{R_error(1, 2), R_error(2, 0), R_error(0, 1)};

  Eigen::Array3d angular_velocity_error{v_angular_target_ - _angular_velocity};

  Eigen::Array3d torque;
  torque = p_gains_ * R_error_vector + d_gains_ * angular_velocity_error;

  if (_scale) {
    double scaler = (torque.abs().maxCoeff());
    if (scaler > 1.0) {
      torque /= scaler;
    }
  } else {
    torque = torque.min(1.0).max(-1.0);
  }

  return Eigen::Vector3d{torque};
}
void GeometricAttitudeControl::SetOrientationTarget(const double _roll,
                                                    const double _pitch,
                                                    const double _yaw) {
  orientation_target_ = Eigen::AngleAxisd(_yaw, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(_pitch, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(_roll, Eigen::Vector3d::UnitX());
}
}  // namespace attitude_control

}  // namespace hippo_control
