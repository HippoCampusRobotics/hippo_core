#include "hippo_control/motor_failure/motor_failure.hpp"
#include <iostream>
namespace hippo_control {
namespace motor_failure {

Eigen::Vector<double, 4> MotorFailure::Update(
    double _pitch_rate, double _yaw_rate, double _surge_velocity,
    const Eigen::Quaterniond &_orientation) {
  double surge_accel =
      surge_p_gain_ * (surge_velocity_target_ - _surge_velocity);

  double pitch_accel = pitch_p_gain_ * (pitch_rate_target_ - _pitch_rate);
  double yaw_accel = yaw_p_gain_ * (yaw_rate_target_ - _yaw_rate);

  Eigen::Vector2d control_direction{pitch_accel, yaw_accel};
  Eigen::Vector2d controllability_direction{1.0, -1.0};
  double controllability = control_direction.dot(controllability_direction) / controllability_direction.squaredNorm();

  double torque = controllability;

  Eigen::Vector<double, 6> thrusts =
      ComputeThrusts(_surge_velocity, surge_accel, _pitch_rate, torque,
                     _yaw_rate, -torque);
  thrusts(2) = -thrusts(1);
  
  return AllocateThrust1and3(thrusts);
}

void MotorFailure::SetTarget(double pitch_rate, double yaw_rate,
                             double surge_velocity) {
  pitch_rate_target_ = pitch_rate;
  yaw_rate_target_ = yaw_rate;
  surge_velocity_target_ = surge_velocity;
}

Eigen::Vector<double, 6> MotorFailure::ComputeThrusts(
    double _surge_velocity, double _surge_accel, double _pitch_velocity,
    double _pitch_accel, double _yaw_velocity, double _yaw_accel) {
  Eigen::Vector<double, 6> thrusts = Eigen::Vector<double, 6>::Zero();
  // pitch
  thrusts(1) =
      pitch_inertia_ * _pitch_accel + pitch_damping_linear_ * _pitch_velocity;
  // yaw
  thrusts(2) = yaw_inertia_ * _yaw_accel + yaw_damping_linear_ * _yaw_velocity;
  // surge thrust
  // thrusts(3) = surge_inertia_ * _surge_accel + surge_damping_linear_ * _surge_velocity;
  thrusts(3) = _surge_accel;
  return thrusts;
}

Eigen::Vector<double, 4> MotorFailure::AllocateThrust1and3(const Eigen::Vector<double, 6> &_thrust) {
  Eigen::Matrix<double, 6, 4> A;
  A.row(0).setZero();
  A.row(1) = Eigen::Vector<double, 4>{-kMotorOffset, kMotorOffset, kMotorOffset, -kMotorOffset};
  A.row(2) = Eigen::Vector<double, 4>{kMotorOffset, kMotorOffset, -kMotorOffset, -kMotorOffset};
  A.row(3) = Eigen::Vector<double, 4>{-1.0, 1.0, -1.0, 1.0};
  A.row(4).setZero();
  A.row(5).setZero();

  A.col(1).setZero();
  A.col(3).setZero();

  Eigen::Vector<double, 4> result = A.colPivHouseholderQr().solve(_thrust);
  return result;
}

Eigen::Vector<double, 4> MotorFailure::AllocateThrust(
    const Eigen::Vector<double, 6> &_thrust) {
      static int i=0;
      ++i;
  Eigen::Matrix<double, 6, 4> A;
  // A << kMotorOffset, -kMotorOffset, kMotorOffset, -1.0, 0.0, 0.0, kMotorOffset,
  //     kMotorOffset, kMotorOffset, 1.0, 0.0, 0.0, kMotorOffset, kMotorOffset,
  //     -kMotorOffset, -1.0, 0.0, 0.0, kMotorOffset, -kMotorOffset, -kMotorOffset,
  //     1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  //     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  // we do not care about roll
  A.row(0).setZero();
  A.row(1) = Eigen::Vector<double, 4>{-kMotorOffset, kMotorOffset, kMotorOffset, -kMotorOffset};
  A.row(2) = Eigen::Vector<double, 4>{kMotorOffset, kMotorOffset, -kMotorOffset, -kMotorOffset};
  A.row(3) = Eigen::Vector<double, 4>{-1.0, 1.0, -1.0, 1.0};
  A.row(4).setZero();
  A.row(5).setZero();

  // we remove thcolourth thruster
  A.col(3).setZero();

  Eigen::Vector<double, 4> result = A.colPivHouseholderQr().solve(_thrust);
  if (i > 50) {
    i=0;
    std::cout << "\nA:\n" << A << "\nx:\n" << result << "\nb:\n" << _thrust << std::endl;
  }
  return result;
}
}  // namespace motor_failure
}  // namespace hippo_control
