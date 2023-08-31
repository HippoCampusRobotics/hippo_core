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

  Eigen::Vector<double, 4> motor_thrusts;
  Eigen::Vector2d control_direction =
      Eigen::Vector2d{pitch_accel, yaw_accel}.normalized();
  // in case motors 0 and 2 are working
  Eigen::Vector2d controllability_direction{1.0, -1.0};
  double controllability{1.0};

  switch (mode_) {
    case mode::kIdle:
    case mode::kUnset:
      motor_thrusts = Eigen::Vector4d::Zero();
      break;
    case mode::kUntangling: {
      motor_thrusts = Eigen::Vector4d::Ones() * 0.3;
      break;
    }
    case mode::kSingleFailureUndetected:
    case mode::kSingleFailureDetected: {
      torque_force_vec_ = ComputeThrusts(_surge_velocity, surge_accel, _pitch_rate,
                               pitch_accel, _yaw_rate, yaw_accel);
      motor_thrusts = AllocateThrust(torque_force_vec_);
      // disable the fourth motor to simulate motor failure
      motor_thrusts(3) = 0.0;
      break;
    }
    case mode::kDoubleFailureUndetected: {
      torque_force_vec_ = ComputeThrusts(_surge_velocity, surge_accel, _pitch_rate,
                               pitch_accel, _yaw_rate, yaw_accel);
      motor_thrusts(1) = 0.0;
      motor_thrusts(3) = 0.0;
      break;
    }
    case mode::kDoubleFailureDetected: {
      controllability = control_direction.dot(controllability_direction) /
                        controllability_direction.squaredNorm();
      double torque = Eigen::Vector2d{pitch_accel, yaw_accel}.dot(
                          controllability_direction) /
                      controllability_direction.squaredNorm();
      // only apply torque if magnitude of controllability is greater 0.5
      // this should result in more wanted than unwanted rotation.
      if (controllability < 0.5 && controllability > -0.5) {
        torque = 0.0;
      }
      // TODO: verify that it is okay so set zero torque. thrusts can be
      // computed by hand otherwise.
      torque_force_vec_ = ComputeThrusts(_surge_velocity, surge_accel, _pitch_rate,
                               torque, _yaw_rate, -torque);
      torque_force_vec_(2) = -torque_force_vec_(1);
      motor_thrusts = AllocateThrust(torque_force_vec_);
      break;
    }

    case mode::kNormal: {
      torque_force_vec_ = ComputeThrusts(_surge_velocity, surge_accel, _pitch_rate,
                               pitch_accel, _yaw_rate, yaw_accel);
      motor_thrusts = AllocateThrust(torque_force_vec_);
      break;
    }
  }
  // store controllability for introspection
  controllability_ = controllability;
  return motor_thrusts;
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
  // thrusts(3) = surge_inertia_ * _surge_accel + surge_damping_linear_ *
  // _surge_velocity;
  thrusts(3) = _surge_accel;
  return thrusts;
}

Eigen::Vector<double, 4> MotorFailure::AllocateThrust(
    const Eigen::Vector<double, 6> &_thrust) {
  Eigen::Vector<double, 4> result =
      mixer_matrix_.colPivHouseholderQr().solve(_thrust);
  return result;
}

Eigen::Matrix<double, 6, 4> MotorFailure::FullMixerMatrix() const {
  Eigen::Matrix<double, 6, 4> A;
  A.row(0) = Eigen::Vector<double, 4>{kTorqueFactor, kTorqueFactor,
                                      kTorqueFactor, kTorqueFactor};
  A.row(1) = Eigen::Vector<double, 4>{-kMotorOffset, kMotorOffset, kMotorOffset,
                                      -kMotorOffset};
  A.row(2) = Eigen::Vector<double, 4>{kMotorOffset, kMotorOffset, -kMotorOffset,
                                      -kMotorOffset};
  A.row(3) = Eigen::Vector<double, 4>{-1.0, 1.0, -1.0, 1.0};
  // no lateral thrust possible for hippocampus
  A.row(4).setZero();
  // no vertical thrust possible for hippocampus
  A.row(5).setZero();
  return A;
}

void MotorFailure::UpdateMixerMatrix() {
  mixer_matrix_ = FullMixerMatrix();
  switch (mode_) {
    case mode::kSingleFailureDetected:
      mixer_matrix_.row(0).setZero();
      // remove the 3rd motor
      mixer_matrix_.col(3).setZero();
      break;
    case mode::kDoubleFailureDetected:
      mixer_matrix_.row(0).setZero();
      // remove 1st and 3rd motor, to use only 0st and 2nd motor.
      mixer_matrix_.col(1).setZero();
      mixer_matrix_.col(3).setZero();
      break;
    default:
      break;
  }
}
}  // namespace motor_failure
}  // namespace hippo_control
