#include "hippo_control/motor_failure/motor_failure.hpp"

#include <iostream>
namespace hippo_control {
namespace motor_failure {

Eigen::Vector<double, 4> MotorFailure::Update(
    const Eigen::Vector3d &_rates, double _surge_velocity,
    const Eigen::Quaterniond &_orientation) {
  double surge_accel =
      surge_p_gain_ * (surge_velocity_target_ - _surge_velocity);

  Eigen::Vector3d angular_accel{roll_p_gain_ * (rate_target_.x() - _rates.x()),
                                pitch_p_gain_ * (rate_target_.y() - _rates.y()),
                                yaw_p_gain_ * (rate_target_.z() - _rates.z())};

  Eigen::Vector<double, 4> motor_thrusts;
  Eigen::Vector2d control_direction =
      Eigen::Vector2d{angular_accel.y(), angular_accel.z()}.normalized();
  // in case motors 0 and 2 are working
  Eigen::Vector2d controllability_direction{1.0, -1.0};
  double controllability{1.0};

  switch (mode_) {
    case mode::kIdle:
    case mode::kUnset:
      motor_thrusts = Eigen::Vector4d::Zero();
      break;
    case mode::kUntangling: {
      motor_thrusts = Eigen::Vector4d::Ones() * 0.8;
      break;
    }
    case mode::kSingleFailureUndetected:
    case mode::kSingleFailureDetected: {
      torque_force_vec_ =
          ComputeThrusts(_surge_velocity, surge_accel, _rates, angular_accel);
      motor_thrusts = AllocateThrust(torque_force_vec_);
      // disable the fourth motor to simulate motor failure
      motor_thrusts(3) = 0.0;
      break;
    }
    case mode::kDoubleFailureUndetected: {
      torque_force_vec_ =
          ComputeThrusts(_surge_velocity, surge_accel, _rates, angular_accel);
      motor_thrusts(1) = 0.0;
      motor_thrusts(3) = 0.0;
      break;
    }
    case mode::kDoubleFailureDetected: {
      controllability = control_direction.dot(controllability_direction) /
                        controllability_direction.squaredNorm();
      double torque = Eigen::Vector2d{angular_accel.y(), angular_accel.z()}.dot(
                          controllability_direction) /
                      controllability_direction.squaredNorm();
      // only apply torque if magnitude of controllability is greater 0.5
      // this should result in more wanted than unwanted rotation.
      if (false && controllability < 0.5 && controllability > -0.5) {
        torque = 0.0;
      }
      // TODO: verify that it is okay so set zero torque. thrusts can be
      // computed by hand otherwise.
      angular_accel.x() = 0.0;
      angular_accel.y() = torque;
      angular_accel.z() = -torque;
      torque_force_vec_ =
          ComputeThrusts(_surge_velocity, surge_accel, _rates, angular_accel);
      torque_force_vec_(2) = -torque_force_vec_(1);
      motor_thrusts = AllocateThrust(torque_force_vec_);
      break;
    }

    case mode::kNormal: {
      torque_force_vec_ =
          ComputeThrusts(_surge_velocity, surge_accel, _rates, angular_accel);
      motor_thrusts = AllocateThrust(torque_force_vec_);
      break;
    }
  }
  // store controllability for introspection
  controllability_ = controllability;
  return motor_thrusts;
}

void MotorFailure::SetTarget(const Eigen::Vector3d &_angular_velocity,
                             double surge_velocity) {
  rate_target_ = _angular_velocity;
  surge_velocity_target_ = surge_velocity;
}

Eigen::Vector<double, 6> MotorFailure::ComputeThrusts(
    double _surge_velocity, double _surge_accel,
    const Eigen::Vector3d &_angular_velocity,
    const Eigen::Vector3d &_angular_accel) {
  Eigen::Vector<double, 6> thrusts = Eigen::Vector<double, 6>::Zero();
  for (int i = 0; i < 3; ++i) {
    thrusts(i) = rotational_inertia_(i) * _angular_accel(i) +
                 rotational_damping_linear_(i) * _angular_velocity(i);
  }
  switch (mode_) {
    case mode::kSingleFailureDetected:
    case mode::kDoubleFailureDetected:
      // do not waste effort for roll rate, since we cannot control it.
      thrusts(0) = 0.0;
      break;
    default:
      break;
  }
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
  Eigen::Matrix<double, 4, 6> mixer_inv =
      mixer_matrix_.completeOrthogonalDecomposition().pseudoInverse();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      thrust_composition_[i](j) = mixer_inv(i, j) * _thrust(j) / result(i);
    }
  }
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
