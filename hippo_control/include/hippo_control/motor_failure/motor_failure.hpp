#pragma once

#include <array>
#include <eigen3/Eigen/Dense>

namespace hippo_control {
namespace motor_failure {
namespace failure_config {
namespace one {
static constexpr int kTopLeft = 0;
static constexpr int kTopRight = 1;
static constexpr int kBottomRight = 2;
static constexpr int kBottomLeft = 3;
}  // namespace one
namespace two {
static constexpr int kTopLeftToBottomRight = 0;
static constexpr int kTopRightToBottomLeft = 1;
}  // namespace two
}  // namespace failure_config

namespace mode {
enum Mode {
  kUnset = -1,
  kIdle = 0,
  kUntangling = 1,
  kNormal = 2,
  kSingleFailureUndetected = 3,
  kDoubleFailureUndetected = 4,
  kSingleFailureDetected = 5,
  kDoubleFailureDetected = 6,
};
}

class MotorFailure {
 public:
  static constexpr double kMotorOffset = 0.069;
  /// factor between force and motor induced torque caused by rotation
  static constexpr double kTorqueFactor = 0.1;
  Eigen::Vector<double, 4> Update(const Eigen::Vector3d &rates,
                                  double surge_velocity,
                                  const Eigen::Quaterniond &_orientation);
  void SetTarget(const Eigen::Vector3d &angular_velocity,
                 double surge_velocity);

  void SetPGains(double surge, double roll, double pitch, double yaw) {
    surge_p_gain_ = surge;
    roll_p_gain_ = roll;
    pitch_p_gain_ = pitch;
    yaw_p_gain_ = yaw;
  }
  void SetTranslationalDampingLinear(double damping) {
    surge_damping_linear_ = damping;
  }
  void SetRotationalDampingLinear(const Eigen::Vector3d &damping) {
    rotational_damping_linear_ = damping;
  }
  void SetTranslationalInertia(double inertia) { surge_inertia_ = inertia; }
  void SetRotationalInertia(const Eigen::Vector3d &inertia) {
    rotational_inertia_ = inertia;
  }
  void SetMode(mode::Mode _mode) {
    mode_ = _mode;
    UpdateMixerMatrix();
  }
  mode::Mode Mode() const { return mode_; };

  double Controllability() const { return controllability_; }
  Eigen::Vector3d DesiredTorque() const {
    return Eigen::Vector3d{torque_force_vec_.head(3)};
  }
  Eigen::Vector3d DesiredForce() const {
    return Eigen::Vector3d{torque_force_vec_.tail(3)};
  }

  Eigen::Vector4d ThrustCompositon(int i_motor) {
    return thrust_composition_[i_motor];
  }

 private:
  Eigen::Vector<double, 6> ComputeThrusts(
      double surge_velocity, double surge_accel,
      const Eigen::Vector3d &angular_velocity,
      const Eigen::Vector3d &angular_accel);
  Eigen::Vector<double, 4> AllocateThrust(
      const Eigen::Vector<double, 6> &thrust);
  Eigen::Matrix<double, 6, 4> FullMixerMatrix() const;
  void UpdateMixerMatrix();

  mode::Mode mode_{mode::Mode::kUnset};
  Eigen::Vector3d rate_target_{0.0, 0.0, 0.0};
  double surge_velocity_target_{0.0};

  double surge_p_gain_{1.0};
  double roll_p_gain_{2.0};
  double pitch_p_gain_{2.0};
  double yaw_p_gain_{2.0};

  double surge_inertia_{3.42};
  double surge_damping_linear_{5.39};
  Eigen::Vector3d rotational_damping_linear_{0.0, 0.007, 0.007};
  Eigen::Vector3d rotational_inertia_{0.0, 0.027, 0.02};
  Eigen::Matrix<double, 6, 4> mixer_matrix_;

  double controllability_{0.0};
  Eigen::Vector<double, 6> torque_force_vec_;
  std::array<Eigen::Vector4d, 4> thrust_composition_;
};
// TODO
}  // namespace motor_failure
}  // namespace hippo_control
