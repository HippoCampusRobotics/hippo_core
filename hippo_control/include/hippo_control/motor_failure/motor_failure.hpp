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

class MotorFailure {
 public:
 static constexpr double kMotorOffset = 0.069;
  Eigen::Vector4d Update(double pitch_rate, double yaw_rate,
                         double surge_velocity,
                         const Eigen::Quaterniond &_orientation);
  void SetTarget(double pitch_rate, double yaw_rate, double surge_velocity);

 private:
  Eigen::Vector3d ComputeThrusts(double surge_velocity, double surge_accel,
                                 double pitch_velocity, double pitch_accel,
                                 double yaw_velocity, double yaw_accel);
  Eigen::Vector4d AllocateThrust(const Eigen::Vector3d &_thrust);
  double pitch_rate_target_{0.0};
  double yaw_rate_target_{0.0};
  double surge_velocity_target_{0.0};

  double surge_p_gain_{1.0};
  double pitch_p_gain_{2.0};
  double yaw_p_gain_{2.0};

  double mass_{3.42};
  double surge_damping_linear_{5.39};
  double pitch_damping_linear_{0.007};
  double yaw_damping_linear_{0.007};
  double pitch_inertia_{0.027};
  double yaw_inertia_{0.027};
};
// TODO
}  // namespace motor_failure
}  // namespace hippo_control
