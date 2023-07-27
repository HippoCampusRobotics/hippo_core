#include "hippo_control/attitude_control/quaternion_controller.hpp"

#include <cmath>

namespace hippo_control {
namespace attitude_control {

Eigen::Quaterniond QuaternionController::ReducedQuaternionCommand(
    const Eigen::Vector3d &_desired_heading) {
  auto current_heading = CurrentForwardAxis();
  auto alpha = acos(current_heading.dot(_desired_heading));
  Eigen::Quaterniond q_error;
  q_error.vec() = sin(alpha * 0.5) * current_heading.cross(_desired_heading);
  q_error.w() = cos(alpha * 0.5);
  return orientation_ * q_error;
}

Eigen::Quaterniond QuaternionController::MixedQuaternionCommand(
    const Eigen::Vector3d &_heading, double _roll) {
  auto q_mix = ReducedQuaternionCommand(_heading).inverse() *
               FullQuaternionCommand(_heading, _roll);
  double tmp = acos(q_mix.w());
  return ReducedQuaternionCommand(_heading) *
         Eigen::Quaterniond{cos(roll_weight_ * tmp), 0.0, 0.0,
                            sin(roll_weight_ * tmp)};
}
}  // namespace attitude_control
}  // namespace hippo_control
