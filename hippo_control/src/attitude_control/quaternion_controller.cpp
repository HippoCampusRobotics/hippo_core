// Copyright (C) 2023 Thies Lennart Alff
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
// USA

#include "hippo_control/attitude_control/quaternion_controller.hpp"

#include <cmath>

namespace hippo_control {
namespace attitude_control {

Eigen::Vector3d QuaternionController::Update(
    const Eigen::Vector3d &_desired_heading, double _desired_roll,
    const Eigen::Quaterniond &_orientation) {
  using hippo_common::tf2_utils::RotationBetweenNormalizedVectors;
  orientation_ = _orientation;
  const Eigen::Vector3d current_heading = CurrentForwardAxis();

  Eigen::Quaterniond q_error_red =
      RotationBetweenNormalizedVectors(current_heading, _desired_heading);
  Eigen::Quaterniond q_des_red = q_error_red * orientation_;

  using hippo_common::tf2_utils::QuaternionFromHeading;
  Eigen::Quaterniond q_des =
      QuaternionFromHeading(_desired_heading, _desired_roll);

  Eigen::Quaterniond q_mix = q_des_red.inverse() * q_des;
  double acos_tmp = acos(q_mix.w());
  double asin_tmp = asin(q_mix.x());
  Eigen::Quaterniond q_mix_scaled = Eigen::Quaterniond{
      cos(roll_weight_ * acos_tmp), sin(roll_weight_ * asin_tmp), 0.0, 0.0};
  Eigen::Quaterniond q_des_mixed = q_des_red * q_mix_scaled;
  Eigen::Quaternion q_error_mixed = orientation_.inverse() * q_des_mixed;
  return 2.0 * gain_ * sgn(q_error_mixed.w()) * q_error_mixed.vec();
}

Eigen::Quaterniond QuaternionController::ReducedQuaternionCommand(
    const Eigen::Vector3d &_desired_heading) {
  Eigen::Vector3d current_heading = CurrentForwardAxis();
  using hippo_common::tf2_utils::RotationBetweenNormalizedVectors;
  Eigen::Quaterniond q_error =
      RotationBetweenNormalizedVectors(current_heading, _desired_heading);
  return orientation_ * q_error;
}

Eigen::Quaterniond QuaternionController::MixedQuaternionCommand(
    const Eigen::Vector3d &_heading, double _roll) {
  auto q_mix = ReducedQuaternionCommand(_heading).inverse() *
               FullQuaternionCommand(_heading, _roll);
  double tmp = acos(q_mix.w());
  Eigen::Quaterniond q_mix_scaled{cos(roll_weight_ * tmp),
                                  sin(roll_weight_ * tmp), 0.0, 0.0};
  Eigen::Quaterniond q_des = ReducedQuaternionCommand(_heading) * q_mix_scaled;
  return orientation_.inverse() * q_des;
}
}  // namespace attitude_control
}  // namespace hippo_control
