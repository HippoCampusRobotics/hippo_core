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
