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

#include <hippo_control/trajectory_tracking/tracking_control.hpp>

namespace hippo_control {
namespace trajectory_tracking {
TrackingController::TrackingController() {}

Eigen::Quaterniond TrackingController::AttitudeFromThrust(
    const Eigen::Vector3d &_thrust, double _roll) {
  Eigen::Vector3d x_axis_desired{_thrust};
  bool valid_attitude{true};
  if (_thrust.squaredNorm() < __FLT_EPSILON__) {
    valid_attitude = false;
    x_axis_desired = last_valid_attitude_ * Eigen::Vector3d::UnitX();
  }
  x_axis_desired.normalize();

  const Eigen::Vector3d z_axis_intermediate{0.0, -sin(_roll), cos(_roll)};
  const Eigen::Vector3d y_axis_desired =
      z_axis_intermediate.cross(x_axis_desired).normalized();
  const Eigen::Vector3d z_axis_desired = x_axis_desired.cross(y_axis_desired);

  Eigen::Matrix3d R;
  for (int i = 0; i < 3; ++i) {
    R(i, 0) = x_axis_desired(i);
    R(i, 1) = y_axis_desired(i);
    R(i, 2) = z_axis_desired(i);
  }
  Eigen::Quaterniond attitude{R};
  last_valid_attitude_ = valid_attitude ? attitude : last_valid_attitude_;
  return attitude;
}

Eigen::Quaterniond TrackingController::Update(
    const Eigen::Vector3d &_position, const Eigen::Vector3d &_velocity,
    const Eigen::Vector3d &_feed_forward_thrust) {
  position_ = _position;
  velocity_ = _velocity;
  thrust_ = ComputeThrust(_feed_forward_thrust);
  return AttitudeFromThrust(thrust_, roll_desired_);
}

Eigen::Vector3d TrackingController::ComputeThrust(
    const Eigen::Vector3d &_feed_forward) {
  return Eigen::Vector3d{(position_desired_ - position_) * position_gain_ +
                         (velocity_desired_ - velocity_) * velocity_gain_ +
                         _feed_forward};
}
}  // namespace trajectory_tracking
}  // namespace hippo_control
