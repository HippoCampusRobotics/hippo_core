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

#include <hippo_control/rate_control/rate_control.hpp>

namespace hippo_control {
namespace rate_control {

Eigen::Vector3d Controller::Update(const Eigen::Vector3d &_rate,
                                   const Eigen::Vector3d &_rate_setpoint,
                                   const Eigen::Vector3d &_angular_acceleration,
                                   const double _dt) {
  Eigen::Vector3d rate_error = _rate_setpoint - _rate;

  Eigen::Vector3d u = p_gain_.cwiseProduct(rate_error) + integral_ -
                      d_gain_.cwiseProduct(_angular_acceleration) +
                      feed_forward_gain_.cwiseProduct(_rate_setpoint);
  UpdateIntegral(rate_error, _dt);
  return u;
}

void Controller::UpdateIntegral(Eigen::Vector3d &_rate_error,
                                const double _dt) {
  for (int i = 0; i < 3; ++i) {
    // smoothly decrease integral for large errors. inspired by:
    // https://github.com/PX4/PX4-Autopilot/blob/a5e4295029162cbc66c3e61f7b11a9672a461bc4/src/lib/rate_control/rate_control.cpp#L93
    double scaler = _rate_error(i) / zero_integral_threshold_;
    scaler = std::max(1.0 - scaler * scaler, 0.0);
    double integral = integral_(i) + scaler * i_gain_(i) * _rate_error(i) * _dt;
    integral_(i) =
        std::clamp(integral, -integral_limit_(i), integral_limit_(i));
  }
}
}  // namespace rate_control
}  // namespace hippo_control
