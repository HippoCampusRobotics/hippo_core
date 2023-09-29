// Copyright (C) 2023 Thies Lennart Alff

// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
// USA

#include "path_planning/static_generation.hpp"

#include <hippo_common/spline.h>

#include <cmath>

namespace path_planning {
namespace static_generation {

Path LemniscateOfBernoulli(std::size_t _n_samples, double _x_limit,
                           double _y_limit) {
  double ax = _x_limit / sqrt(2) * 0.5;
  double ay = _y_limit;
  double a = std::min(ax, ay);
  std::vector<Eigen::Vector3d> positions;
  positions.reserve(_n_samples);
  for (std::size_t i = 0; i < _n_samples; ++i) {
    double t = (double)i / _n_samples * 2.0 * M_PI;
    double x = a * sqrt(2.0) * cos(t) / (sin(t) * sin(t) + 1.0);
    double y = a * sqrt(2.0) * cos(t) * sin(t) / (sin(t) * sin(t) + 1.0);
    double z = 0.0;
    Eigen::Vector3d p{x, y, z};
    positions.push_back(p);
  }

  return Path{positions, true};
}

Path MotorFailureSurface(size_t _n_samples) {
  std::vector<double> y = {0.0, 5.0, 6.0, 7.0};
  std::vector<double> z = {-0.5, -0.5, -0.3, 0.7};
  tk::spline s;
  s.set_boundary(tk::spline::first_deriv, 0.0, tk::spline::first_deriv, 1.0);
  s.set_points(y, z, tk::spline::cspline);
  s.make_monotonic();

  double xmin = 0.0, xmax = 8.5;
  std::vector<Eigen::Vector3d> positions;
  positions.reserve(_n_samples);
  for (size_t i = 0; i < _n_samples; ++i) {
    double x = xmin + (double)i * (xmax - xmin) / (_n_samples - 1);
    Eigen::Vector3d p{0.0, x, s(x)};
    positions.push_back(p);
  }
  return Path(positions, false);
}
}  // namespace static_generation

}  // namespace path_planning
