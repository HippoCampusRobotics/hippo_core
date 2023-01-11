/*!
 * Rapid trajectory generation for quadrocopters
 *
 * Copyright 2014 by Mark W. Mueller <mwm@mwm.im>
 *
 * This code is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This code is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the code.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rapid_trajectories/trajectory/generator.hpp"

#include <algorithm>
#include <limits>

#include "rapid_trajectories/roots/quartic.hpp"

namespace rapid_trajectories {
namespace minimum_jerk {
template <typename T>
int sgn(T value) {
  return (T(0) < value) - (value < T(0));
}

template <typename T>
T max(T a, T b, T c) {
  if (a > b) {
    if (a > c) {
      return a;
    }
    return c;
  } else {
    if (b > c) {
      return b;
    }
    return c;
  }
}

template <typename T>
T min(T a, T b, T c) {
  if (a < b) {
    if (a < c) {
      return a;
    }
    return c;
  } else {
    if (b < c) {
      return b;
    }
    return c;
  }
}

Trajectory::Trajectory(const Eigen::Vector3d &_p0, const Eigen::Vector3d &_v0,
                       const Eigen::Vector3d &_a0,
                       const Eigen::Vector3d &_gravity, const double _mass_rb,
                       const double _mass_added, const double _damping,
                       const Eigen::Quaterniond &_rotation) {
  // initialise each axis:
  Reset();
  rotation_ = _rotation;
  damping_ = _damping;
  mass_rb_ = _mass_rb;
  mass_added_ = _mass_added;
  gravity_ = _gravity;
  for (size_t i = 0; i < axis_.size(); i++) {
    axis_[i].SetGravity(gravity_[i]);
    axis_[i].SetDamping(damping_);
    axis_[i].SetMassRigidBody(mass_rb_);
    axis_[i].SetMassAdded(mass_added_);
    axis_[i].SetInitialState(_p0[i], _v0[i], _a0[i]);
  }
}
/**
 * @brief
 *
 * @param _in
 */
void Trajectory::SetGoalPosition(const Eigen::Vector3d &_in) {
  for (unsigned i = 0; i < 3; i++) {
    SetGoalPositionInAxis(i, _in[i]);
  }
}

void Trajectory::SetGoalVelocity(const Eigen::Vector3d &_in) {
  for (int i = 0; i < 3; i++) {
    SetGoalVelocityInAxis(i, _in[i]);
  }
}

void Trajectory::SetGoalAcceleration(const Eigen::Vector3d &_in) {
  for (int i = 0; i < 3; i++) {
    SetGoalAccelerationInAxis(i, _in[i]);
  }
}

void Trajectory::Reset(void) {
  for (int i = 0; i < 3; i++) {
    axis_[i].Reset();
  }
  t_final_ = 0;
}

// Generate the trajectory:
void Trajectory::Generate(const double timeToFinish) {
  t_final_ = timeToFinish;
  for (int i = 0; i < 3; i++) {
    axis_[i].GenerateTrajectory(t_final_);
  }
}

Trajectory::InputFeasibilityResult Trajectory::CheckInputFeasibilitySection(
    double _f_min_allowed, double _f_max_allowed, double _w_max_allowed,
    double _t1, double _t2, double _dt_min) {
  if (_t2 - _t1 < _dt_min) return InputIndeterminable;
  // test the acceleration at both limits:
  if (std::max(GetThrust(_t1), GetThrust(_t2)) > _f_max_allowed)
    return InputInfeasibleThrustHigh;
  if (std::min(GetThrust(_t1), GetThrust(_t2)) < _f_min_allowed)
    return InputInfeasibleThrustLow;

  // The following sums are used as a rough upper and lower limit. The actual
  // minimum or maximum over the trajectory time horizon might be much larger
  // than the sum of the sum of minimas and much smaller than the sum of
  // maximas of each axis. If this coarse criterium is not sufficient to
  // determine feasibility, this function can be called recursively with the
  // time horizon

  // split in half. sum of min(f²) over all axes
  double f_square_min_sum = 0;
  // sum of max(f^2) over all axes
  double f_square_max_sum = 0;
  double jerk_square_max = 0;

  // Test the limits of the box we're putting around the trajectory:
  for (int i = 0; i < 3; i++) {
    auto [f_min, f_max] = axis_[i].GetMinMaxForce(_t1, _t2);

    double f_min_square = f_min * f_min;
    double f_max_square = f_max * f_max;
    if (f_min_square > f_max_square) {
      std::swap(f_min_square, f_max_square);
    }

    // if a single axis already has max(f²) > f²ₘₐₓ, the trajectory
    // is not feasible. Very coarse criterium.
    if (f_max_square > _f_max_allowed * _f_max_allowed) {
      return InputInfeasibleThrustHigh;
    }

    // zero crossing => min(f²) = 0
    if (f_min * f_max < 0) {
      // gets optimized away by compilers. just for readability
      f_square_min_sum += 0;
    } else {
      f_square_min_sum += f_min_square;
    }
    f_square_max_sum += f_max_square;
    jerk_square_max += axis_[i].GetMaxJerkSquared(_t1, _t2);
  }

  // if not even the sum of individual maxima is sufficient to reach the minum
  // allowed force, all hope is lost.
  if (f_square_max_sum < _f_min_allowed * _f_min_allowed) {
    return InputInfeasibleThrustLow;
  }
  // if not even the sum of indiviual minima is sufficient to be below the
  // maximum allowed force, we can stop right here.
  if (f_square_min_sum > _f_max_allowed * _f_max_allowed) {
    return InputInfeasibleThrustHigh;
  }

  double omega_bound_square;
  // dont divide by zero
  if (f_square_min_sum > 1e-6) {
    omega_bound_square = jerk_square_max / f_square_min_sum;
  } else {
    omega_bound_square = std::numeric_limits<double>::max();
  }

  // opposite of:
  // sum(min(f²)) >= f²ₘₐₓ AND
  // sum(max(f²)) <= f²ₘᵢₙ AND
  // ω² <= ω²ₘₐₓ
  // that is the only (but coarse) criterium for feasibility.
  if ((f_square_min_sum < _f_min_allowed * _f_min_allowed) ||
      (f_square_max_sum > _f_max_allowed * _f_max_allowed) ||
      (omega_bound_square > _w_max_allowed * _w_max_allowed)) {
    // we need to split the interval
    double t_mid = (_t1 + _t2) * 0.5;
    InputFeasibilityResult result;
    result = CheckInputFeasibilitySection(_f_min_allowed, _f_max_allowed,
                                          _w_max_allowed, _t1, t_mid, _dt_min);
    if (result == InputFeasible) {
      result = CheckInputFeasibilitySection(
          _f_min_allowed, _f_max_allowed, _w_max_allowed, t_mid, _t2, _dt_min);
    }
    return result;
  }
  return InputFeasible;
}

Trajectory::InputFeasibilityResult Trajectory::CheckInputFeasibility(
    double _f_max_allowed, double _f_min_allowed, double _w_max_allowed,
    double _dt_min) {
  // required thrust limits along trajectory
  double _t1 = 0;
  double _t2 = t_final_;

  return CheckInputFeasibilitySection(_f_max_allowed, _f_min_allowed,
                                      _w_max_allowed, _t1, _t2, _dt_min);
}

Trajectory::StateFeasibilityResult Trajectory::CheckPositionFeasibility(
    Eigen::Vector3d &_boundary_point, Eigen::Vector3d &_boundary_normal) {
  // Ensure that the normal is a unit vector:
  _boundary_normal.normalize();

  // first, we will build the polynomial describing the velocity of the a
  // quadrocopter in the direction of the normal. Then we will solve for
  // the zeros of this, which give us the times when the position is at a
  // critical point. Then we evaluate the position at these points, and at
  // the trajectory beginning and end, to see whether we are feasible.

  // need to check that the trajectory stays inside the safe box throughout the
  // flight:

  // the coefficients of the quartic equation: x(t) = c[0]t**4 + c[1]*t**3 +
  // c[2]*t**2 + c[3]*t + c[4]
  double c[5] = {0, 0, 0, 0, 0};

  for (int dim = 0; dim < 3; dim++) {
    c[0] += _boundary_normal[dim] * axis_[dim].GetParamAlpha() / 24.0;  // t**4
    c[1] += _boundary_normal[dim] * axis_[dim].GetParamBeta() / 6.0;    // t**3
    c[2] += _boundary_normal[dim] * axis_[dim].GetParamGamma() / 2.0;   // t**2
    c[3] += _boundary_normal[dim] * axis_[dim].GetInitialAcceleration();  // t
    c[4] += _boundary_normal[dim] * axis_[dim].GetInitialVelocity();      // 1
  }

  // Solve the roots (we prepend the times 0 and tf):
  double roots[6];
  roots[0] = 0;
  roots[1] = t_final_;

  size_t rootCount;
  if (fabs(c[0]) > 1e-6) {
    rootCount = magnet::math::quarticSolve(c[1] / c[0], c[2] / c[0],
                                           c[3] / c[0], c[4] / c[0], roots[2],
                                           roots[3], roots[4], roots[5]);
  } else {
    rootCount = magnet::math::cubicSolve(c[2] / c[1], c[3] / c[1], c[4] / c[1],
                                         roots[2], roots[3], roots[4]);
  }

  for (unsigned i = 0; i < (rootCount + 2); i++) {
    // don't evaluate points outside the domain
    if (roots[i] < 0) continue;
    if (roots[i] > t_final_) continue;

    if ((GetPosition(roots[i]) - _boundary_point).dot(_boundary_normal) <= 0) {
      // touching, or on the wrong side of, the boundary!
      return StateInfeasible;
    }
  }
  return StateFeasible;
}

Eigen::Vector3d Trajectory::GetOmega(double t, double _dt) const {
  // Calculates the body rates necessary at time t, to rotate the normal vector.
  // The result is coordinated in the world frame, i.e. would have to be rotated
  // into a body frame.
  const Eigen::Vector3d &n0 = GetNormalVector(t);
  const Eigen::Vector3d &n1 = GetNormalVector(t + _dt);

  const Eigen::Vector3d &crossProd{
      n0.cross(n1)};  // direction of omega, in inertial axes

  if (crossProd.norm())
    return (acos(n0.dot(n1)) / _dt) * crossProd.normalized();
  else
    return Eigen::Vector3d::Zero();
}
};  // namespace minimum_jerk
};  // namespace rapid_trajectories
