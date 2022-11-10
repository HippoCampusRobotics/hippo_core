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

#include "rapid_trajectories/trajectory_generator/generator.h"

#include <algorithm>
#include <limits>

#include "rapid_trajectories/roots/quartic.hpp"

namespace rapid_trajectories {
namespace trajectory_generator {
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

RapidTrajectoryGenerator::RapidTrajectoryGenerator(const Eigen::Vector3d &_p0,
                                                   const Eigen::Vector3d &_v0,
                                                   const Eigen::Vector3d &_a0) {
  // initialise each axis:
  Reset();
  for (int i = 0; i < 3; i++) {
    axis_[i].SetInitialState(_p0[i], _v0[i], _a0[i]);
  }
}

void RapidTrajectoryGenerator::SetGoalPosition(const Eigen::Vector3d &_in) {
  for (unsigned i = 0; i < 3; i++) {
    SetGoalPositionInAxis(i, _in[i]);
  }
}

void RapidTrajectoryGenerator::SetGoalVelocity(const Eigen::Vector3d &_in) {
  for (int i = 0; i < 3; i++) SetGoalVelocityInAxis(i, _in[i]);
}

void RapidTrajectoryGenerator::SetGoalAcceleration(const Eigen::Vector3d &_in) {
  for (int i = 0; i < 3; i++) SetGoalAccelerationInAxis(i, _in[i]);
}

void RapidTrajectoryGenerator::Reset(void) {
  for (int i = 0; i < 3; i++) {
    axis_[i].Reset();
  }
  t_final_ = 0;
}

// Generate the trajectory:
void RapidTrajectoryGenerator::Generate(const double timeToFinish) {
  t_final_ = timeToFinish;
  for (int i = 0; i < 3; i++) {
    axis_[i].GenerateTrajectory(t_final_);
  }
}

RapidTrajectoryGenerator::InputFeasibilityResult
RapidTrajectoryGenerator::CheckInputFeasibilitySection(double _f_min_allowed,
                                                       double _f_max_allowed,
                                                       double _w_max_allowed,
                                                       double _t1, double _t2,
                                                       double _dt_min) {
  if (_t2 - _t1 < _dt_min) return InputIndeterminable;
  // test the acceleration at the two limits:
  if (std::max(GetThrust(_t1), GetThrust(_t2)) > _f_max_allowed)
    return InputInfeasibleThrustHigh;
  if (std::min(GetThrust(_t1), GetThrust(_t2)) < _f_min_allowed)
    return InputInfeasibleThrustLow;

  double f_min_square = 0;
  double f_max_square = 0;
  double jerk_max_square = 0;

  // Test the limits of the box we're putting around the trajectory:
  for (int i = 0; i < 3; i++) {
    double roots[6];
    size_t n_roots;
    double alpha = axis_[i].GetParamAlpha();
    double beta = axis_[i].GetParamBeta();
    double gamma = axis_[i].GetParamGamma();
    double f_dot[5];
    f_dot[0] = damping_ * alpha / 6.0;
    f_dot[1] = mass_param_ * alpha / 6.0 + damping_ * beta * 0.5;
    f_dot[2] = mass_param_ * beta * 0.5 + damping_ * gamma;
    f_dot[3] = mass_param_ * gamma + damping_ * axis_[i].GetInitialAcceleration();

    // TODO(lennartalff): check if another algorithm is needed
    // bring it in normal form by dividing by f_dot[0]
    n_roots = magnet::math::cubicSolve(f_dot[1] / f_dot[0], f_dot[2] / f_dot[0], f_dot[3] / f_dot[0],
                                       roots[0], roots[1], roots[2]);
    // TODO(lennartalff): handle other cases
    assert(n_roots == 3);
    // no need to check explicitly for saddle points. In this case, the min and
    // max value would lie on the interval borders and we checked feasibility
    // for the borders in the beginning.

    // TODO(lennartalff): why this check? what about n_roots?
    if (roots[2] < 0) {
      roots[2] = 0;
    }
    double v1, v2;
    // TODO: why do i need two variables, if they are the same?
    v1 = v2 = GetThrust(roots[2]);

    thrust_max_ = v1;
    // TODO(lennartalff): WTF is this doing? roots[3] is uninitalized and
    // therefore the value is undefined!!
    thrust_min_ = GetThrust(roots[3]);

    f_max_square = thrust_max_ * thrust_max_;
    if (f_max_square > _f_max_allowed * _f_max_allowed) {
      return InputInfeasibleThrustHigh;
    }

    f_min_square = p

        // definitely infeasible:
        if (std::max(pow(v1, 2), pow(v2, 2)) >
            pow(_f_max_allowed, 2)) return InputInfeasibleThrustHigh;

    if (v1 * v2 < 0) {
      // sign of acceleration changes, so we've gone through zero
      f_min_square += 0;
    } else {
      f_min_square += pow(std::min(fabs(v1), fabs(v2)), 2);
    }

    f_max_square += pow(std::max(fabs(v1), fabs(v2)), 2);

    jerk_max_square += axis_[i].GetMaxJerkSquared(_t1, _t2);
  }

  double fmin = sqrt(f_min_square);
  double fmax = sqrt(f_max_square);
  double wBound;
  if (f_min_square > 1e-6)
    wBound = sqrt(jerk_max_square / f_min_square);  // the 1e-6 is a
                                                    // divide-by-zero protection
  else
    wBound = std::numeric_limits<double>::max();

  // definitely infeasible:
  if (fmax < _f_min_allowed) return InputInfeasibleThrustLow;
  if (fmin > _f_max_allowed) return InputInfeasibleThrustHigh;

  // possibly infeasible:
  if (fmin < _f_min_allowed || fmax > _f_max_allowed ||
      wBound > _w_max_allowed) {  // indeterminate: must check more closely:
    double tHalf = (_t1 + _t2) / 2;
    InputFeasibilityResult r1 = CheckInputFeasibilitySection(
        _f_min_allowed, _f_max_allowed, _w_max_allowed, _t1, tHalf, _dt_min);

    if (r1 == InputFeasible) {
      // continue with second half
      return CheckInputFeasibilitySection(_f_min_allowed, _f_max_allowed,
                                          _w_max_allowed, tHalf, _t2, _dt_min);
    }

    // first section is already infeasible, or indeterminate:
    return r1;
  }

  // definitely feasible:
  return InputFeasible;
}

RapidTrajectoryGenerator::InputFeasibilityResult
RapidTrajectoryGenerator::CheckInputFeasibility(double _f_max_allowed,
                                                double _f_min_allowed,
                                                double _w_max_allowed,
                                                double _dt_min) {
  // required thrust limits along trajectory
  double _t1 = 0;
  double _t2 = t_final_;

  return CheckInputFeasibilitySection(_f_max_allowed, _f_min_allowed,
                                      _w_max_allowed, _t1, _t2, _dt_min);
}

RapidTrajectoryGenerator::StateFeasibilityResult
RapidTrajectoryGenerator::CheckPositionFeasibility(
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

    if ((GetPosition(roots[i]) - boundaryPoint).Dot(_boundary_normal) <= 0) {
      // touching, or on the wrong side of, the boundary!
      return StateInfeasible;
    }
  }
  return StateFeasible;
}

Eigen::Vector3d RapidTrajectoryGenerator::GetOmega(double t, double _dt) const {
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
};  // namespace trajectory_generator
};  // namespace rapid_trajectories
