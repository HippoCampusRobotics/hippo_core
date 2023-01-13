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

#include "rapid_trajectories/trajectory/single_axis.hpp"

#include <math.h>

#include <algorithm>  //For min/max
#include <limits>     //for max double

#include "rapid_trajectories/roots/quartic.hpp"

namespace rapid_trajectories {
namespace minimum_jerk {

SingleAxisTrajectory::SingleAxisTrajectory(void)
    : alpha_(0),
      beta_(0),
      gamma_(0),
      damping_(5.4),
      mass_rb_(1.5),
      mass_added_(1.5) {
  Reset();
}

SingleAxisTrajectory::SingleAxisTrajectory(double _mass_rb, double _mass_added,
                                           double _damping)
    : alpha_(0),
      beta_(0),
      gamma_(0),
      damping_(_damping),
      mass_rb_(_mass_rb),
      mass_added_(_mass_added) {
  Reset();
}

/// @brief This constructor should only be used to sample a specific trajectory
/// for visualization or similar purposes. Do not use this constructor for
/// trajectory generation.
/// @param _alpha
/// @param _beta
/// @param _gamma
/// @param _mass
/// @param _damping
/// @param _p_start
/// @param _v_start
/// @param _a_start
SingleAxisTrajectory::SingleAxisTrajectory(double _alpha, double _beta,
                                           double _gamma, double _mass_rb,
                                           double _mass_added, double _damping,
                                           double _p_start, double _v_start,
                                           double _a_start)
    : p_start_(_p_start),
      v_start_(_v_start),
      a_start_(_a_start),
      alpha_(_alpha),
      beta_(_beta),
      gamma_(_gamma),
      damping_(_damping),
      mass_rb_(_mass_rb),
      mass_added_(_mass_added) {
  Reset();
}

/*!
 * Resets the goal state constraints and the cost.
 */
void SingleAxisTrajectory::Reset(void) {
  position_constrained_ = velocity_constrained_ = acceleration_constrained_ =
      false;
  cost_ = std::numeric_limits<double>::max();
  accel_peak_times_.computed = false;
}

/*!
 * Calculate the coefficients that define a trajectory. This solves, in
 * closed form the optimal trajectory for some given set of goal
 * constraints. Before calling this,
 *   - define the initial state
 *   - define the relevant parts of the trajectory goal state.
 *
 * @param Tf the trajectory duration [s]
 * @see SetInitialState()
 * @see SetGoalPosition()
 * @see SetGoalVelocity()
 * @see SetGoalAcceleration()
 */
void SingleAxisTrajectory::GenerateTrajectory(const double Tf) {
  t_final_ = Tf;
  // define starting position:
  double delta_a = a_final_ - a_start_;
  double delta_v = v_final_ - v_start_ - a_start_ * Tf;
  double delta_p =
      p_final_ - p_start_ - v_start_ * Tf - 0.5 * a_start_ * Tf * Tf;

  // powers of the end time:
  const double T2 = Tf * Tf;
  const double T3 = T2 * Tf;
  const double T4 = T3 * Tf;
  const double T5 = T4 * Tf;

  // solve the trajectories, depending on what's constrained:
  if (position_constrained_ && velocity_constrained_ &&
      acceleration_constrained_) {
    alpha_ = (60 * T2 * delta_a - 360 * Tf * delta_v + 720 * 1 * delta_p) / T5;
    beta_ = (-24 * T3 * delta_a + 168 * T2 * delta_v - 360 * Tf * delta_p) / T5;
    gamma_ = (3 * T4 * delta_a - 24 * T3 * delta_v + 60 * T2 * delta_p) / T5;
  } else if (position_constrained_ && velocity_constrained_) {
    alpha_ = (-120 * Tf * delta_v + 320 * delta_p) / T5;
    beta_ = (72 * T2 * delta_v - 200 * Tf * delta_p) / T5;
    gamma_ = (-12 * T3 * delta_v + 40 * T2 * delta_p) / T5;
  } else if (position_constrained_ && acceleration_constrained_) {
    alpha_ = (-15 * T2 * delta_a + 90 * delta_p) / (2 * T5);
    beta_ = (15 * T3 * delta_a - 90 * Tf * delta_p) / (2 * T5);
    gamma_ = (-3 * T4 * delta_a + 30 * T2 * delta_p) / (2 * T5);
  } else if (velocity_constrained_ && acceleration_constrained_) {
    alpha_ = 0;
    beta_ = (6 * Tf * delta_a - 12 * delta_v) / T3;
    gamma_ = (-2 * T2 * delta_a + 6 * Tf * delta_v) / T3;
  } else if (position_constrained_) {
    alpha_ = 20 * delta_p / T5;
    beta_ = -20 * delta_p / T4;
    gamma_ = 10 * delta_p / T3;
  } else if (velocity_constrained_) {
    alpha_ = 0;
    beta_ = -3 * delta_v / T3;
    gamma_ = 3 * delta_v / T2;
  } else if (acceleration_constrained_) {
    alpha_ = 0;
    beta_ = 0;
    gamma_ = delta_a / Tf;
  } else {  // Nothing to do!
    alpha_ = beta_ = gamma_ = 0;
  }

  // Calculate the cost:
  cost_ = gamma_ * gamma_ + beta_ * gamma_ * Tf + beta_ * beta_ * T2 / 3.0 +
          alpha_ * gamma_ * T2 / 3.0 + alpha_ * beta_ * T3 / 4.0 +
          alpha_ * alpha_ * T4 / 20.0;
  UpdatePolynomialCoefficients();
}

void SingleAxisTrajectory::UpdatePolynomialCoefficients() {
  v_poly_coeff_ = {v_start_, a_start_, 0.5 * gamma_, 1.0 / 6.0 * beta_,
                   1.0 / 24.0 * alpha_};
  a_poly_coeff_ = {a_start_, gamma_, 0.5 * beta_, 1.0 / 6.0 * alpha_};
  j_poly_coeff_ = {gamma_, beta_, 0.5 * alpha_};
  xi_poly_coeff_ = {damping_ * a_poly_coeff_[0] + j_poly_coeff_[0],
                    damping_ * a_poly_coeff_[1] + j_poly_coeff_[1],
                    damping_ * a_poly_coeff_[2] + j_poly_coeff_[2],
                    damping_ * a_poly_coeff_[3]};
  dxi_poly_coeff_ = {xi_poly_coeff_[1], xi_poly_coeff_[2] * 2.0,
                     xi_poly_coeff_[3] * 3.0};
  const double m = GetMassEffective();
  f_poly_coeff_ = {damping_ * v_poly_coeff_[0] + m * a_poly_coeff_[0],
                   damping_ * v_poly_coeff_[1] + m * a_poly_coeff_[1],
                   damping_ * v_poly_coeff_[2] + m * a_poly_coeff_[2],
                   damping_ * v_poly_coeff_[3] + m * a_poly_coeff_[3],
                   damping_ * v_poly_coeff_[4]};
  df_poly_coeff_ = {f_poly_coeff_[1], f_poly_coeff_[2] * 2.0,
                    f_poly_coeff_[3] * 3.0, f_poly_coeff_[4] * 4.0};
}

std::pair<double, double> SingleAxisTrajectory::GetMinMaxXi(double _t1,
                                                            double _t2) {
  if (!xi_peak_times_.computed) {
    // xi is of third order -> maximum of 2 extrema
    // solve roots of quadratic function for peak candidates
    if (dxi_poly_coeff_.at(2)) {
      const double det = dxi_poly_coeff_.at(1) * dxi_poly_coeff_.at(1) -
                         4.0 * dxi_poly_coeff_.at(2) * dxi_poly_coeff_.at(0);
      if (det < 0) {
        // no real roots
        xi_peak_times_.t.at(0) = kNoPeakTime;
        xi_peak_times_.t.at(1) = kNoPeakTime;
      } else {
        xi_peak_times_.t.at(0) =
            0.5 * (-dxi_poly_coeff_.at(1) - sqrt(det)) / dxi_poly_coeff_.at(2);
        xi_peak_times_.t.at(1) =
            0.5 * (-dxi_poly_coeff_.at(1) + sqrt(det)) / dxi_poly_coeff_.at(2);
      }
    } else {
      // we have a linear function since the first coeff is zero
      //  at most a single root expected
      xi_peak_times_.t.at(1) = kNoPeakTime;
      if (dxi_poly_coeff_.at(1)) {
        xi_peak_times_.t.at(0) =
            -1.0 * dxi_poly_coeff_.at(0) / dxi_poly_coeff_.at(1);
      } else {
        // the function is a constant. no roots at all
        xi_peak_times_.t.at(0) = kNoPeakTime;
      }
    }
    xi_peak_times_.computed = true;
  }
  double min = PolyEval(xi_poly_coeff_, _t1);
  double max = PolyEval(xi_poly_coeff_, _t2);
  if (min > max) {
    std::swap(min, max);
  }
  for (int i = 0; i < 2; ++i) {
    const double t = xi_peak_times_.t.at(i);
    if (t <= _t1 || t >= _t2) {
      // only interested if strictly inside [_t1, _t2]
      continue;
    }
    min = std::min(min, PolyEval(xi_poly_coeff_, t));
    max = std::max(max, PolyEval(xi_poly_coeff_, t));
  }
  return std::pair{min, max};
}

/**
 * @brief Returns the roots of the force derivate polynomial in descending
 * order.
 *
 */
std::vector<double> SingleAxisTrajectory::GetThrustDerivativeRoots() {
  std::vector<double> result;
  result.reserve(kMaxForceDerivateRoots);
  double roots_array[kMaxForceDerivateRoots];
  size_t n_roots;
  // normalize the polynomial and solve for the roots
  double div = df_poly_coeff_.at(3);
  n_roots = magnet::math::cubicSolve(df_poly_coeff_.at(2) / div,
                                     df_poly_coeff_.at(1) / div,
                                     df_poly_coeff_.at(0) / div, roots_array[0],
                                     roots_array[1], roots_array[2]);
  for (size_t i = 0; i < n_roots; ++i) {
    // only add roots inside the time horizon of the whole trajectory.
    if (roots_array[i] < t_final_ && roots_array[i] > 0.0) {
      result.push_back(roots_array[i]);
    }
  }
  return result;
}

std::pair<double, double> SingleAxisTrajectory::GetMinMaxForce(double _t1,
                                                               double _t2) {
  double min;
  double max;
  if (!force_peak_times_.computed) {
    // compute the local extrama in the interval [0; t_final_].
    std::vector<double> roots = GetThrustDerivativeRoots();

    switch (roots.size()) {
      case 0: {
        // no extrema, min/max will lie on the boundaries
        force_peak_times_.t[0] = kNoPeakTime;
        force_peak_times_.t[1] = kNoPeakTime;
        break;
      }
      case 1: {
        force_peak_times_.t[0] = roots[0];
        force_peak_times_.t[1] = kNoPeakTime;
        break;
      }
      case 2: {
        force_peak_times_.t[0] = roots[0];
        force_peak_times_.t[1] = roots[1];
        break;
      }
      case 3: {
        std::array<double, 3> peak_candidates{
            GetForce(roots[0]), GetForce(roots[1]), GetForce(roots[2])};
        auto it =
            std::minmax_element(peak_candidates.begin(), peak_candidates.end());
        int index1 = std::distance(peak_candidates.begin(), it.first);
        int index2 = std::distance(peak_candidates.begin(), it.second);
        force_peak_times_.t[0] = roots[index1];
        force_peak_times_.t[1] = roots[index2];
        break;
      }
      default:
        break;
    }
    force_peak_times_.computed = true;
  }

  // the min/max might lie on the boundaries of the interval, so evaluate
  // these values first.
  min = GetForce(_t1);
  max = GetForce(_t2);
  if (GetForce(_t1) > GetForce(_t2)) {
    std::swap(min, max);
  }

  for (int i = 0; i < 2; ++i) {
    const double t = force_peak_times_.t[i];
    // skip the peak, if the peak is not strictly inside the interval [_t1;
    // _t2]
    if (t >= _t2 || t <= _t1) {
      continue;
    }
    min = std::min(min, GetForce(t));
    max = std::max(max, GetForce(t));
  }
  return std::pair{min, max};
}

/*!
 * Calculate the extrema of the acceleration trajectory. This is made
 * relatively easy by the fact that the acceleration is a cubic polynomial,
 * so we only need to solve for the roots of a quadratic.
 * @param aMinOut [out] The minimum acceleration in the segment
 * @param aMaxOut [out] The maximum acceleration in the segment
 * @param t1 [in] The start time of the segment
 * @param t2 [in] The end time of the segment
 */
void SingleAxisTrajectory::GetMinMaxAcc(double &aMinOut, double &aMaxOut,
                                        double t1, double t2) {
  if (!accel_peak_times_.computed) {
    // calculate the roots of the polynomial
    if (alpha_) {  // solve a quadratic for t
      double det = beta_ * beta_ - 2 * gamma_ * alpha_;
      if (det < 0) {  // no real roots
        accel_peak_times_.t[0] = 0;
        accel_peak_times_.t[1] = 0;
      } else {
        accel_peak_times_.t[0] = (-beta_ + sqrt(det)) / alpha_;
        accel_peak_times_.t[1] = (-beta_ - sqrt(det)) / alpha_;
      }
    } else {  // solve linear equation: gamma_ + beta_*t == 0:
      if (beta_)
        accel_peak_times_.t[0] = -gamma_ / beta_;
      else
        accel_peak_times_.t[0] = 0;
      accel_peak_times_.t[1] = 0;
    }

    accel_peak_times_.computed = 1;
  }

  // Evaluate the acceleration at the boundaries of the period:
  aMinOut = std::min(GetAcceleration(t1), GetAcceleration(t2));
  aMaxOut = std::max(GetAcceleration(t1), GetAcceleration(t2));

  // Evaluate at the maximum/minimum times:
  for (int i = 0; i < 2; i++) {
    if (accel_peak_times_.t[i] <= t1) continue;
    if (accel_peak_times_.t[i] >= t2) continue;

    aMinOut = std::min(aMinOut, GetAcceleration(accel_peak_times_.t[i]));
    aMaxOut = std::max(aMaxOut, GetAcceleration(accel_peak_times_.t[i]));
  }
}

/*!
 * Calculate the maximum jerk squared along the trajectory. This is made
 * easy because the jerk is quadratic, so we need to evaluate at most three
 * points.
 * @param t1 [in] The start time of the segment
 * @param t2 [in] The end time of the segment
 * @return the maximum jerk
 */
double SingleAxisTrajectory::GetMaxJerkSquared(double t1, double t2) {
  double jMaxSqr = std::max(pow(GetJerk(t1), 2), pow(GetJerk(t2), 2));

  if (alpha_) {
    double tMax = -1;
    tMax = -beta_ / alpha_;
    if (tMax > t1 && tMax < t2) {
      jMaxSqr = std::max(pow(GetJerk(tMax), 2), jMaxSqr);
    }
  }

  return jMaxSqr;
}
};  // namespace minimum_jerk
};  // namespace rapid_trajectories
