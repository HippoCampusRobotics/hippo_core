/**
 * Rapid trajectory generation for quadrocopters
 *
 *  Copyright 2014 by Mark W. Mueller <mwm@mwm.im>
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

#pragma once
#include <array>
#include <vector>

namespace rapid_trajectories {
namespace trajectory_generator {

//! An axis along one spatial direction
/*!
 * For more information, please refer to Section II of the publication.
 */
class SingleAxisTrajectory {
 public:
  static constexpr int kMaxForceDerivateRoots = 3;
  /// @brief We always evaluate at the borders first and 0.0 is always at the
  /// border. So extrema at this time value will be ignored.
  static constexpr double kNoPeakTime = 0.0;
  static constexpr int kTrajectoryParamsCount = 3;
  //! Constructor, calls Reset() function.
  SingleAxisTrajectory();
  SingleAxisTrajectory(double _mass, double _damping);
  SingleAxisTrajectory(double _alpha, double _beta, double _gamma, double _mass,
                       double _damping, double _p_start, double _v_start,
                       double _a_start);

  //! Define the trajectory's initial state (position, velocity, acceleration),
  //! at time zero
  void SetInitialState(const double pos0, const double vel0,
                       const double acc0) {
    p_start_ = pos0;
    v_start_ = vel0;
    a_start_ = acc0;
    Reset();
  };

  //! Define the trajectory's final position (if you don't call this, it is left
  //! free)
  void SetGoalPosition(const double posf) {
    position_constrained_ = true;
    p_final_ = posf;
  };

  //! Define the trajectory's final velocity (if you don't call this, it is left
  //! free)
  void SetGoalVelocity(const double velf) {
    velocity_constrained_ = true;
    v_final_ = velf;
  };

  //! Define the trajectory's final acceleration (if you don't call this, it is
  //! left free)
  void SetGoalAcceleration(const double accf) {
    acceleration_constrained_ = true;
    a_final_ = accf;
  };

  inline void SetDamping(const double _damping) { damping_ = _damping; }

  inline double GetDamping() const { return damping_; }

  inline void SetMass(const double _mass) { mass_ = _mass; }

  inline double GetMass() const { return mass_; }

  //! Generate the trajectory, from the defined initial state to the defined
  //! components of the final state.
  void GenerateTrajectory(const double timeToGo);

  //! Resets the cost, coefficients, and goal state constraints. Does *not*
  //! reset the initial state
  void Reset(void);

  //! Returns the jerk at time t
  double GetJerk(double t) const {
    return gamma_ + beta_ * t + (1 / 2.0) * alpha_ * t * t;
  };

  //! Returns the acceleration at time t
  double GetAcceleration(double t) const {
    return a_start_ + gamma_ * t + (1 / 2.0) * beta_ * t * t +
           (1 / 6.0) * alpha_ * t * t * t;
  };

  double GetForce(double _t) const;

  //! Returns the velocity at time t
  double GetVelocity(double t) const {
    return v_start_ + a_start_ * t + (1 / 2.0) * gamma_ * t * t +
           (1 / 6.0) * beta_ * t * t * t + (1 / 24.0) * alpha_ * t * t * t * t;
  };

  //! Returns the position at time t
  double GetPosition(double t) const {
    return p_start_ + v_start_ * t + (1 / 2.0) * a_start_ * t * t +
           (1 / 6.0) * gamma_ * t * t * t + (1 / 24.0) * beta_ * t * t * t * t +
           (1 / 120.0) * alpha_ * t * t * t * t * t;
  };

  std::vector<double> GetForceDerivativeRoots() const;

  std::pair<double, double> GetMinMaxForce(double _t1, double _t2);

  //! Calculate the extrema of the acceleration trajectory over a section
  void GetMinMaxAcc(double &aMinOut, double &aMaxOut, double t1, double t2);

  //! Calculate the extrema of the jerk squared over a section
  double GetMaxJerkSquared(double t1, double t2);

  //! Get the parameters defining the trajectory
  double GetParamAlpha(void) const { return alpha_; };
  //! Get the parameters defining the trajectory
  double GetParamBeta(void) const { return beta_; };
  //! Get the parameters defining the trajectory
  double GetParamGamma(void) const { return gamma_; };
  //! Get the parameters defining the trajectory
  double GetInitialAcceleration(void) const { return a_start_; };
  //! Get the parameters defining the trajectory
  double GetInitialVelocity(void) const { return v_start_; };
  //! Get the parameters defining the trajectory
  double GetInitialPosition(void) const { return p_start_; };

  //! Get the trajectory cost value
  double GetCost(void) const { return cost_; };

 private:
  inline void HandleNoPeaks(const double &_t1, const double &_t2, double &_min,
                            double &_max);
  struct PeakTimes {
    std::array<double, 2> t;
    bool computed{false};
  };
  /// @brief Start position of the trajectory.
  double p_start_;
  /// @brief Start velocity of the trajectory.
  double v_start_;
  /// @brief Start acceleration of the trajectory.
  double a_start_;

  /// @brief Final position of the trajectory.
  double p_final_;
  /// @brief Final velocity of the trajectory.
  double v_final_;
  /// @brief Final acceleration of the trajectory.
  double a_final_;
  /// @brief Time horizon of the trajectory.
  double t_final_;

  /// @brief Position for the final state is defined.
  bool position_constrained_;
  /// @brief Velocity for the final state is defined.
  bool velocity_constrained_;
  /// @brief Acceleleration for the final state is defined.
  bool acceleration_constrained_;
  /// @brief Coefficient for the trajectory polynomial
  double alpha_;
  /// @brief Coefficient for the trajectory polynomial
  double beta_;
  /// @brief Coefficient for the trajectory polynomial
  double gamma_;

  /// @brief Trajectory cost.
  double cost_;

  PeakTimes accel_peak_times_;
  PeakTimes force_peak_times_;

  /// @brief Damping is modelled as a linear function of the velocity;
  double damping_{5.4};
  /// @brief Effective mass of the vehicle;
  double mass_{2.6};
};
};  // namespace trajectory_generator
};  // namespace rapid_trajectories
