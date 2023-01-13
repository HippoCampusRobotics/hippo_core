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

#pragma once
#include <eigen3/Eigen/Dense>

#include "rapid_trajectories/trajectory/single_axis.hpp"

namespace rapid_trajectories {
namespace minimum_jerk {

//! A quadrocopter state interception trajectory.
/*!
 * A quadrocopter state interception trajectory. The trajectory starts at a
 * state defined by the vehicle's position, velocity, and acceleration. The
 * acceleration can be calculated directly from the quadrocopter's attitude
 * and thrust value. The trajectory duration is fixed, and given by the user.
 *
 * The trajectory goal state can include any combination of components from
 * the quadrocopter's position, velocity, and acceleration. The acceleration
 * allows to encode the direction of the quadrocopter's thrust at the end time.
 *
 * The trajectories are generated without consideration for any constraints,
 * and are optimal with respect to the integral of the jerk squared (which is
 * equivalent to an upper bound on a product of the inputs).
 *
 * The trajectories can then be tested with respect to input constraints
 * (thrust/body rates) with an efficient, recursive algorithm. Whether linear
 * combinations of states along the trajectory remain within some bounds can
 * also be tested efficiently.
 *
 * For more information, please see the publication `A computationally efficient
 * motion primitive for quadrocopter trajectory generation', avaible here:
 * http://www.mwm.im/research/publications/
 *
 * NOTE: in the publication, axes are 1-indexed, while here they are
 * zero-indexed.
 */
class Trajectory {
 public:
  enum InputFeasibilityResult {
    InputFeasible = 0,        //!< The trajectory is input feasible
    InputIndeterminable = 1,  //!< Cannot determine whether the trajectory is
                              //!< feasible with respect to the inputs
    InputInfeasibleThrustHigh =
        2,  //!< Trajectory is infeasible, failed max. thrust test first
    InputInfeasibleThrustLow =
        3,  //!< Trajectory is infeasible, failed min. thrust test first
    InputInfeasibleRates =
        4,  //!< Trajectory is infeasible, failed max. rates test first
  };

  enum StateFeasibilityResult {
    StateFeasible = 0,    //!< The trajectory is feasible w.r.t. the test
    StateInfeasible = 1,  //!< Trajectory is infeasible
  };

  //! Constructor, user must define initial state, and the direction of gravity.
  Trajectory(const Eigen::Vector3d &_p0, const Eigen::Vector3d &_v0,
             const Eigen::Vector3d &_a0, const Eigen::Vector3d &_gravity,
             const double _mass_rb, const double _mass_added,
             const double _damping, const Eigen::Quaterniond &_rotation);

  inline Eigen::Vector3d GetAxisParameters(int _i) const {
    return Eigen::Vector3d{GetAxisParamAlpha(_i), GetAxisParamBeta(_i),
                           GetAxisParamGamma(_i)};
  }

  inline Eigen::Vector3d GetAlphas() const {
    return Eigen::Vector3d{GetAxisParamAlpha(0), GetAxisParamAlpha(1),
                           GetAxisParamAlpha(2)};
  }

  inline Eigen::Vector3d GetBetas() const {
    return Eigen::Vector3d{GetAxisParamBeta(0), GetAxisParamBeta(1),
                           GetAxisParamBeta(2)};
  }

  inline Eigen::Vector3d GetGammas() const {
    return Eigen::Vector3d{GetAxisParamGamma(0), GetAxisParamGamma(1),
                           GetAxisParamGamma(2)};
  }

  inline void SetMassRigidBody(double _v) { mass_rb_ = _v; }
  inline void SetMassAdded(double _v) { mass_added_ = _v; }
  inline void SetDamping(double _v) { damping_ = _v; }
  inline void SetRotation(const Eigen::Quaterniond &_v) { rotation_ = _v; }
  inline void SetGravity(const Eigen::Vector3d &_v) { gravity_ = _v; }

  inline double GetMassRigidBody() const { return mass_rb_; }

  inline double GetMassAdded() const { return mass_added_; }

  inline double GetMassEffective() const { return mass_rb_ + mass_added_; }

  inline double GetDamping() const { return damping_; }

  Eigen::Quaterniond GetRotation() const { return rotation_; }
  Eigen::Vector3d GetGravity() const { return gravity_; }

  // set the final state for all axes:
  //! Fix the full position at the end time (see also the per-axis functions).
  void SetGoalPosition(const Eigen::Vector3d &_in);
  //! Fix the full velocity at the end time (see also the per-axis functions).
  void SetGoalVelocity(const Eigen::Vector3d &_in);
  //! Fix the full acceleration at the end time (see also the per-axis
  //! functions).
  void SetGoalAcceleration(const Eigen::Vector3d &_in);

  // set final state per axis:
  //! Fix the position at the end time in one axis. If not set, it is left free.
  void SetGoalPositionInAxis(const unsigned _index, const double _in) {
    axis_[_index].SetGoalPosition(_in);
  };
  //! Fix the velocity at the end time in one axis. If not set, it is left free.
  void SetGoalVelocityInAxis(const unsigned _index, const double _in) {
    axis_[_index].SetGoalVelocity(_in);
  };
  //! Fix the acceleration at the end time in one axis. If not set, it is left
  //! free.
  void SetGoalAccelerationInAxis(const unsigned _index, const double _in) {
    axis_[_index].SetGoalAcceleration(_in);
  };

  //! Reset the trajectory, clearing any end state constraints.
  void Reset(void);

  /*! Calculate the optimal trajectory of duration `_duration`.
   *
   * Calculate the full trajectory, for all the parameters defined so far.
   * @param _duration The trajectory duration, in [s].
   * @param _t_now_abs_ns Absolute time in [ns].
   */
  void Generate(const double _duration, const uint64_t _t_now_abs_ns);

  /*! Test the trajectory for input feasibility.
   *
   * Test whether the inputs required along the trajectory are within the
   * allowable limits. Note that the test either
   *   - proves feasibility,
   *   - proves infeasibility,
   *   - fails to prove anything ("indeterminate")
   *
   * The user must also specify a minimumTimeSection, which then determines the
   * precision of tests (and thus limit the number of recursion steps).
   *
   * Refer to the paper for a full discussion on these tests.
   *
   * Note that if the result is not feasible, the result is that of the first
   * section which tested infeasible/indeterminate.
   *
   * @param _f_min_allowed Minimum thrust value inputs allowed [m/s**2].
   * @param _f_max_allowed Maximum thrust value inputs allowed [m/s**2].
   * @param _w_max_allowed Maximum body rates input allowed [rad/s].
   * @param _dt_min Minimum time section to test during the recursion
   * [s].
   * @return an instance of InputFeasibilityResult.
   */
  InputFeasibilityResult CheckInputFeasibility(double _f_min_allowed,
                                               double _f_max_allowed,
                                               double _w_max_allowed,
                                               double _dt_min);

  /*! Test the trajectory for position feasibility.
   *
   * Test whether the position flown along the trajectory is feasible with
   * respect to a user defined boundary (modelled as a plane). The plane is
   * defined by a point lying on the plane, and the normal of the plane.
   *
   * No test is done for degenerate normals (of the form (0,0,0)).
   *
   * @param boundaryPoint The coordinates of any point on the plane defining the
   * boundary.
   * @param boundaryNormal A vector pointing in the allowable direction, away
   * from the boundary.
   * @return An instance of StateFeasibilityResult.
   */
  StateFeasibilityResult CheckPositionFeasibility(
      Eigen::Vector3d &_boundaryPoint, Eigen::Vector3d &_boundary_normal);

  //! Return the jerk along the trajectory at time _t
  Eigen::Vector3d GetJerk(double _t) const {
    return Eigen::Vector3d(axis_[0].GetJerk(_t), axis_[1].GetJerk(_t),
                           axis_[2].GetJerk(_t));
  };
  //! Return the acceleration along the trajectory at time _t
  Eigen::Vector3d GetAcceleration(double _t) const {
    return Eigen::Vector3d{axis_[0].GetAcceleration(_t),
                           axis_[1].GetAcceleration(_t),
                           axis_[2].GetAcceleration(_t)};
  };
  //! Return the velocity along the trajectory at time _t
  Eigen::Vector3d GetVelocity(double _t) const {
    return Eigen::Vector3d(axis_[0].GetVelocity(_t), axis_[1].GetVelocity(_t),
                           axis_[2].GetVelocity(_t));
  };
  //! Return the position along the trajectory at time t
  Eigen::Vector3d GetPosition(double _t) const {
    return Eigen::Vector3d(axis_[0].GetPosition(_t), axis_[1].GetPosition(_t),
                           axis_[2].GetPosition(_t));
  };

  inline double GetFinalTime() const { return t_final_; }

  inline uint64_t GetFinalTimeAbsNs() const { return t_final_abs_ns_; }

  //! Return the quadrocopter's normal vector along the trajectory at time _t
  inline Eigen::Vector3d GetNormalVector(double _t) const {
    // add almost zero vector to ensure we do not normalize a zero vector
    return Eigen::Vector3d{axis_[0].GetForce(_t), axis_[1].GetForce(_t),
                           axis_[2].GetForce(_t)}
        .normalized();
  };

  inline Eigen::Vector3d GetNormalVector(uint64_t _t_abs_ns) const {
    return GetNormalVector((_t_abs_ns - t_start_abs_ns_) * 1e-9);
  }
  //! Return the quadrocopter's thrust input along the trajectory at time _t
  inline double GetThrust(double _t) const {
    // return (GetAcceleration(_t) * mass_param_ + GetVelocity(_t) * damping_)
    //     .norm();
    return GetThrustVector(_t).norm();
  };

  inline double GetThrust(uint64_t _t_abs_ns) const {
    return GetThrust((_t_abs_ns - t_start_abs_ns_) * 1e-9);
  }

  inline Eigen::Vector3d GetThrustVector(double _t) const {
    return Eigen::Vector3d{axis_[0].GetForce(_t), axis_[1].GetForce(_t),
                           axis_[2].GetForce(_t)};
  }

  inline Eigen::Vector3d GetThrustVector(uint64_t _t_abs_ns) const {
    return GetThrustVector((_t_abs_ns - t_start_abs_ns_) * 1e-9);
  }

  inline uint64_t TimeLeft(uint64_t _t_now_ns) const {
    return t_final_abs_ns_ > _t_now_ns ? (t_final_abs_ns_ - _t_now_ns) : 0;
  }

  inline uint64_t TimeOnTrajectoryNs(uint64_t _t_now_ns) const {
    return _t_now_ns > t_start_abs_ns_ ? (_t_now_ns - t_start_abs_ns_) : 0;
  }

  inline double TimeOnTrajectory(uint64_t _t_now_ns) const {
    return TimeOnTrajectoryNs(_t_now_ns) * 1e-9;
  }

  inline double TimeLeft(double _t) const {
    return _t < t_final_ ? (t_final_ - _t) : 0.0;
  }

  inline uint64_t GetStartTimeNs() const {
    return t_start_abs_ns_;
  }

  /*! Return the quadrocopter's body rates along the trajectory at time _t
   *
   * Returns the required body rates along the trajectory. These are expressed
   * in the world frame (in which the trajectory was planned). To convert them
   * to (p,q,r), this needs to be rotated by the quadrocopter's attitude.
   *
   * The rates are calculated by taking the rates required to rotate the
   * quadrocopter's normal at time `_t` to that at time `_t+dt`. Therefore, if
   * the trajectory is used as implicit MPC control law, the value dt should
   * correspond to the controller period.
   *
   * @param _t Time along the trajectory to be evaluated [s].
   * @param timeStep The timestep size for the finite differencing [s].
   * @return The body rates, expressed in the inertial frame [rad/s]
   */
  Eigen::Vector3d GetOmega(double _t, double timeStep) const;

  //! Return the total cost of the trajectory.
  double GetCost(void) const {
    return axis_[0].GetCost() + axis_[1].GetCost() + axis_[2].GetCost();
  };

  //! Return the parameter defining the trajectory.
  inline double GetAxisParamAlpha(int i) const {
    return axis_[i].GetParamAlpha();
  };
  //! Return the parameter defining the trajectory.
  inline double GetAxisParamBeta(int i) const {
    return axis_[i].GetParamBeta();
  };
  //! Return the parameter defining the trajectory.
  inline double GetAxisParamGamma(int i) const {
    return axis_[i].GetParamGamma();
  };

  inline Eigen::Vector3d ToWorld(const Eigen::Vector3d &_vec) const {
    return rotation_ * _vec;
  }

 private:
  //! Test a section of the trajectory for input feasibility (recursion).
  InputFeasibilityResult CheckInputFeasibilitySection(double _f_min_allowed,
                                                      double _f_max_allowed,
                                                      double _w_max_allowed,
                                                      double _t1, double _t2,
                                                      double _dt_min);

  std::array<SingleAxisTrajectory, 3> axis_;
  uint64_t t_start_abs_ns_{0};
  uint64_t t_final_abs_ns_{0};
  double t_final_{0.0};  //!< trajectory end time [s]
  double damping_{5.4};
  double mass_rb_{1.5};
  double mass_added_{1.5};
  double thrust_max_{0.0};
  double thrust_min_{0.0};

  /**
   * @brief Rotation of the used inertial frame relative to the world frame.
   * To get trajectory vectors/points in world frame, you would write
   * x_world = rotation_ * x_trajectory
   */
  Eigen::Quaterniond rotation_;

  Eigen::Vector3d gravity_{0.0, 0.0, 0.0};
};
};  // namespace minimum_jerk
};  // namespace rapid_trajectories
