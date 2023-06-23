#include <eigen3/Eigen/Dense>
namespace hippo_control {
namespace trajectory_tracking {
class TrackingController {
 public:
  TrackingController();
  void SetDesiredVelocity(const Eigen::Vector3d &_v) { velocity_desired_ = _v; }
  void SetDesiredPosition(const Eigen::Vector3d &_v) { position_desired_ = _v; }
  void SetDesiredState(const Eigen::Vector3d &_position,
                       const Eigen::Vector3d &_velocity) {
    position_desired_ = _position;
    velocity_desired_ = _velocity;
  };
  void SetCurrentVelocity(const Eigen::Vector3d &_v) { velocity_ = _v; }
  void SetCurrentPosition(const Eigen::Vector3d &_v) { position_ = _v; }
  void SetCurrentState(const Eigen::Vector3d &_position,
                       const Eigen::Vector3d &_velocity) {
    position_ = _position;
    velocity_ = _velocity;
  }
  void SetPositionGain(double _gain) { position_gain_ = _gain; }
  void SetVelocityGain(double _gain) { velocity_gain_ = _gain; }
  /**
   * @brief Applies a feedback control law based on:
   * Mellinger, Daniel and Vijay R. Kumar. “Minimum snap trajectory generation
   * and control for quadrotors.” 2011 IEEE International Conference on Robotics
   * and Automation (2011): 2520-2525.
   *
   * @param _feed_forward A feedforward term (based on the vehicle dynamics for
   * example) can be added.
   * @return Eigen::Vector3d
   */
  Eigen::Vector3d RequiredThrust(const Eigen::Vector3d &_feed_forward) const {
    return Eigen::Vector3d{(position_desired_ - position_) * position_gain_ +
                           (velocity_desired_ - velocity_) * velocity_gain_ +
                           _feed_forward};
  }
  /**
   * @brief Computes the attitude from a thrust vector and the roll angle. In
   * case the thrust vector is close to the zero vector, the attitude based on
   * the last time the thrust was non-zero is used.
   *
   * @param _thrust The required thrust vector that specifies the vehicle's
   * attitude with the exception of the roll angle.
   * @param _roll The thrust vector /a _thrust does not restrict the roll angle.
   * Therefore, it needs to be specified explicitly.
   * @return Eigen::Quaterniond
   */
  Eigen::Quaterniond AttitudeFromThrust(const Eigen::Vector3d &_thrust,
                                        double _roll);

 private:
  Eigen::Vector3d velocity_{0.0, 0.0, 0.0};
  Eigen::Vector3d position_{0.0, 0.0, 0.0};
  Eigen::Vector3d velocity_desired_{0.0, 0.0, 0.0};
  Eigen::Vector3d position_desired_{0.0, 0.0, 0.0};

  Eigen::Quaterniond last_valid_attitude_{0.0, 0.0, 0.0, 1.0};

  double position_gain_{1.0};
  double velocity_gain_{1.0};
};
}  // namespace trajectory_tracking
}  // namespace hippo_control
