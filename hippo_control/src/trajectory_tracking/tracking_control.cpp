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
  thrust_ = RequiredThrust(_feed_forward_thrust);
  return AttitudeFromThrust(thrust_, roll_desired_);
}

Eigen::Vector3d TrackingController::RequiredThrust(
    const Eigen::Vector3d &_feed_forward) {
  thrust_ = Eigen::Vector3d{(position_desired_ - position_) * position_gain_ +
                            (velocity_desired_ - velocity_) * velocity_gain_ +
                            _feed_forward};
  return thrust_;
}
}  // namespace trajectory_tracking
}  // namespace hippo_control
