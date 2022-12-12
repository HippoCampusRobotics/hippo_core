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
