#pragma once

#include <eigen3/Eigen/Dense>

namespace hippo_control {
namespace rate_control {
class Controller {
 public:
  void SetRollGainP(double _gain) { p_gain_.x() = _gain; }
  void SetRollGainI(double _gain) { i_gain_.x() = _gain; }
  void SetRollGainD(double _gain) { d_gain_.x() = _gain; }
  void SetRollFeedForwardGain(double _gain) { feed_forward_gain_.x() = _gain; }
  void SetRollIntegralLimit(double _limit) { integral_limit_.x() = _limit; }

  void SetPitchGainP(double _gain) { p_gain_.y() = _gain; }
  void SetPitchGainI(double _gain) { i_gain_.y() = _gain; }
  void SetPitchGainD(double _gain) { d_gain_.y() = _gain; }
  void SetPitchFeedForwardGain(double _gain) { feed_forward_gain_.y() = _gain; }
  void SetPitchIntegralLimit(double _limit) { integral_limit_.y() = _limit; }

  void SetYawGainP(double _gain) { p_gain_.z() = _gain; }
  void SetYawGainI(double _gain) { i_gain_.z() = _gain; }
  void SetYawGainD(double _gain) { d_gain_.z() = _gain; }
  void SetYawFeedForwardGain(double _gain) { feed_forward_gain_.z() = _gain; }
  void SetYawIntegralLimit(double _limit) { integral_limit_.z() = _limit; }

  void SetZeroIntegralThreshold(double _v) { zero_integral_threshold_ = _v; }

  void ResetIntegral() { integral_.setZero(); }

  Eigen::Vector3d Update(const Eigen::Vector3d &_rate,
                         const Eigen::Vector3d &_rate_setpoint,
                         const Eigen::Vector3d &_angular_acceleration,
                         const double _dt);

 private:
  void UpdateIntegral(Eigen::Vector3d &_rate_error, const double _dt);
  Eigen::Vector3d p_gain_;
  Eigen::Vector3d i_gain_;
  Eigen::Vector3d d_gain_;
  Eigen::Vector3d integral_limit_;
  Eigen::Vector3d feed_forward_gain_;
  double zero_integral_threshold_;

  Eigen::Vector3d integral_;
};
}  // namespace rate_control
}  // namespace hippo_control
