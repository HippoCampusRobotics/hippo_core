#pragma once

#include <eigen3/Eigen/Dense>
#include <hippo_common/tf2_utils.hpp>

namespace hippo_control {
namespace attitude_control {

template <typename T>
int sgn(T value) {
  return (T(0) < value) - (value < T(0));
}

class QuaternionController {
 public:
  Eigen::Vector3d Update(const Eigen::Vector3d &desired_heading,
                         double desired_roll,
                         const Eigen::Quaterniond &orientation);

  inline double &gain() { return gain_; }
  inline double &roll_weight() { return roll_weight_; }

 private:
  Eigen::Quaterniond ReducedQuaternionCommand(
      const Eigen::Vector3d &desired_heading);
  inline Eigen::Quaterniond FullQuaternionCommand(
      const Eigen::Vector3d &_heading, double _roll) {
    return hippo_common::tf2_utils::QuaternionFromHeading(_heading, _roll);
  }
  Eigen::Quaterniond MixedQuaternionCommand(const Eigen::Vector3d &_heading,
                                            double _roll);
  inline Eigen::Vector3d CurrentForwardAxis() {
    return orientation_ * Eigen::Vector3d::UnitX();
  }
  Eigen::Quaterniond orientation_{1.0, 0.0, 0.0, 0.0};
  /// @brief 1.0 means full attitude control, 0.0 reduced (ignoring roll)
  /// control only.
  double roll_weight_{0.5};
  double gain_{1.0};
};

}  // namespace attitude_control
}  // namespace hippo_control
