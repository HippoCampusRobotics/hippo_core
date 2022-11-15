#pragma once
#include <eigen3/Eigen/Dense>
#include <vector>
namespace hippo_control {
namespace attitude_control {
class GeometricAttitudeControl {
 public:
  /**
   * @brief Computes the controller output.
   *
   * @param _orientation Current orientation as quaternion.
   * @param _angular_velocity Current angular velocity (rad/s).
   * @return Eigen::Vector3d Control output as vector of roll, pitch and yaw
   * torque normalized to a maximum of 1.0.
   */
  Eigen::Vector3d Update(const Eigen::Quaterniond &_orientation,
                         const Eigen::Vector3d &_angular_velocity);
  void SetOrientationTarget(const Eigen::Quaterniond &_target) {
    orientation_target_ = _target;
  }
  void SetOrientationTarget(const double _roll, const double _pitch,
                            const double _yaw);
  void SetAngularVelocityTarget(const Eigen::Vector3d &_target) {
    v_angular_target_ = _target;
  }
  void SetAngularVelocityTarget(const double _roll, const double _pitch,
                                const double _yaw) {
    v_angular_target_.x() = _roll;
    v_angular_target_.y() = _pitch;
    v_angular_target_.z() = _yaw;
  }
  void SetRollGainP(double _gain) { p_gains_.x() = _gain; }

  void SetPitchGainP(double _gain) { p_gains_.y() = _gain; }

  void SetYawGainP(double _gain) { p_gains_.z() = _gain; }

  void SetRollGainD(double _gain) { d_gains_.x() = _gain; }

  void SetPitchGainD(double _gain) { d_gains_.y() = _gain; }

  void SetYawGainD(double _gain) { d_gains_.z() = _gain; }

  /**
   * @brief Set proportional gains for roll, pitch and yaw angle.
   *
   * @param _gains
   */
  void SetPgains(const Eigen::Array3d &_gains) {
    p_gains_ = Eigen::Array3d(_gains);
  }
  /**
   * @overload
   */
  void SetPgains(const std::vector<double> &_gains) {
    p_gains_ = Eigen::Array3d(_gains.data());
  }
  /**
   * @overload
   */
  void SetPgains(const std::array<double, 3> &_gains) {
    p_gains_ = Eigen::Array3d(_gains.data());
  }

  /**
   * @brief Set the derivate gains for roll, pitch and yaw angular velocity.
   *
   * @param _gains
   */
  void SetDgains(const Eigen::Array3d &_gains) {
    d_gains_ = Eigen::Array3d(_gains);
  }
  /**
   * @overload
   */
  void SetDgains(const std::vector<double> &_gains) {
    d_gains_ = Eigen::Array3d(_gains.data());
  }
  /**
   * @overload
   */
  void SetDgains(const std::array<double, 3> &_gains) {
    d_gains_ = Eigen::Array3d(_gains.data());
  }

 private:
  // use array instead of vector to simplify coefficient wise multiplication
  Eigen::Array3d p_gains_{1.0, 1.0, 1.0};
  Eigen::Array3d d_gains_{0.1, 0.1, 0.1};

  Eigen::Vector3d v_angular_target_{0.0, 0.0, 0.0};
  Eigen::Quaterniond orientation_target_{1.0, 0.0, 0.0, 0.0};
};
}  // namespace attitude_control
}  // namespace hippo_control
