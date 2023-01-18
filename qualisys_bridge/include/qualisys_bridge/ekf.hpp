#pragma once
#include <eigen3/Eigen/Dense>

namespace qualisys_bridge {
class Ekf {
 public:
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  typedef Eigen::Matrix<double, 15, 15> Matrix15d;
  typedef Eigen::Matrix<double, 6, 15> Matrix6_15d;
  typedef Eigen::Matrix<double, 15, 6> Matrix15_6d;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, 15, 1> Vector15d;
  Ekf();
  bool Init(const Matrix15d &_process_covariance,
            const Matrix6d &_measurement_covariance, int _frequency);
  bool SetInitialCondition(double _time,
                           const Eigen::Quaterniond &_orientation_measurement,
                           const Eigen::Vector3d &_position_measurement);

  inline bool IsReady() const { return state_ == State::READY; }

  inline void Reset() { state_ = State::INIT_POSE; }

  void Predict(double _time);
  void Update(const Eigen::Quaterniond &_orientation_measurement,
              const Eigen::Vector3d &_position_measurement);

  inline Eigen::Vector3d GetPosition() const { return position_; }
  inline Eigen::Vector3d GetLinearVelocity() const { return velocity_linear_; }
  inline Eigen::Vector3d GetLinearAcceleration() const {
    return acceleration_linear_;
  }
  inline Eigen::Vector3d GetAngularVelocity() const {
    return velocity_angular_;
  }
  inline Eigen::Quaterniond GetOrientation() const { return orientation_; }

  inline Eigen::Matrix<double, 15, 15> GetStateCovariance() const {
    return state_covariance_;
  }

 private:
  enum class State { INIT_POSE, INIT_TWIST, INIT_ACCEL, READY };
  State state_;
  /// @brief orientation of the body relative to the world frame
  Eigen::Quaterniond orientation_;
  /// @brief position in world frame
  Eigen::Vector3d position_;
  /// @brief angular velocity in body frame
  Eigen::Vector3d velocity_angular_;
  /// @brief linear velocity in world frame
  Eigen::Vector3d velocity_linear_;
  /// @brief linear acceleration in world frame
  Eigen::Vector3d acceleration_linear_;

  Matrix15d state_covariance_;
  Matrix15d process_covariance_;
  Matrix6d measurement_covariance_;

  Matrix15d process_jacobian_;
  Matrix6_15d measurement_jacobian_;
  Matrix15d process_noise_jacobian_;
  Matrix6d measurement_noise_jacobian_;

  double time_last_;
  double expected_update_interval_;
};
}  // namespace qualisys_bridge
