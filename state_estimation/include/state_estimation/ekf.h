#pragma once
#include <state_estimation/alpha_filter.h>
#include <state_estimation/common.h>
#include <state_estimation/interface.h>

#include <eigen3/Eigen/Dense>

namespace StateIndex {
static constexpr int qw = 0;
static constexpr int qx = 1;
static constexpr int qy = 2;
static constexpr int qz = 3;
static constexpr int velocity_x = 4;
static constexpr int velocity_y = 5;
static constexpr int velocity_z = 6;
static constexpr int position_x = 7;
static constexpr int position_y = 8;
static constexpr int position_z = 9;
static constexpr int delta_angle_bias_x = 10;
static constexpr int delta_angle_bias_y = 11;
static constexpr int delta_angle_bias_z = 12;
static constexpr int delta_velocity_bias_x = 13;
static constexpr int delta_velocity_bias_y = 14;
static constexpr int delta_velocity_bias_z = 15;
static constexpr int NumStates = 16;
}  // namespace StateIndex

class Ekf final : public Interface {
 public:
  Ekf() = default;
  virtual ~Ekf() = default;

  bool Init(uint64_t timestamp_us) override;

  void UpdateDeadreckoningStatus();

  bool Update();
  void PredictState();
  void PredictCovariance();
  void CalculateOutputState(const ImuSample &imu_sample);
  void CorrectOutputBuffer(const Eigen::Vector3d &velocity_correction,
                           const Eigen::Vector3d &position_correction);
  void Fuse(const StateVectord &K, double innovation);
  bool FuseHorizontalPosition(const Eigen::Vector3d &innovation,
                              const Eigen::Vector2d &innovation_gate,
                              const Eigen::Vector3d &observation_var,
                              Eigen::Vector3d &innovation_var,
                              Eigen::Vector2d &test_ratio, bool inhibit_gate);
  bool FuseVerticalPosition(const Eigen::Vector3d &innovation,
                            const Eigen::Vector2d &innovation_gate,
                            const Eigen::Vector3d &observation_var,
                            Eigen::Vector3d &innovation_var,
                            Eigen::Vector2d &test_ratio);
  void FuseVelocityPositionHeight(const double innovation,
                                  const double innovation_var,
                                  const int observation_index);
  void FuseOrientation();
  void FuseHeading();
  void FuseYaw321(double yaw, double yaw_var, bool zero_innovation);
  void FuseYaw312(double yaw, double yaw_var, bool zero_innovation);
  void UpdateQuaternion(const double innovation, const double variance,
                        const double gate_sigma,
                        const Eigen::Vector4d &yaw_jacobian);
  void FixCovarianceErrors(bool force_symmetry);
  void SetVelocityPositionFaultStatus(const int index, const bool is_bad);
  bool InitTilt();
  void InitCovariance();
  void InitQuaternionCovariances();
  void ResetQuaternionCovariance();
  void SetZeroQuaternionCovariance();
  void UncorrelateQuaternion();
  void UncorrelateHorizontalPositionSetTo(const Eigen::Vector2d &position);
  void UncorrelateHorizontalVelocitySetTo(const Eigen::Vector2d &velocity);
  bool CheckAndFixCovarianceUpdate(const StateMatrixd &KHP);
  Eigen::Vector3d CalcRotationVectorVariances();

  //////////////////////////////////////////////////////////////////////////////
  // Innovation Getter /////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  // Vision
  void VisionPositionInnovation(double position[3]) const;
  void VisionPositionInnovationVar(double position[3]) const;
  void VisionPositionInnovationRatio(double &horizonal, double &vertical) const;
  // Baro
  void BaroHeightInnovation(double &baro_height_innovation) const;
  void BaroHeightInnovationVar(double &baro_height_innovation_var) const;
  void BaroHeightInnovationRatio(double &baro_height_innovation_ratio) const;

  // Heading
  void HeadingInnovation(double &heading_innovation) const;
  void HeadingInnovationVar(double &heading_innovation_var) const;
  void HeadingInnovationRatio(double &heading_innovation_ratio) const;

  //////////////////////////////////////////////////////////////////////////////
  // State Getter //////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  StateVectord StateAtFusionTime() const;
  Eigen::Quaterniond Orientation() const { return output_new_.orientation; }
  Eigen::Vector3d Position() const { return output_new_.position; }
  Eigen::Vector3d GyroBias() const {
    return state_.delta_angle_bias / dt_average_;
  }
  Eigen::Vector3d AccelerationBias() const {
    return state_.delta_velocity_bias / dt_average_;
  }
  Eigen::Vector3d GyroBiasVariance() const {
    constexpr auto x = StateIndex::delta_angle_bias_x;
    constexpr auto y = StateIndex::delta_angle_bias_y;
    constexpr auto z = StateIndex::delta_angle_bias_z;
    return Eigen::Vector3d{P_(x, x), P_(y, y), P_(z, z)} / dt_average_;
  }
  Eigen::Vector3d AccelerationBiasVariance() const {
    constexpr auto x = StateIndex::delta_velocity_bias_x;
    constexpr auto y = StateIndex::delta_velocity_bias_y;
    constexpr auto z = StateIndex::delta_velocity_bias_z;
    return Eigen::Vector3d{P_(x, x), P_(y, y), P_(z, z)} / dt_average_;
  }
  Eigen::Vector3d Velocity() const {
    return Eigen::Vector3d(output_new_.velocity);
  }

  //////////////////////////////////////////////////////////////////////////////
  // Helper ////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  void AlignOutputFilter();
  void ConstrainStates();
  void UpdateSensorFusion();
  void UpdateVisionFusion();
  void UpdateHeightSensorTimeout();
  void UpdateHeightFusion();
  void SetControlBaroHeight();
  void SetControlVisionHeight();
  void IncreaseQuaternionYawErrorVariance(double yaw_var);
  void ResetHeight();
  void ResetYawGyroBiasCov();
  void ResetHorizontalPosition();
  void ResetHorizontalPositionTo(const Eigen::Vector2d &position);
  void ResetHorizontalPositionToVision();
  void ResetVerticalVelocityTo(double velocity);
  void ResetVerticalPositionTo(double position);
  bool ResetYawToVision();
  void ResetVelocity();
  void ResetHorizontalVelocityToZero();
  void ResetHorizontalVelocityTo(const Eigen::Vector2d &velocity);
  void ResetQuaternionStateYaw(double yaw, double yaw_var, bool update_buffer);
  void UpdateBaroHeightOffset();
  void StartBaroHeightFusion();
  void StartVisionPositionFusion();
  void StartVisionYawFusion();
  void StopVisionHeightFusion();
  void StopVisionFusion();
  void StopVisionPositionFusion();
  void StopVisionYawFusion();
  void CheckVerticalAccelHealth();
  inline Eigen::Quaterniond QuaternionFromDeltaAngle(
      const Eigen::Vector3d &delta_angle, double dt) {
    const Eigen::Vector3d axis = Eigen::Vector3d(delta_angle).normalized();
    const double angle = dt * delta_angle.norm();
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
  }

  bool IsTimedOut(uint64_t timestamp_us, uint64_t timeout_period_us) {
    return (timestamp_us + timeout_period_us) < time_last_imu_;
  }
  bool IsRecent(uint64_t timestamp_us, uint64_t interval) const {
    return timestamp_us + interval > time_last_imu_;
  }
  template <int Width>
  void MakeCovarianceBlockSymmetric(int first) {
    if (Width > 1) {
      for (int row = first + 1; row < first + Width; ++row) {
        for (int col = first; col < row; ++col) {
          double tmp = (P_(row, col) + P_(col, row)) / 2.0;
          P_(row, col) = P_(col, row) = tmp;
        }
      }
    }
  }

  template <int Width>
  void MakeCovarianceSymmetric(int first) {
    MakeCovarianceBlockSymmetric<Width>(first);
    for (int row = first; row < first + Width; ++row) {
      for (int col = 0; col < first; ++col) {
        double tmp = (P_(row, col) + P_(col, row)) / 2.0;
        P_(row, col) = P_(col, row) = tmp;
      }
      for (int col = first + Width; col < kNumStates; ++col) {
        double tmp = (P_(row, col) + P_(col, row)) / 2.0;
        P_(row, col) = P_(col, row) = tmp;
      }
    }
  }

 private:
  void Reset();
  bool InitFilter();

  StateSample state_{};  ///< Filtered state at delayed time horizon.
  bool filter_initialized_{false};

  bool baro_data_ready_{false};
  bool vision_data_ready_{false};

  Eigen::Matrix3d R_to_earth_;

  bool baro_height_faulty_{false};

  uint64_t time_bad_vertical_accel_us_{0};
  uint64_t time_good_vertical_accel_us_{0};
  bool bad_vertical_accel_detected{false};
  unsigned int accel_clip_counter_{0};

  uint64_t time_acceleration_bias_check_us_{0};
  uint64_t delta_time_baro_us_{0};

  double yaw_delta_ef_{0.0};
  double yaw_rate_lpf_ef_{0.0};

  bool is_first_imu_sample_{true};
  int baro_counter_{0};

  AlphaFilter<Eigen::Vector3d> acceleration_filter_{0.1};
  AlphaFilter<Eigen::Vector3d> acceleration_magnitude_filter{0.1};
  AlphaFilter<Eigen::Vector3d> gyro_filter_{0.1};

  bool accel_bias_inhibited_[3]{};
  double gyro_magnitude_filtered_{0.0};
  double accel_magnitude_filtered_{0.0};
  Eigen::Vector3d accel_vec_filtered_;
  Eigen::Vector3d prev_delta_velocity_bias_var_;

  uint64_t time_last_horizontal_position_fuse_{0};
  uint64_t time_last_height_fuse_{0};

  double baro_height_offset_{0.0};
  double height_sensor_offset_{0.0};
  double dt_average_{kFilterUpdatePeriodUs * 1e-6};

  uint64_t time_deadreckoning_{0};

  Eigen::Vector3d delta_angle_correction_;
  Eigen::Vector3d velocity_error_integral_;
  Eigen::Vector3d position_error_integral_;
  Eigen::Vector3d output_tracking_error_;

  double vertical_position_innovation_ratio_{0.0};
  uint64_t vertical_position_fuse_attempt_time_us_{0};

  StateMatrixd P_;
  Eigen::Vector3d delta_velocity_bias_var_accumulated_;
  Eigen::Vector3d delta_angle_bias_var_accumulated_;

  Eigen::Vector3d vision_position_innovation_;
  Eigen::Vector3d vision_position_innovation_var_;

  Eigen::Vector3d baro_height_innovation_;
  Eigen::Vector3d baro_height_innovation_var_;

  double heading_innovation_{0.0};
  double heading_innovation_var_{0.0};

  Eigen::Vector3d angular_rate_delayed_raw_;
  double last_static_yaw_{0.0};
};
