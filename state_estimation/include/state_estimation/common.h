#pragma once

#include <stdint.h>
#define EIGEN_MATRIXBASE_PLUGIN <state_estimation/matrix_base_addon.h>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#define LOGGER_NAME "Ekf"
#define EKF_INFO_ONCE(...) RCLCPP_INFO_ONCE(rclcpp::get_logger(LOGGER_NAME), ##__VA_ARGS__)
#define EKF_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), ##__VA_ARGS__)
#define EKF_INFO(...) RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), ##__VA_ARGS__)
#define EKF_WARN(...) RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), ##__VA_ARGS__)
#define EKF_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), ##__VA_ARGS__)

// #define EKF_INFO_ONCE(X, ...) printf(X "\n", ##__VA_ARGS__)
// #define EKF_DEBUG(X, ...) printf(X "\n", ##__VA_ARGS__)
// #define EKF_INFO(X, ...) printf(X "\n", ##__VA_ARGS__)
// #define EKF_WARN(X, ...) printf(X "\n", ##__VA_ARGS__)
// #define EKF_ERROR(X, ...) printf(X "\n", ##__VA_ARGS__)

constexpr uint64_t kVisionMaxIntervalUs = (uint64_t)200e3;
constexpr uint64_t kBaroMaxIntervalUs = (uint64_t)200e3;
constexpr uint64_t kFilterUpdatePeriodUs = (uint64_t)10e3;
constexpr double kGravity = 9.81;
constexpr double kPi = 3.1415926;
constexpr uint64_t kBadAccelProbation = (uint64_t)10e6;


constexpr int kNumStates{16};
typedef Eigen::Matrix<double, kNumStates, 1> StateVectord;
typedef Eigen::Matrix<double, kNumStates, kNumStates> StateMatrixd;

union ControlStatus {
  struct {
    uint32_t tilt_align: 1;
    uint32_t yaw_align: 1;
    uint32_t in_air: 1;
    uint32_t baro_height: 1;
    uint32_t vision_height: 1;
    uint32_t vision_position: 1;
    uint32_t vision_yaw: 1;
    uint32_t vision_orientation: 1;
    uint32_t vehicle_at_rest: 1;
  } flags;
  uint32_t value;
};

union FaultStatus {
  struct {
    bool bad_heading: 1;
    bool bad_velocity_x: 1;
    bool bad_velocity_y: 1;
    bool bad_velocity_z: 1;
    bool bad_position_x: 1;
    bool bad_position_y: 1;
    bool bad_position_z: 1;
    bool bad_acceleration_bias: 1;
    bool bad_acceleration_vertical: 1;
    bool bad_acceleration_clipping: 1;
  }flags;
  uint32_t value;
};

union InnovationFault {
  struct {
    bool reject_horizontal_velocity: 1;
    bool reject_vertical_velocity: 1;
    bool reject_horizontal_position: 1;
    bool reject_vertical_position: 1;
    bool reject_yaw: 1;
    bool reject_baro: 1;
  }flags;
  uint16_t value;
};

struct StateSample {
  Eigen::Quaterniond orientation;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d delta_angle_bias;
  Eigen::Vector3d delta_velocity_bias;
};

struct ImuSample {
  uint64_t time_us{0};
  Eigen::Vector3d delta_angle;
  Eigen::Vector3d delta_velocity;
  double delta_angle_dt;
  double delta_velocity_dt;
  bool delta_velocity_clipping[3]{};
};

struct BaroSample {
  uint64_t time_us{0};
  double height;
};

struct VisionSample {
  uint64_t time_us{0};
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d position_variance;
  Eigen::Vector3d velocity_variance;
  double angular_variance;
};

struct OutputSample {
  uint64_t time_us{0};
  Eigen::Quaterniond orientation;
  Eigen::Vector3d velocity;
  Eigen::Vector3d position;
};

struct ImuBiasEstimation {
  double time_constant{0.5};
  double accel_bias_magnitude_limit{0.4};  ///< max acceleration bias magnitude
  double accel_magnitude_limit{25.0};  ///< max acceleration magnitude for which
                                       ///< bias estimation will be applied.
  double gyro_magnitude_limit{
      3.0};  /// < max gyroscope rate vector magnitude for which bias estimation
             /// will be applied.
};

enum class HeightMode {
  kBaro,
  kVision,

};

struct Settings {
  uint64_t min_observation_interval_us{(uint64_t)20e3};
  uint64_t vision_delay_us{(uint64_t)100e3};
  uint64_t baro_delay_us{(uint64_t)0};

  HeightMode height_mode{HeightMode::kBaro};

  double gyro_noise{1.5e-2};
  double accel_noise{3.5e-1};

  double gyro_bias_noise{1.0e-3};
  double accel_bias_noise{1.0e-2};

  double baro_noise{0.02};
  double baro_innovation_gate{5.0};
  double baro_height_offset{0.0};

  uint64_t bad_accel_reset_delay_us{500000};

  const uint64_t reset_timeout_max_us{7000000};

  double velocity_time_constant{0.25};
  double position_time_constant{0.25};

  ImuBiasEstimation imu_bias_estimation;

  double initial_tilt_error{0.1};
  double initial_gyro_bias{0.1};
  double initial_accel_bias{0.2};

  double vision_innovation_gate{5.0};

  //////////////////////////////////////////////////////////////////////////////
  // state limits
  Eigen::Vector3d position_upper_limit = {2.0, 4.0, 0.5}; ///< constrains the valid position values of the delayed ekf state.
  Eigen::Vector3d position_lower_limit = {0.0, 0.0, -2.0}; ///< constrains the valid position values of the delayed ekf state.
  double velocity_limit{15.0}; ///< constrains the valid velocity values of the delayed ekf state.
  double delta_angle_bias_limit{5*M_PI * kFilterUpdatePeriodUs * 1e-6};

  Eigen::Vector3d velocity_noise = {0.1, 0.1, 0.1};
  Eigen::Vector3d position_noise = {0.1, 0.1, 0.1};

  Eigen::Vector3d imu_position_body = {0.0, 0.0, 0.0};

  double vertical_innovation_test_limit{3.0};

  double heading_innovation_gate{2.6};

};

