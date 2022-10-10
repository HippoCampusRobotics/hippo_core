#pragma once
#include <state_estimation/common.h>
#include <state_estimation/ring_buffer.h>

#include <eigen3/Eigen/Dense>

class Interface {
 public:
  void SetImuData(const ImuSample &imu_sample);
  void SetBaroData(const BaroSample &baro_sample);
  void SetVisionData(const VisionSample &vision_sample);
  bool AttitudeValid() const { return control_status_.flags.tilt_align; }

  const decltype(FaultStatus::flags) &FaultStatusFlag() const {
    return fault_status_.flags;
  }
  Settings settings_;

 protected:
  Interface() = default;
  virtual ~Interface() = default;
  virtual bool Init(uint64_t timestamp_us) = 0;
  int imu_buffer_length_{0};
  int observation_buffer_length_{0};
  double imu_dt_average_{0.0};

  ImuSample imu_sample_delayed_{};

  BaroSample baro_sample_delayed_{};
  VisionSample vision_sample_delayed_{};

  OutputSample output_new_{};
  ImuSample latest_imu_sample_{};
  Eigen::Vector3d velocity_rel_imu_body_enu_;
  Eigen::Matrix3d R_to_earth_now_;
  Eigen::Vector3d velocity_derivative_;

  InnovationFault innovation_check_status_{};
  FaultStatus fault_status_{};
  ControlStatus control_status_{};
  ControlStatus control_status_prev_{};

  bool imu_updated_{false};
  bool initialized_{false};

  RingBuffer<ImuSample> imu_buffer_;
  RingBuffer<OutputSample> output_buffer_;
  RingBuffer<BaroSample> baro_buffer_;
  RingBuffer<VisionSample> vision_buffer_;

  uint64_t time_last_imu_{0};
  uint64_t time_last_baro_{0};
  uint64_t time_last_vision_{0};

  Eigen::Vector2d baro_height_test_ratio_;
  Eigen::Vector2d vision_position_test_ratio_;
  double yaw_test_ratio_{};

  uint64_t time_last_in_air_us_{0};
  uint64_t time_last_on_ground_us_{0};

  bool InitInterface(uint64_t timestamp_us);

 private:
  uint64_t min_observation_interval_us_{0};
  Eigen::Vector3d delta_angle_previous_;
  Eigen::Vector3d delta_velocity_previous_;
  uint64_t baro_timestamp_sum{0};
};
