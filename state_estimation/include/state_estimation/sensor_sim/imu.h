#pragma once

#include <eigen3/Eigen/Dense>

#include "sensor.h"

namespace sensor_sim {
namespace sensor {
class Imu : public Sensor {
 public:
  Imu(std::shared_ptr<Ekf> ekf);
  ~Imu();

  void SetData(const Eigen::Vector3d &acceleration,
               const Eigen::Vector3d &gyro);
  inline void SetAccelerationData(const Eigen::Vector3d &acceleration) {
    accel_data_ = acceleration;
  }
  inline void SetGyroData(const Eigen::Vector3d &gyro) { gyro_data_ = gyro; }

 private:
  Eigen::Vector3d accel_data_;
  Eigen::Vector3d gyro_data_;
  void Send(uint64_t time_us) override;
};
}  // namespace sensor
}  // namespace sensor_sim
