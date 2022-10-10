#include <state_estimation/sensor_sim/imu.h>
namespace sensor_sim {
namespace sensor {
Imu::Imu(std::shared_ptr<Ekf> ekf) : Sensor(ekf) {}
Imu::~Imu() {}

void Imu::Send(uint64_t time_us) {
  const double dt = double((time_us - time_last_data_sent_us_) * 1e-6);
  ImuSample sample;
  sample.time_us = time_us;
  sample.delta_angle_dt = dt;
  sample.delta_velocity_dt = dt;
  sample.delta_angle = gyro_data_ * sample.delta_angle_dt;
  sample.delta_velocity = accel_data_ * sample.delta_velocity_dt;
  ekf_->SetImuData(sample);
}

void Imu::SetData(const Eigen::Vector3d &accel, const Eigen::Vector3d &gyro) {
  SetAccelerationData(accel);
  SetGyroData(gyro);
}

}  // namespace sensor
}  // namespace sensor_sim
