#include <state_estimation/sensor_sim/sensor_sim.h>

SensorSim::SensorSim(std::shared_ptr<Ekf> ekf)
    : imu_(ekf), baro_(ekf), vision_{ekf}, ekf_{ekf} {
  SetSensorRateToDefault();
  SetSensorDataToDefault();
  StartBasicSensors();
}

SensorSim::~SensorSim() {}

void SensorSim::SetSensorDataToDefault() {
  baro_.SetData(0.0);
  imu_.SetData(Eigen::Vector3d{0.0, 0.0, kGravity},
               Eigen::Vector3d{0.0, 0.0, 0.0});
  vision_.SetData(vision_.DataAtRest());
}

void SensorSim::SetSensorRateToDefault() {
  imu_.SetRate(200.0);
  baro_.SetRate(50.0);
  vision_.SetRate(5.0);
}

void SensorSim::StartBasicSensors() {
  baro_.Start();
  imu_.Start();
  vision_.Start();
}

void SensorSim::RunSeconds(double duration) {
  RunMicros(uint64_t(duration * 1e6));
}

void SensorSim::RunMicros(uint64_t duration_us) {
  const uint64_t start = time_us_;
  while (time_us_ < start + duration_us) {
    time_us_ += 1000;
    bool update_imu = imu_.ShouldSend(time_us_);
    UpdateSensors();
    if (update_imu) {
      ekf_->Update();
    }
  }
}

void SensorSim::UpdateSensors() {
  imu_.Update(time_us_);
  baro_.Update(time_us_);
  vision_.Update(time_us_);
}

void SensorSim::SetImuBias(const Eigen::Vector3d &accel_bias,
                           const Eigen::Vector3d &gyro_bias) {
  imu_.SetData(Eigen::Vector3d{0.0, 0.0, -kGravity} + accel_bias,
               Eigen::Vector3d{0.0, 0.0, 0.0} + gyro_bias);
}

void SensorSim::SimulateOrientation(const Eigen::Quaterniond &orientation) {
  // TODO(lennartalff): Directly apply rotation without creating the rotation matrix
  const Eigen::Vector3d world_gravity = {0.0, 0.0, kGravity};
  const Eigen::Matrix3d R_to_earth = orientation.toRotationMatrix();
  const Eigen::Vector3d body_gravity = R_to_earth.transpose() * world_gravity;
  imu_.SetData(body_gravity, Eigen::Vector3d::Zero());
}
