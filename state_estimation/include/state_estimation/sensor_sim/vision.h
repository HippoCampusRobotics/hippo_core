#pragma once

#include <state_estimation/sensor_sim/sensor.h>
namespace sensor_sim {
namespace sensor {
class Vision : public Sensor {
 public:
  Vision(std::shared_ptr<Ekf> ekf);
  ~Vision();
  void SetData(const VisionSample &vision_sample);
  void SetPosition(const Eigen::Vector3d &p);
  void SetPositionCovariance(const Eigen::Vector3d &covariance);
  void SetOrientation(const Eigen::Quaterniond &P);
  void SetAngularVariance(double var);
  VisionSample DataAtRest();

  private:
  VisionSample vision_data_;
  void Send(uint64_t time_us) override;
};
}  // namespace sensor
}  // namespace sensor_sim
