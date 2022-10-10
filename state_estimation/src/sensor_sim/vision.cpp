#include <state_estimation/sensor_sim/vision.h>
namespace sensor_sim {
namespace sensor {
Vision::Vision(std::shared_ptr<Ekf> ekf) : Sensor(ekf) {}
Vision::~Vision(){}

void Vision::Send(uint64_t time_us) {
  vision_data_.time_us = time_us;
  ekf_->SetVisionData(vision_data_);
}

void Vision::SetData(const VisionSample &vision_sample) {
  vision_data_ = vision_sample;
}

void Vision::SetPosition(const Eigen::Vector3d &p) {
  vision_data_.position = p;
}

void Vision::SetPositionCovariance(const Eigen::Vector3d &covariance) {
  vision_data_.position_variance = covariance;
}

void Vision::SetOrientation(const Eigen::Quaterniond &orientation) {
  vision_data_.orientation = orientation;
}

void Vision::SetAngularVariance(double var) {
  vision_data_.angular_variance = var;
}

VisionSample Vision::DataAtRest() {
  VisionSample sample;
  sample.position = Eigen::Vector3d::Zero();
  sample.velocity = Eigen::Vector3d::Zero();
  sample.orientation = Eigen::Quaterniond{0.9689124, 0.0, 0.0, 0.247404}.normalized();
  sample.position_variance = Eigen::Vector3d{0.1, 0.1, 0.1};
  sample.angular_variance = 0.05;
  return sample;
}

}  // namespace sensor
}  // namespace sensor_sim
