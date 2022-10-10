#include <state_estimation/ekf.h>
#include <memory>

#include <state_estimation/sensor_sim/imu.h>
#include <state_estimation/sensor_sim/baro.h>
#include <state_estimation/sensor_sim/vision.h>

using namespace sensor_sim::sensor;

class SensorSim {
 public:
  SensorSim(std::shared_ptr<Ekf> ekf);
  ~SensorSim();
  uint64_t TimeUs() const { return time_us_; }
  void RunSeconds(double duration);
  void RunMicros(uint64_t duration_us);
  void SetImuBias(const Eigen::Vector3d &accel_bias, const Eigen::Vector3d &gyro_bias);
  void SimulateOrientation(const Eigen::Quaterniond &orientation);
  Imu imu_;
  Baro baro_;
  Vision vision_;

  private:
  void SetSensorDataToDefault();
  void SetSensorRateToDefault();
  void StartBasicSensors();
  void UpdateSensors();
  std::shared_ptr<Ekf> ekf_{nullptr};
  uint64_t time_us_{0};
};
