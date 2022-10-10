#pragma once
#include "sensor.h"

namespace sensor_sim {
namespace sensor {
class Baro : public Sensor {
 public:
  Baro(std::shared_ptr<Ekf> ekf) : Sensor(ekf){};
  ~Baro(){};
  inline void SetData(double height) { baro_data_ = height; }

 private:
  double baro_data_{0.0};
  void Send(uint64_t time_us) override {
    ekf_->SetBaroData(BaroSample{time_us, baro_data_});
  }
};
}  // namespace sensor
}  // namespace sensor_sim
