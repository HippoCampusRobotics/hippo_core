#include <state_estimation/sensor_sim/sensor.h>

namespace sensor_sim {
Sensor::Sensor(std::shared_ptr<Ekf> ekf) : ekf_{ekf} {}
Sensor::~Sensor() {}
void Sensor::Update(uint64_t time_us) {
  if (ShouldSend(time_us)) {
    Send(time_us);
    time_last_data_sent_us_ = time_us;
  }
}
}  // namespace sensor_sim
