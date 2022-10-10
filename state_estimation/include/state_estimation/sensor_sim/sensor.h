#pragma once

#include <state_estimation/ekf.h>

#include <memory>

namespace sensor_sim {
class Sensor {
 public:
  Sensor(std::shared_ptr<Ekf> ekf);
  virtual ~Sensor();
  void Update(uint64_t time_us);
  void SetRate(double rate) { update_period_us_ = uint64_t(1.0 / rate * 1e6); }
  bool IsRunning() const { return is_running_; }
  void Start() { is_running_ = true; }
  void Stop() { is_running_ = false; }
  inline bool ShouldSend(uint64_t time_us) const {
    return is_running_ && IsTimeToSend(time_us);
  }

 protected:
  std::shared_ptr<Ekf> ekf_;
  uint64_t update_period_us_;
  uint64_t time_last_data_sent_us_{0};
  bool is_running_{false};
  bool IsTimeToSend(uint64_t time_us) const {
    return (time_us >= time_last_data_sent_us_) &&
           ((time_us - time_last_data_sent_us_) >= update_period_us_);
  }
  virtual void Send(uint64_t time_us) = 0;
};
}  // namespace sensor_sim
