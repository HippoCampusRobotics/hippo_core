#pragma once
#include "hardware/barometer/ms5837.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

namespace hardware {
namespace barometer {
class Barometer : public rclcpp::Node {
public:
  explicit Barometer(rclcpp::NodeOptions const &_options);

private:
  struct Params {
    std::string device;
    int update_period_ms;
  };
  void InitPublishers();
  void InitTimers();
  void InitParams();
  void OnReadTimer();

  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;

  rclcpp::TimerBase::SharedPtr read_timer_;

  MS5837 barometer_;
  bool barometer_initialized_{false};
  Params params_;
};

} // namespace barometer
} // namespace hardware
