#include "barometer.hpp"

namespace hardware {
namespace barometer {
Barometer::Barometer(rclcpp::NodeOptions const &_options)
    : Node("barometer", _options) {
  InitParams();
  InitPublishers();
  barometer_.Open(params_.device);
  InitTimers();
}

void Barometer::InitPublishers() {
  std::string topic;

  topic = "pressure";
  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);
  pressure_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>(topic, qos);
}

void Barometer::InitTimers() {
  read_timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(params_.update_period_ms),
      [this]() { OnReadTimer(); });
}

void Barometer::OnReadTimer() {
  if (!barometer_initialized_) {
    MS5837::Status status = barometer_.Init();
    if (status != MS5837::Status::kOk) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Could not initialize barometer [%s]. Return Code: %d",
          params_.device.c_str(), static_cast<int>(status));
      return;
    }
    RCLCPP_INFO(get_logger(), "Initialized barometer [%s]",
                params_.device.c_str());
    barometer_initialized_ = true;
  }

  MS5837::Status status = barometer_.Read(MS5837::Oversampling::k2048);
  if (status != MS5837::Status::kOk) {
    RCLCPP_ERROR(get_logger(), "Failed to read sensor.");
    barometer_initialized_ = false;
    return;
  }

  sensor_msgs::msg::FluidPressure msg;
  msg.header.stamp = now();
  msg.fluid_pressure = barometer_.PressureCompensated();
  if (!pressure_pub_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Pressure publisher not created!");
    return;
  }
  pressure_pub_->publish(msg);
}

} // namespace barometer
} // namespace hardware

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hardware::barometer::Barometer)
