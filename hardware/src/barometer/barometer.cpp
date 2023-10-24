// Copyright (C) 2023 Thies Lennart Alff
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
// USA

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
  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);

  topic = "pressure";
  pressure_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>(topic, qos);

  topic = "temperature";
  temperature_pub_ =
      create_publisher<sensor_msgs::msg::Temperature>(topic, qos);
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
  const rclcpp::Time t_now = now();
  const double pressure = barometer_.PressureCompensated();
  PublishPressure(t_now, pressure);
  const double temperature = barometer_.TemperatureCompensated();
  PublishTemperature(t_now, temperature);
}

void Barometer::PublishPressure(const rclcpp::Time &_now, double _pressure) {
  sensor_msgs::msg::FluidPressure msg;
  msg.header.stamp = _now;
  msg.fluid_pressure = _pressure;
  if (!pressure_pub_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Pressure publisher not created!");
    return;
  }
  pressure_pub_->publish(msg);
}

void Barometer::PublishTemperature(const rclcpp::Time &_now,
                                   double _temperature) {
  sensor_msgs::msg::Temperature msg;
  msg.header.stamp = _now;
  msg.temperature = _temperature;
  if (!temperature_pub_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Pressure publisher not created!");
    return;
  }
  temperature_pub_->publish(msg);
}

}  // namespace barometer
}  // namespace hardware

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hardware::barometer::Barometer)
