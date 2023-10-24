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

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include "hardware/barometer/ms5837.hpp"

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
  void PublishPressure(const rclcpp::Time &_now, double _pressure);
  void PublishTemperature(const rclcpp::Time &_now, double _temperature);

  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;

  rclcpp::TimerBase::SharedPtr read_timer_;

  MS5837 barometer_;
  bool barometer_initialized_{false};
  Params params_;
};

}  // namespace barometer
}  // namespace hardware
