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

#include <termios.h>

#include <esc_serial.hpp>
#include <hippo_msgs/msg/actuator_controls.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace esc {
namespace teensy {

constexpr int n_coeffs = 3;

inline double interpolate(const double &x, const double &x_low,
                          const double &x_up, const double &y_low,
                          const double &y_up) {
  return (y_up - y_low) / (x_up - x_low) * (x - x_low) + y_low;
}

inline double inverseSecondOrderPolynomial(const double &input,
                                           const double &quadratic_coeff,
                                           const double &linear_coeff,
                                           const double &constant_coeff) {
  return (-1.0 * linear_coeff +
          sqrt(4.0 * quadratic_coeff * input + linear_coeff * linear_coeff -
               4.0 * quadratic_coeff * constant_coeff)) /
         (2.0 * quadratic_coeff);
}

class TeensyCommander : public rclcpp::Node {
 public:
  static constexpr int kThrottleInputTimeoutMs = 500;
  static constexpr int kReadSerialIntervalMs = 100;
  static constexpr int kPublishArmingStateIntervalMs = 500;
  static constexpr int kPublishBatteryVoltageIntervalMs = 500;
  explicit TeensyCommander(rclcpp::NodeOptions const &_options);

 private:
  struct Params {
    std::string serial_port{"/dev/teensy_data"};
  };
  void DeclareParams();
  bool InitSerial(std::string _port_name);
  void InitPublishers();
  void InitTimers();
  void InitSubscribers();
  void InitServices();
  void SetThrottle(const std::array<double, 8> &_values);
  void SetThrottle(double _value);
  uint16_t InputToPWM(double input);
  double PWMToInput(uint16_t pwm);
  void PublishArmingState();
  void PublishBatteryVoltage();
  void PublishThrusterValues(std::array<double, 8> &_values);
  void PublishPWMValues(esc_serial::ActuatorControlsMessage &_msg);

  void HandleActuatorControlsMessage(esc_serial::ActuatorControlsMessage &_msg);
  void HandleBatteryVoltageMessage(esc_serial::BatteryVoltageMessage &_msg);

  void ReadSerial();

  //////////////////////////////////////////////////////////////////////////////
  // Publishers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arming_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_voltage_pub_;
  rclcpp::Publisher<hippo_msgs::msg::ActuatorControls>::SharedPtr
      actuator_controls_pub_;
  rclcpp::Publisher<hippo_msgs::msg::ActuatorControls>::SharedPtr
      pwm_output_debug_pub_;
  //////////////////////////////////////////////////////////////////////////////
  // Subscribers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Subscription<hippo_msgs::msg::ActuatorControls>::SharedPtr
      actuator_controls_sub_;
  void OnActuatorControls(hippo_msgs::msg::ActuatorControls::ConstSharedPtr);

  //////////////////////////////////////////////////////////////////////////////
  // Services
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arming_service_;

  //////////////////////////////////////////////////////////////////////////////
  // Callbacks
  //////////////////////////////////////////////////////////////////////////////
  void OnThrottleInputTimeout();
  void OnReadSerialTimer();
  void OnPublishArmingStateTimer();
  void OnPublishBatteryVoltageTimer();

  void ServeArming(const std_srvs::srv::SetBool_Request::SharedPtr _request,
                   std_srvs::srv::SetBool_Response::SharedPtr _response);

  //////////////////////////////////////////////////////////////////////////////
  // Timers
  //////////////////////////////////////////////////////////////////////////////
  struct Timers {
    rclcpp::TimerBase::SharedPtr control_timeout;
    rclcpp::TimerBase::SharedPtr publish_arming_state;
    rclcpp::TimerBase::SharedPtr publish_battery_voltage;
    rclcpp::TimerBase::SharedPtr read_serial;
  };

  Timers timers_;

  Params params_;
  int serial_port_;
  struct termios tty_;
  bool serial_initialized_{false};
  bool timed_out_{true};
  bool armed_{false};
  double battery_voltage_{0.0};
  double battery_voltage_mapping_{
      15.0};  //!< battery voltage used for rpm->pwm mapping, makes sure that
              //!< battery voltage used for calculation is clamped at edges of
              //!< expected range
  double zero_rpm_threshold_{0.0001};

  struct Coefficients {
    std::array<double, n_coeffs>
        forward;  //!< polynomial coefficients for forward turning direction,
                  //!< from high polynomial degree to low
    std::array<double, n_coeffs>
        backward;  //!< polynomial coefficients for backward turning direction,
                   //!< from high polynomial degree to low
    double voltage;
  };

  struct MappingCoefficients {
    Coefficients upper;
    Coefficients lower;
  };

  MappingCoefficients mapping_coeffs_;

  esc_serial::Packet packet_;
};
}  // namespace teensy
}  // namespace esc
