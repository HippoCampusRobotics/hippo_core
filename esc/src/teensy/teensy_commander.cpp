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

#include "teensy_commander.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>

#include <algorithm>
#include <thread>

namespace esc {
namespace teensy {
TeensyCommander::TeensyCommander(rclcpp::NodeOptions const &_options)
    : Node("teensy_commander", _options) {
  DeclareParams();
  serial_initialized_ = InitSerial(params_.serial_port);
  packet_.Reset();
  InitPublishers();
  InitTimers();
  InitServices();
  InitSubscribers();
}

void TeensyCommander::InitTimers() {
  timers_.control_timeout = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(kThrottleInputTimeoutMs),
      [this]() { OnThrottleInputTimeout(); });

  timers_.read_serial = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(kReadSerialIntervalMs),
      [this]() { OnReadSerialTimer(); });

  timers_.publish_arming_state = rclcpp::create_timer(
      this, get_clock(),
      std::chrono::milliseconds(kPublishArmingStateIntervalMs),
      [this]() { OnPublishArmingStateTimer(); });

  timers_.publish_battery_voltage = rclcpp::create_timer(
      this, get_clock(),
      std::chrono::milliseconds(kPublishBatteryVoltageIntervalMs),
      [this]() { OnPublishBatteryVoltageTimer(); });
}

void TeensyCommander::InitServices() {
  std::string name;

  name = "arm";
  arming_service_ = create_service<std_srvs::srv::SetBool>(
      name, std::bind(&TeensyCommander::ServeArming, this,
                      std::placeholders::_1, std::placeholders::_2));
}

void TeensyCommander::ServeArming(
    const std_srvs::srv::SetBool_Request::SharedPtr _request,
    std_srvs::srv::SetBool_Response::SharedPtr _response) {
  if (_request->data) {
    if (armed_) {
      _response->message = "Already armed.";
      _response->success = false;
      return;
    } else {
      armed_ = _request->data;
      RCLCPP_INFO(get_logger(), "Arming the thrusters.");
      _response->message = "Armed";
      _response->success = false;
      SetThrottle(0.0);
    }
  } else {
    if (armed_) {
      RCLCPP_INFO(get_logger(), "Disarming the thrusters.");
      armed_ = false;
      SetThrottle(0.0);
      _response->message = "Disarmed.";
      _response->success = true;
    } else {
      _response->message = "Already disarmed.";
      _response->success = false;
    }
  }
}

void TeensyCommander::InitPublishers() {
  std::string topic;

  topic = "arming_state";
  arming_state_pub_ =
      create_publisher<std_msgs::msg::Bool>(topic, rclcpp::SystemDefaultsQoS());

  topic = "battery_voltage";
  battery_voltage_pub_ = create_publisher<std_msgs::msg::Float64>(
      topic, rclcpp::SystemDefaultsQoS());

  topic = "thruster_values";
  actuator_controls_pub_ =
      create_publisher<hippo_control_msgs::msg::ActuatorControls>(
          topic, rclcpp::SystemDefaultsQoS());
  topic = "debug_pwm_output";
  pwm_output_debug_pub_ =
      create_publisher<hippo_control_msgs::msg::ActuatorControls>(
          topic, rclcpp::SystemDefaultsQoS());
}

void TeensyCommander::InitSubscribers() {
  std::string topic;

  topic = "thruster_command";
  actuator_controls_sub_ =
      create_subscription<hippo_control_msgs::msg::ActuatorControls>(
          topic, rclcpp::SensorDataQoS(),
          std::bind(&TeensyCommander::OnActuatorControls, this,
                    std::placeholders::_1));
}

/**
 * @brief Stops the thrusters, sets the timeout state and cancels the timer.
 *
 */
void TeensyCommander::OnThrottleInputTimeout() {
  RCLCPP_WARN(get_logger(), "'%s' controls timed out.",
              actuator_controls_sub_->get_topic_name());
  SetThrottle(0.0);
  timed_out_ = true;
  timers_.control_timeout->cancel();
}

void TeensyCommander::OnReadSerialTimer() { ReadSerial(); }

void TeensyCommander::OnPublishArmingStateTimer() { PublishArmingState(); }

void TeensyCommander::OnPublishBatteryVoltageTimer() {
  PublishBatteryVoltage();
}

void TeensyCommander::OnActuatorControls(
    hippo_control_msgs::msg::ActuatorControls::ConstSharedPtr _msg) {
  // reset/restart the timeout timmer since we got a message obviously.
  timers_.control_timeout->reset();
  if (timed_out_) {
    timed_out_ = false;
    RCLCPP_INFO(get_logger(), "Topic '%s' received. Not timed out anymore.",
                actuator_controls_sub_->get_topic_name());
  }
  if (!armed_) {
    SetThrottle(0.0);
    return;
  }
  SetThrottle(_msg->control);
  // always read serial port input after sending something
  ReadSerial();
}

uint16_t TeensyCommander::InputToPWM(double input) {
  if (std::abs(input) < zero_rpm_threshold_) {
    return uint16_t(1500);
  }
  double pwm = 0;
  for (int i = 0; i < n_coeffs; i++) {
    if (input >= 0) {
      pwm +=
          interpolate(battery_voltage_mapping_, mapping_coeffs_.lower.voltage,
                      mapping_coeffs_.upper.voltage,
                      mapping_coeffs_.lower.forward[i],
                      mapping_coeffs_.upper.forward[i]) *
          std::pow(input, double(n_coeffs - 1 - i));
    } else {
      pwm +=
          interpolate(battery_voltage_mapping_, mapping_coeffs_.lower.voltage,
                      mapping_coeffs_.upper.voltage,
                      mapping_coeffs_.lower.backward[i],
                      mapping_coeffs_.upper.backward[i]) *
          std::pow(input, double(n_coeffs - 1 - i));
    }
  }
  return std::clamp(uint16_t(pwm), uint16_t(1000), uint16_t(2000));
}

double TeensyCommander::PWMToInput(uint16_t pwm) {
  const double upper_limit_deadzone = interpolate(
      battery_voltage_mapping_, mapping_coeffs_.lower.voltage,
      mapping_coeffs_.upper.voltage, mapping_coeffs_.lower.forward.back(),
      mapping_coeffs_.upper.forward.back());
  const double lower_limit_deadzone = interpolate(
      battery_voltage_mapping_, mapping_coeffs_.lower.voltage,
      mapping_coeffs_.upper.voltage, mapping_coeffs_.lower.backward.back(),
      mapping_coeffs_.upper.backward.back());
  if (double(pwm) > lower_limit_deadzone &&
      double(pwm) < upper_limit_deadzone) {
    return 0.0;
  } else if (pwm > 1500) {
    const double quadratic_coeff = interpolate(
        battery_voltage_mapping_, mapping_coeffs_.lower.voltage,
        mapping_coeffs_.upper.voltage, mapping_coeffs_.lower.forward[0],
        mapping_coeffs_.upper.forward[0]);
    const double linear_coeff = interpolate(
        battery_voltage_mapping_, mapping_coeffs_.lower.voltage,
        mapping_coeffs_.upper.voltage, mapping_coeffs_.lower.forward[1],
        mapping_coeffs_.upper.forward[1]);
    const double constant_coeff = interpolate(
        battery_voltage_mapping_, mapping_coeffs_.lower.voltage,
        mapping_coeffs_.upper.voltage, mapping_coeffs_.lower.forward[2],
        mapping_coeffs_.upper.forward[2]);
    return inverseSecondOrderPolynomial(double(pwm), quadratic_coeff,
                                        linear_coeff, constant_coeff);
  } else {
    const double quadratic_coeff = interpolate(
        battery_voltage_mapping_, mapping_coeffs_.lower.voltage,
        mapping_coeffs_.upper.voltage, mapping_coeffs_.lower.backward[0],
        mapping_coeffs_.upper.backward[0]);
    const double linear_coeff = interpolate(
        battery_voltage_mapping_, mapping_coeffs_.lower.voltage,
        mapping_coeffs_.upper.voltage, mapping_coeffs_.lower.backward[1],
        mapping_coeffs_.upper.backward[1]);
    const double constant_coeff = interpolate(
        battery_voltage_mapping_, mapping_coeffs_.lower.voltage,
        mapping_coeffs_.upper.voltage, mapping_coeffs_.lower.backward[2],
        mapping_coeffs_.upper.backward[2]);
    return inverseSecondOrderPolynomial(double(pwm), quadratic_coeff,
                                        linear_coeff, constant_coeff);
  }
}

void TeensyCommander::SetThrottle(const std::array<double, 8> &_values) {
  if (!serial_initialized_) {
    serial_initialized_ = InitSerial(params_.serial_port);
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Trying to set throttle but serial port not initialized.");
    return;
  }
  esc_serial::ActuatorControlsMessage msg;
  esc_serial::Packet packet;
  for (size_t i = 0; i < _values.size(); ++i) {
    uint16_t pwm = InputToPWM(std::clamp(_values[i], -1.0, 1.0));
    msg.payload_.pwm[i] = pwm;
  }
  PublishPWMValues(msg);
  size_t size =
      msg.Serialize(packet.MutablePayloadStart(), packet.PayloadCapacity());
  if (!size) {
    RCLCPP_ERROR(get_logger(),
                 "Could not serialize message. Serialized %lu of %lu bytes.",
                 size, (size_t)msg.MSG_SIZE + msg.Header().HEADER_SIZE);
    return;
  }
  packet.SetPayloadSize(size);
  packet.Packetize();
  int bytes_written = write(serial_port_, packet.Data(), packet.Size());
  if ((size_t)bytes_written != packet.Size()) {
    RCLCPP_ERROR(
        get_logger(),
        "Failed to write data to serial port. Written %d of %lu bytes.",
        bytes_written, packet.Size());
  }
}

void TeensyCommander::SetThrottle(double _value) {
  std::array<double, 8> values;
  for (size_t i = 0; i < values.size(); ++i) {
    values[i] = _value;
  }
  SetThrottle(values);
}

void TeensyCommander::PublishArmingState() {
  if (!arming_state_pub_) {
    RCLCPP_WARN(get_logger(), "Arming State Publisher not available.");
    return;
  }

  std_msgs::msg::Bool msg;
  msg.data = armed_;
  arming_state_pub_->publish(msg);
}

void TeensyCommander::PublishBatteryVoltage() {
  if (!battery_voltage_pub_) {
    RCLCPP_WARN(get_logger(), "Battery Voltage Publisher not available.");
    return;
  }

  std_msgs::msg::Float64 msg;
  msg.data = battery_voltage_;
  battery_voltage_pub_->publish(msg);
}

void TeensyCommander::PublishThrusterValues(std::array<double, 8> &_values) {
  if (!actuator_controls_pub_) {
    RCLCPP_WARN(get_logger(), "Thruster Values Publisher not available.");
    return;
  }
  hippo_control_msgs::msg::ActuatorControls msg;
  msg.control = _values;
  msg.header.stamp = now();
  actuator_controls_pub_->publish(msg);
}

void TeensyCommander::PublishPWMValues(
    esc_serial::ActuatorControlsMessage &_msg) {
  if (!pwm_output_debug_pub_) {
    RCLCPP_WARN(get_logger(), "PWM Values Publisher not available.");
    return;
  }
  hippo_control_msgs::msg::ActuatorControls msg;
  for (int i = 0; i < 8; i++) {
    msg.control[i] = double(_msg.payload_.pwm[i]);
  }
  msg.header.stamp = now();
  pwm_output_debug_pub_->publish(msg);
}

bool TeensyCommander::InitSerial(std::string _port_name) {
  serial_port_ = open(_port_name.c_str(), O_RDWR, O_NOCTTY);
  if (serial_port_ < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to open serial port '%s'. Exit code: %d",
                 _port_name.c_str(), serial_port_);
    return false;
  }
  tty_.c_cflag &= ~CRTSCTS;  // disable hardware flow control
  tty_.c_cflag |= CREAD | CLOCAL | CS8;
  tty_.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
  // Prevent conversion of newline to carriage return/line feed
  tty_.c_oflag &= ~ONLCR;

  // this combination of TIME and MIN makes a read call blocking until the
  // lesser of MIN and the requested bytes are availabe.
  tty_.c_cc[VTIME] = 0;
  tty_.c_cc[VMIN] = 0;
  cfmakeraw(&tty_);
  // baud rate
  cfsetispeed(&tty_, B115200);
  cfsetospeed(&tty_, B115200);

  if (tcsetattr(serial_port_, TCSANOW, &tty_) != 0) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not apply serial port settings!");
    return false;
  }
  return true;
}

void TeensyCommander::HandleActuatorControlsMessage(
    esc_serial::ActuatorControlsMessage &_msg) {
  std::array<double, 8> values;
  for (int i = 0; i < 8; ++i) {
    values[i] = PWMToInput(_msg.payload_.pwm[i]);
  }
  PublishThrusterValues(values);
}

void TeensyCommander::HandleBatteryVoltageMessage(
    esc_serial::BatteryVoltageMessage &_msg) {
  battery_voltage_ = _msg.payload_.voltage_mv / 1000.0;
  battery_voltage_mapping_ =
      std::min(std::max(battery_voltage_, mapping_coeffs_.lower.voltage),
               mapping_coeffs_.upper.voltage);
}

void TeensyCommander::ReadSerial() {
  int available_bytes = 0;
  if (ioctl(serial_port_, FIONREAD, &available_bytes) < 0) {
    return;
  }
  for (int i = 0; i < available_bytes; ++i) {
    uint8_t byte;
    int length = read(serial_port_, &byte, 1);
    if (length > 0) {
      if (!packet_.AddByte(byte)) {
        packet_.Reset();
        RCLCPP_WARN(get_logger(),
                    "Packet buffer full before packet was complete.");
        continue;
      }

      if (packet_.CompletelyReceived()) {
        esc_serial::msg_id_t msg_id = packet_.ParseMessage();
        RCLCPP_INFO(get_logger(), "Received packet with id: %u", msg_id);
        switch (msg_id) {
          case esc_serial::ActuatorControlsMessage::MSG_ID: {
            RCLCPP_DEBUG(get_logger(), "Received ActuatControlsMessage.");
            esc_serial::ActuatorControlsMessage msg;
            msg.Deserialize(packet_.PayloadStart(), packet_.PayloadSize());
            HandleActuatorControlsMessage(msg);
          } break;
          case esc_serial::BatteryVoltageMessage::MSG_ID: {
            RCLCPP_DEBUG(get_logger(), "Received BatteryVoltageMessage.");
            esc_serial::BatteryVoltageMessage msg;
            msg.Deserialize(packet_.PayloadStart(), packet_.PayloadSize());
            HandleBatteryVoltageMessage(msg);
          } break;
          default:
            RCLCPP_WARN(get_logger(),
                        "Receiving unhandled message with id: %hu", msg_id);
            break;
        }
        // TODO handle packet.
        packet_.Reset();
      }
    }
  }
}
}  // namespace teensy
}  // namespace esc

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(esc::teensy::TeensyCommander)
