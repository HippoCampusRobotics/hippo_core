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
      std ::chrono::milliseconds(kPublishArmingStateIntervalMs),
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
  actuator_controls_pub_ = create_publisher<hippo_msgs::msg::ActuatorControls>(
      topic, rclcpp::SystemDefaultsQoS());
}

void TeensyCommander::InitSubscribers() {
  std::string topic;

  topic = "thruster_command";
  actuator_controls_sub_ =
      create_subscription<hippo_msgs::msg::ActuatorControls>(
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
    hippo_msgs::msg::ActuatorControls::ConstSharedPtr _msg) {
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

void TeensyCommander::SetThrottle(const std::array<double, 8> &_values) {
  if (!serial_initialized_) {
    serial_initialized_ = InitSerial(params_.serial_port);
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Trying to set throttle but serial port not initialized.");
    return;
  }
  ActuatorControlsMessage msg;
  Packet packet;
  for (size_t i = 0; i < _values.size(); ++i) {
    uint16_t pwm = (uint16_t)1500 + (std::clamp(_values[i], -1.0, 1.0) * 500);
    msg.payload_.pwm[i] = pwm;
  }
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
  if (bytes_written != packet.Size()) {
    RCLCPP_ERROR(get_logger(),
                 "Failed to write data to serial port. Written %d of %d bytes.",
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
  hippo_msgs::msg::ActuatorControls msg;
  msg.control = _values;
  msg.header.stamp = now();
  actuator_controls_pub_->publish(msg);
}

bool TeensyCommander::InitSerial(std::string _port_name) {
  serial_port_ = open(_port_name.c_str(), O_RDWR);
  if (serial_port_ < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to open serial port '%s'. Exit code: %d",
                 _port_name.c_str(), serial_port_);
    return false;
  }
  tty_.c_cflag &= ~CRTSCTS;  // disable hardware flow control
  tty_.c_cflag |= CREAD | CLOCAL;
  tty_.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
  // Prevent conversion of newline to carriage return/line feed
  tty_.c_oflag &= ~ONLCR;

  // this combination of TIME and MIN makes a read call blocking until the
  // lesser of MIN and the requested bytes are availabe.
  tty_.c_cc[VTIME] = 0;
  tty_.c_cc[VMIN] = 1;
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
    ActuatorControlsMessage &_msg) {
  std::array<double, 8> values;
  for (int i = 0; i < 8; ++i) {
    values[i] = (_msg.payload_.pwm[i] - 1500) / 500.0;
  }
  PublishThrusterValues(values);
}

void TeensyCommander::HandleBatteryVoltageMessage(BatteryVoltageMessage &_msg) {
  battery_voltage_ = _msg.payload_.voltage_mv * 1000.0;
}

void TeensyCommander::ReadSerial() {
  int available_bytes = 0;
  if (ioctl(serial_port_, FIONREAD, &available_bytes) < 0) {
    return;
  }
  for (int i = 0; i < available_bytes; ++i) {
    uint8_t byte;
    int length = read(serial_port_, &byte, 1);
    if (length) {
      if (!packet_.AddByte(byte)) {
        packet_.Reset();
        RCLCPP_WARN(get_logger(),
                    "Packet buffer full before packet was complete.");
        return;
      }

      if (packet_.CompletelyReceived()) {
        msg_id_t msg_id = packet_.ParseMessage();
        RCLCPP_INFO(get_logger(), "Received packet with id: %u", msg_id);
        switch (msg_id) {
          case ActuatorControlsMessage::MSG_ID: {
            ActuatorControlsMessage msg;
            msg.Deserialize(packet_.PayloadStart(), packet_.PayloadSize());
            HandleActuatorControlsMessage(msg);
          } break;
          case BatteryVoltageMessage::MSG_ID: {
            BatteryVoltageMessage msg;
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
