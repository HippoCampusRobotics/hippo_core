#include "teensy_commander.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>

#include <algorithm>

namespace esc {
namespace teensy {
TeensyCommander::TeensyCommander(rclcpp::NodeOptions const &_options)
    : Node("teensy_commander", _options) {
  DeclareParams();
  initialized_ = InitSerial(params_.serial_port);
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

void TeensyCommander::OnActuatorControls(
    hippo_msgs::msg::ActuatorControls::ConstSharedPtr _msg) {
  if (timed_out_) {
    timed_out_ = false;
    RCLCPP_INFO(get_logger(), "Topic '%s' received. Not timed out anymore.",
                actuator_controls_sub_->get_topic_name());
  }
  if (!armed_) {
    SetThrottle(0.0);
    return;
  }
  for (size_t i = 0; i < _msg->control.size(); ++i) {
  }
}

void TeensyCommander::SetThrottle(std::array<double, 8> _values) {
  ActuatorControlsMessage msg;
  Packet packet;
  for (size_t i = 0; i < _values.size(); ++i) {
    uint16_t pwm =
        (uint16_t)1500 + (uint16_t)(std::clamp(_values[i], -1.0, 1.0) * 500);
    msg.payload_.pwm[i] = pwm;
  }
  msg.SerializePayload(packet.MutablePayloadStart(), msg.MSG_SIZE);
  packet.SetPayloadSize(msg.MSG_SIZE);
  packet.Packetize();
  int bytes_written = write(serial_port_, packet.Data(), packet.Size());
  if (bytes_written != packet.Size()) {
    RCLCPP_ERROR(get_logger(),
                 "Failed to write data to serial port. Written %d of %d bytes.",
                 bytes_written, packet.Size());
  }
}

void TeensyCommander::SetThrottle(double _value) {
  ActuatorControlsMessage msg;
  Packet packet;
  uint16_t pwm =
      (uint16_t)1500 + (uint16_t)(std::clamp(_value, -1.0, 1.0) * 500);
  for (size_t i = 0; i < ARRAY_LENGTH(msg.payload_.pwm); ++i) {
    msg.payload_.pwm[i] = pwm;
  }
  msg.SerializePayload(packet.MutablePayloadStart(), msg.MSG_SIZE);
  packet.SetPayloadSize(msg.MSG_SIZE);
  packet.Packetize();
  int bytes_written = write(serial_port_, packet.Data(), packet.Size());
  if (bytes_written != packet.Size()) {
    RCLCPP_ERROR(get_logger(),
                 "Failed to write data to serial port. Written %d of %d bytes.",
                 bytes_written, packet.Size());
  }
}

bool TeensyCommander::InitSerial(std::string _port_name) {
  serial_port_ = open(_port_name.c_str(), O_RDWR);
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
}  // namespace teensy
}  // namespace esc

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(esc::teensy::TeensyCommander)
