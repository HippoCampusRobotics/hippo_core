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
  InitSubscribers();
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
  SetThrottle(_msg->control);
}

void TeensyCommander::SetThrottle(const std::array<double, 8> &_values) {
  ActuatorControlsMessage msg;
  Packet packet;
  for (size_t i = 0; i < _values.size(); ++i) {
    uint16_t pwm = (uint16_t)1500 + (std::clamp(_values[i], -1.0, 1.0) * 500);
    msg.payload_.pwm[i] = pwm;
  }
  size_t size =
      msg.Serialize(packet.MutablePayloadStart(), packet.PayloadCapacity());
  RCLCPP_WARN_STREAM(get_logger(), "Size:" << std::to_string(size));
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
        if (msg_id == ActuatorControlsMessage::MSG_ID) {
          ActuatorControlsMessage msg;
          msg.Deserialize(packet_.PayloadStart(), packet_.PayloadSize());
          for (int i = 0; i < 8; ++i) {
            printf("%u\n", msg.payload_.pwm[i]);
          }
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
