#pragma once

#include <termios.h>

#include <esc_serial.hpp>
#include <hippo_msgs/msg/actuator_controls.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/bool.hpp>

namespace esc {
namespace teensy {
class TeensyCommander : public rclcpp::Node {
 public:
  explicit TeensyCommander(rclcpp::NodeOptions const &_options);

 private:
  struct Params {
    std::string serial_port{""};
  };
  void DeclareParams();
  bool InitSerial(std::string _port_name);
  void InitPublishers();
  void InitSubscribers();
  void SetThrottle(std::array<double, 8> _values);
  void SetThrottle(double _value);


  //////////////////////////////////////////////////////////////////////////////
  // Publishers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arming_state_pub_;
  //////////////////////////////////////////////////////////////////////////////
  // Subscribers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Subscription<hippo_msgs::msg::ActuatorControls>::SharedPtr
      actuator_controls_sub_;
  void OnActuatorControls(hippo_msgs::msg::ActuatorControls::ConstSharedPtr);

  //////////////////////////////////////////////////////////////////////////////
  // Services
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arming_serivce_;
  

  Params params_;
  int serial_port_;
  struct termios tty_;
  bool initialized_{false};
  bool timed_out_{true};
  bool armed_{false};
  double battery_voltage_{0.0};
};
}  // namespace teensy
}  // namespace esc
