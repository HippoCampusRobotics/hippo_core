#pragma once
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <qualisys_bridge/qualisys/RTProtocol.h>

namespace qualisys_bridge {
class Bridge : public rclcpp::Node {
 public:
  explicit Bridge(rclcpp::NodeOptions const &_options);

 private:
  void OnUpdate();
  bool Connect();
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  CRTProtocol rt_protocol_;

  std::string server_address_{"192.168.0.161"};
  unsigned short base_port_{22222};
  int major_version_{1};
  int minor_version_{19};
  bool big_endian_{false};
  unsigned short udp_port_{6734};
  bool data_available_{false};
  bool stream_frames_{false};
};
}  // namespace qualisys_bridge
