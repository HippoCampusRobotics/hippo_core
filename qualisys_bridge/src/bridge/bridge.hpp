#pragma once
#include <qualisys_bridge/qualisys/RTProtocol.h>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "qualisys_bridge/ekf.hpp"

namespace qualisys_bridge {
class Bridge : public rclcpp::Node {
 public:
  explicit Bridge(rclcpp::NodeOptions const &_options);

 private:
  struct Params {
    std::string name{"uuv00"};
  } params_;
  void OnUpdate();
  bool Connect();
  void HandlePacket(CRTPacket *_packet);
  void PublishOdometry();
  void PublishAcceleration();
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr accel_pub_;
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
  Ekf ekf_;
  Ekf::Matrix15d process_noise_;
  Ekf::Matrix6d measurement_noise_;
  double t_start_frame_ros_{0.0};
  double t_start_frame_qtm_{0.0};
};
}  // namespace qualisys_bridge
