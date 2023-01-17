#include "bridge.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp_components/register_node_macro.hpp>

#include "qualisys_bridge/qualisys/RTProtocol.h"
RCLCPP_COMPONENTS_REGISTER_NODE(qualisys_bridge::Bridge)

namespace qualisys_bridge {
Bridge::Bridge(rclcpp::NodeOptions const &_options)
    : Node("apriltag_simple", _options) {
  odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::SystemDefaultsQoS());

  update_timer_ =
      rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(1),
                           std::bind(&Bridge::OnUpdate, this));
}

void Bridge::OnUpdate() {
  if (!rt_protocol_.Connected()) {
    RCLCPP_INFO(get_logger(), "Trying to connect.");
    if (!Connect()) {
      return;
    }
    RCLCPP_INFO(get_logger(), "Connected.");
  }
  if (!data_available_) {
    if (!rt_protocol_.Read6DOFSettings(data_available_)) {
      RCLCPP_WARN(get_logger(), "rtProtocol.StreamFrames: %s\n\n",
                  rt_protocol_.GetErrorString());
      return;
    }
  }

  if (!stream_frames_) {
    if (!rt_protocol_.StreamFrames(CRTProtocol::RateAllFrames, 0, udp_port_,
                                   NULL, CRTProtocol::cComponent6d)) {
      RCLCPP_WARN(get_logger(), "rtProtocol.StreamFrames: %s\n\n",
                  rt_protocol_.GetErrorString());
      return;
    }
    stream_frames_ = true;
    RCLCPP_INFO(get_logger(), "Start streaming.");
  }

  CRTPacket::EPacketType packet_type;
  if (rt_protocol_.Receive(packet_type, true) ==
      CNetwork::ResponseType::success) {
    if (packet_type == CRTPacket::PacketData) {
      float x, y, z;
      float R[9];
      CRTPacket *packet = rt_protocol_.GetRTPacket();
      bool found{false};
      for (unsigned int i = 0; i < packet->Get6DOFBodyCount(); ++i) {
        if (packet->Get6DOFBody(i, x, y, z, R)) {
          const char *c_name = rt_protocol_.Get6DOFBodyName(i);
          std::string name{c_name};
          // if (name == "hippo") {
          if (true) {
            nav_msgs::msg::Odometry msg;
            msg.header.stamp = now();
            msg.header.frame_id = "map";
            msg.child_frame_id = "base_link";
            msg.pose.pose.position.x = x;
            msg.pose.pose.position.y = y;
            msg.pose.pose.position.z = z;
            tf2::Matrix3x3 R_mat(R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7],
                                 R[8]);
            tf2::Quaternion q;
            R_mat.getRotation(q);
            msg.pose.pose.orientation.w = q.w();
            msg.pose.pose.orientation.x = q.x();
            msg.pose.pose.orientation.y = q.y();
            msg.pose.pose.orientation.z = q.z();
            odometry_pub_->publish(msg);
          }
        }
      }
    }
  }
}

bool Bridge::Connect() {
  if (!rt_protocol_.Connect(server_address_.c_str(), base_port_, &udp_port_,
                            major_version_, minor_version_, big_endian_)) {
    RCLCPP_ERROR(get_logger(), "Failed to connect!");
    return false;
  }
  return true;
}

}  // namespace qualisys_bridge
