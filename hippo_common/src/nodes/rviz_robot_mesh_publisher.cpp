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

#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <hippo_common/convert.hpp>
#include <hippo_common/tf2_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class RobotMeshPublisher : public rclcpp::Node {
 public:
  RobotMeshPublisher() : Node("robot_mesh_publisher") {
    mesh_url_ = declare_parameter<std::string>(
        "mesh_url", "package://hippo_sim/models/bluerov/meshes/bluerov.dae");
    marker_frame_ = declare_parameter<std::string>(
        "marker_frame", hippo_common::tf2_utils::frame_id::BaseLink(this));

    topic_ = declare_parameter<std::string>("topic", "robot_mesh_marker");

    rclcpp::SystemDefaultsQoS qos;
    publisher_ = create_publisher<visualization_msgs::msg::Marker>(topic_, qos);
    timer_ = rclcpp::create_timer(
        this, get_clock(), std::chrono::milliseconds(20),
        std::bind(&RobotMeshPublisher::PublishMarker, this));
  }
  void PublishMarker() {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = marker_frame_;
    marker.mesh_resource = mesh_url_;
    marker.type = marker.MESH_RESOURCE;
    marker.color.a = 1.0;
    marker.color.b = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.pose.orientation.w = 1.0;
    publisher_->publish(marker);
  }

 private:
  std::string mesh_url_;
  std::string topic_;
  std::string parent_frame_;
  std::string marker_frame_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotMeshPublisher>();
  rclcpp::spin(node);
  return 0;
}
