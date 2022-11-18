#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <hippo_common/convert.hpp>
#include <hippo_common/tf2_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class MeshPublisher : public rclcpp::Node {
 public:
  MeshPublisher() : Node("mesh_publisher") {
    mesh_url_ = declare_parameter<std::string>(
        "mesh_url", "package://hippo_sim/models/pool/meshes/pool.dae");
    marker_frame_ = declare_parameter<std::string>("marker_frame", "pool_xy_center");

    topic_ = declare_parameter<std::string>("topic", "marker_topic");
    if (declare_parameter<bool>("static_tf", true)) {
      RCLCPP_INFO(get_logger(), "Publishing TF.");
      broadcaster_ =
          std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
      parent_frame_ = declare_parameter<std::string>("parent_frame", "map");
      auto x = declare_parameter<double>("x", 1.0);
      auto y = declare_parameter<double>("y", 2.0);
      auto z = declare_parameter<double>("z", -1.5);
      auto R = declare_parameter<double>("R", 0.0);
      auto P = declare_parameter<double>("P", 0.0);
      auto Y = declare_parameter<double>("Y", 0.0);

      Eigen::Quaterniond q =
          hippo_common::tf2_utils::EulerToQuaternion(R, P, Y);

      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now();
      t.header.frame_id = parent_frame_;
      t.child_frame_id = marker_frame_;
      t.transform.translation.x = x;
      t.transform.translation.y = y;
      t.transform.translation.z = z;
      hippo_common::convert::EigenToRos(q, t.transform.rotation);
      broadcaster_->sendTransform(t);
    }

    rclcpp::SystemDefaultsQoS qos;
    publisher_ = create_publisher<visualization_msgs::msg::Marker>(topic_, qos);
    timer_ =
        rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(1000),
                             std::bind(&MeshPublisher::PublishMarker, this));
  }
  void PublishMarker() {
    visualization_msgs::msg::Marker marker;
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
  auto node = std::make_shared<MeshPublisher>();
  rclcpp::spin(node);
  return 0;
}
