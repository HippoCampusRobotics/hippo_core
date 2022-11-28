#pragma once
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hippo_common {
class TfPublisher : public rclcpp::Node {
 public:
  explicit TfPublisher(rclcpp::NodeOptions const &_options);

 private:
  void DeclareParameters();
  void DeclareVerticalCameraParameters();
  void BroadCastStatic();
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  struct CameraPose {
    double x{-0.1};
    double y{0.0};
    double z{0.0};
    double qw{0.7071068};
    double qx{0.0};
    double qy{0.7071068};
    double qz{0.0};
  };
  struct Params {
    struct CameraPose vertical_camera;
  } params_;
};
}  // namespace hippo_common
