#pragma once
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hippo_common {
class TfPublisherVehicle : public rclcpp::Node {
 public:
  explicit TfPublisherVehicle(rclcpp::NodeOptions const &_options);

 private:
  struct Pose {
    double x{-0.1};
    double y{0.0};
    double z{0.0};
    double qw{0.7071068};
    double qx{0.0};
    double qy{0.7071068};
    double qz{0.0};
  };
  struct Params {
    bool has_vertical_camera{false};
    Pose vertical_camera;
    bool has_front_camera{false};
    Pose front_camera;
  };
  void DeclareParameters();
  void DeclareCameraParameters(std::string _camera_name, Pose &_pose_param);
  void DeclareVerticalCameraParameters();
  void DeclareFrontCameraParameters();
  void BroadCastStatic();

  void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr _msg);
  void OnVisionPoseCovariance(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr _msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_{nullptr};
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      vision_pose_cov_sub_{nullptr};

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  Params params_;
};
}  // namespace hippo_common
