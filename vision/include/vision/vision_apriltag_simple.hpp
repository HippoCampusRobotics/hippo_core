
#pragma once
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <apriltag_ros/msg/april_tag_detection_array.hpp>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace vision {

class AprilTagSimple : public rclcpp::Node {
 public:
  explicit AprilTagSimple(rclcpp::NodeOptions const &_options);

 private:
  struct Params {
    std::string bundle_frame{"map_bundle"};
  };
  void OnAprilTagDetections(
      apriltag_ros::msg::AprilTagDetectionArray::ConstSharedPtr _msg);
  void OnLocalPosition(
      px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr _msg);
  void OnAttitude(px4_msgs::msg::VehicleAttitude::ConstSharedPtr _msg);
  void OnUpdateOdometry();

  Params params_;

  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr
      visual_odometry_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      vision_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  rclcpp::Subscription<apriltag_ros::msg::AprilTagDetectionArray>::SharedPtr
      apriltag_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      px4_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr
      px4_attitude_sub_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  Eigen::Vector3d position_px4_;
  Eigen::Quaterniond orientation_px4_;
  Eigen::Vector3d velocity_px4_;
  bool px4_position_update_{false};
  bool px4_attitude_update_{false};
};
}  // namespace vision
