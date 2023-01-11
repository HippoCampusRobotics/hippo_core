#pragma once
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <rapid_trajectories/trajectory/generator.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace rapid_trajectories {
class RvizHelper {
 public:
  RvizHelper(rclcpp::Node::SharedPtr _node);
  void PublishTrajectory(
      const minimum_jerk::Trajectory &_trajectory);

  void PublishTarget(const Eigen::Vector3d &_point);

  void PublishStart(const Eigen::Vector3d &_point);

  void PublishHeading(const Eigen::Vector3d &_position,
                      const Eigen::Quaterniond &_orientation);
  void PublishHeading(const Eigen::Vector3d &_position,
                      const Eigen::Vector3d &_axis);

 private:
  void InitTargetMarker();
  void InitStartMarker();
  void InitThrustMarker();
  void InitHeadingMarker();
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr heading_pub_;
  visualization_msgs::msg::Marker target_marker_;
  visualization_msgs::msg::Marker start_marker_;
  visualization_msgs::msg::Marker thrust_marker_;
  visualization_msgs::msg::Marker heading_marker_;
  int n_trajectory_samples_{50};
};
}  // namespace rapid_trajectories
