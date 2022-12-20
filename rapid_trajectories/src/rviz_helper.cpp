#include <eigen3/Eigen/Dense>
#include <rapid_trajectories/rviz_helper.hpp>
#include <hippo_common/convert.hpp>

namespace rapid_trajectories {
RvizHelper::RvizHelper(rclcpp::Node::SharedPtr _node) {
  node_ = _node;
  rclcpp::SystemDefaultsQoS qos;
  qos.keep_last(50);
  trajectory_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "trajectory", qos);
  target_pub_ =
      node_->create_publisher<visualization_msgs::msg::Marker>("target", qos);
  start_pub_ =
      node_->create_publisher<visualization_msgs::msg::Marker>("start", qos);
  heading_pub_ =
      node_->create_publisher<visualization_msgs::msg::Marker>("heading", qos);
  InitStartMarker();
  InitTargetMarker();
  InitThrustMarker();
  InitHeadingMarker();
}

void RvizHelper::PublishTrajectory(
    const minimum_jerk::Generator &_trajectory) {
  visualization_msgs::msg::Marker path;
  path.points.resize(n_trajectory_samples_);
  path.header.frame_id = "map";
  path.ns = "trajectory";
  path.action = visualization_msgs::msg::Marker::ADD;
  path.id = 0;
  path.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path.scale.x = 0.1;
  path.color.a = 1.0;
  path.color.r = 0.0;
  path.color.g = 1.0;
  path.color.b = 0.0;
  path.pose.orientation.w = 1.0;
  for (int i = 0; i < n_trajectory_samples_; ++i) {
    geometry_msgs::msg::Point &path_point = path.points.at(i);
    double t = _trajectory.GetFinalTime() / (n_trajectory_samples_ - 1) * i;
    Eigen::Vector3d p = _trajectory.GetPosition(t);
    hippo_common::convert::EigenToRos(p, path_point);
    geometry_msgs::msg::Point thrust_point;
    thrust_point = path_point;

    double thrust = _trajectory.GetThrust(t);
    thrust_marker_.points.at(2 * i) = thrust_point;
    thrust_point.z += thrust * 0.1;
    thrust_marker_.points.at(2 * i + 1) = thrust_point;
  }
  // trajectory_pub_->publish(path);
  trajectory_pub_->publish(thrust_marker_);
}

void RvizHelper::PublishTarget(const Eigen::Vector3d &_point) {
  geometry_msgs::msg::Point p;
  hippo_common::convert::EigenToRos(_point, p);
  target_marker_.header.stamp = node_->get_clock()->now();
  target_marker_.pose.position = p;
  target_pub_->publish(target_marker_);
}

void RvizHelper::PublishHeading(const Eigen::Vector3d &_position,
                                const Eigen::Quaterniond &_orientation) {
  heading_marker_.points.clear();

  heading_marker_.scale.x = 1.0; // length
  heading_marker_.scale.y = 0.05; // width
  heading_marker_.scale.z = 0.05; // height
  geometry_msgs::msg::Point p;
  hippo_common::convert::EigenToRos(_position, p);
  geometry_msgs::msg::Quaternion q;
  hippo_common::convert::EigenToRos(_orientation, q);
  heading_marker_.header.stamp = node_->now();
  heading_marker_.pose.position = p;
  heading_marker_.pose.orientation = q;
  heading_pub_->publish(heading_marker_);
}

void RvizHelper::PublishHeading(const Eigen::Vector3d &_position,
                                const Eigen::Vector3d &_axis) {
  heading_marker_.points.clear();
  heading_marker_.scale.x = 0.05; // diameter
  heading_marker_.scale.y = 0.1; // head diameter
  heading_marker_.scale.z = 0.1; // head length
  geometry_msgs::msg::Point p;
  hippo_common::convert::EigenToRos(_position, p);
  heading_marker_.points.push_back(p);
  Eigen::Vector3d tip = _position + _axis;
  hippo_common::convert::EigenToRos(tip, p);
  heading_marker_.points.push_back(p);
  heading_marker_.header.stamp = node_->now();
  heading_marker_.pose.orientation.w = 1;
  heading_marker_.pose.orientation.x = 0;
  heading_marker_.pose.orientation.y = 0;
  heading_marker_.pose.orientation.z = 0;
  heading_marker_.pose.position.x = 0;
  heading_marker_.pose.position.y = 0;
  heading_marker_.pose.position.z = 0;
  heading_pub_->publish(heading_marker_);
}

void RvizHelper::PublishStart(const Eigen::Vector3d &_point) {
  geometry_msgs::msg::Point p;
  hippo_common::convert::EigenToRos(_point, p);
  start_marker_.header.stamp = node_->get_clock()->now();
  start_marker_.pose.position = p;
  start_pub_->publish(start_marker_);
}

void RvizHelper::InitHeadingMarker() {
  heading_marker_.header.frame_id = "map";
  heading_marker_.ns = "heading";
  heading_marker_.action = visualization_msgs::msg::Marker::ADD;
  heading_marker_.id = 0;
  heading_marker_.type = visualization_msgs::msg::Marker::ARROW;
  heading_marker_.color.a = 1.0;
  heading_marker_.color.g = 1.0;
}

void RvizHelper::InitTargetMarker() {
  target_marker_.header.frame_id = "map";
  target_marker_.ns = "target";
  target_marker_.action = visualization_msgs::msg::Marker::ADD;
  target_marker_.id = 0;
  target_marker_.type = visualization_msgs::msg::Marker::SPHERE;

  target_marker_.scale.x = 0.3;
  target_marker_.scale.y = 0.3;
  target_marker_.scale.z = 0.3;

  target_marker_.color.a = 1.0;
  target_marker_.color.r = 1.0;
  target_marker_.pose.orientation.w = 1.0;
}

void RvizHelper::InitStartMarker() {
  start_marker_.header.frame_id = "map";
  start_marker_.ns = "start";
  start_marker_.action = visualization_msgs::msg::Marker::ADD;
  start_marker_.id = 0;
  start_marker_.type = visualization_msgs::msg::Marker::SPHERE;

  start_marker_.scale.x = 0.3;
  start_marker_.scale.y = 0.3;
  start_marker_.scale.z = 0.3;

  start_marker_.color.a = 1.0;
  start_marker_.color.b = 1.0;
  start_marker_.pose.orientation.w = 1.0;
}

void RvizHelper::InitThrustMarker() {
  thrust_marker_.header.frame_id = "map";
  thrust_marker_.ns = "acceleration";
  thrust_marker_.action = visualization_msgs::msg::Marker::ADD;
  thrust_marker_.id = 0;
  thrust_marker_.type = visualization_msgs::msg::Marker::LINE_LIST;

  thrust_marker_.scale.x = 0.01;

  thrust_marker_.color.a = 1.0;
  thrust_marker_.color.g = 1.0;
  thrust_marker_.points.resize(n_trajectory_samples_ * 2);
  thrust_marker_.pose.orientation.w = 1.0;
}

}  // namespace rapid_trajectories
