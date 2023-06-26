#include "hippo_control/trajectory_tracking/tracking_controller.hpp"
namespace hippo_control {
namespace trajectory_tracking {

TrackingControllerNode::TrackingControllerNode(
    rclcpp::NodeOptions const &_options)
    : Node("tracking_controller", _options) {
  // DeclareParams();
  InitPublishers();
  InitSubscriptions();
}

void TrackingControllerNode::InitPublishers() {
  std::string topic;

  topic = "attitude_target";
  attitude_target_pub_ =
      create_publisher<hippo_msgs::msg::AttitudeTarget>(topic, 10);
}

void TrackingControllerNode::InitSubscriptions() {
  std::string topic;

  topic = "roll_target";
  roll_target_sub_ = create_subscription<std_msgs::msg::Float64>(
      topic, 10, [this](const std_msgs::msg::Float64::SharedPtr _msg) {
        OnRollTarget(_msg);
      });

  topic = "position_target";
  position_target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      topic, 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr _msg) {
        OnPositionTarget(_msg);
      });

  topic = "velocity_target";
  velocity_target_sub_ =
      create_subscription<geometry_msgs::msg::Vector3Stamped>(
          topic, 10,
          [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr _msg) {
            OnVelocityTarget(_msg);
          });

  topic = "odometry";
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      topic, 10, [this](const nav_msgs::msg::Odometry::SharedPtr _msg) {
        OnOdometry(_msg);
      });
}

void TrackingControllerNode::OnOdometry(
    const nav_msgs::msg::Odometry::SharedPtr _msg) {
  hippo_common::convert::RosToEigen(_msg->pose.pose.position, position_);
  hippo_common::convert::RosToEigen(_msg->twist.twist.linear, velocity_);
  hippo_common::convert::RosToEigen(_msg->pose.pose.orientation, attitude_);
  controller_.SetDesiredState(position_target_, velocity_target_, roll_target_);
  // TODO: add feedforward term based on some model!
  attitude_target_ =
      controller_.Update(position_, velocity_, Eigen::Vector3d::Zero());
  hippo_msgs::msg::AttitudeTarget msg;
  msg.header.stamp = now();
  msg.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(attitude_target_, msg.attitude);
  thrust_ = controller_.Thrust(attitude_);
  msg.thrust = thrust_;
  if (attitude_target_pub_ == nullptr) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Publisher for attitude_target not available");
    return;
  }
  attitude_target_pub_->publish(msg);
}

}  // namespace trajectory_tracking
}  // namespace hippo_control

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
    hippo_control::trajectory_tracking::TrackingControllerNode)
