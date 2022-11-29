#include <tf2/time.h>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <hippo_common/tf2_utils.hpp>
#include <vision/vision_apriltag_simple.hpp>

namespace vision {
AprilTagSimple::AprilTagSimple(rclcpp::NodeOptions const &_options)
    : Node("apriltag_simple", _options) {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::QoS px4_qos{1};
  px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  px4_qos.history(rclcpp::HistoryPolicy::KeepLast);

  visual_odometry_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
      "fmu/in/visual_odometry", px4_qos);
  vision_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "vision_pose", rclcpp::SystemDefaultsQoS());
  odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::SystemDefaultsQoS());
  apriltag_sub_ =
      create_subscription<apriltag_ros::msg::AprilTagDetectionArray>(
          "tag_detections", rclcpp::SystemDefaultsQoS(),
          std::bind(&AprilTagSimple::OnAprilTagDetections, this,
                    std::placeholders::_1));
  px4_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "fmu/out/vehicle_local_position", px4_qos,
      std::bind(&AprilTagSimple::OnLocalPosition, this, std::placeholders::_1));
  px4_attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "fmu/out/vehicle_atttiude", px4_qos,
      std::bind(&AprilTagSimple::OnAttitude, this, std::placeholders::_1));
}

void AprilTagSimple::OnAprilTagDetections(
    apriltag_ros::msg::AprilTagDetectionArray::ConstSharedPtr _msg) {
  if (_msg->detections.empty()) {
    return;
  }
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer_->lookupTransform(
        params_.bundle_frame, hippo_common::tf2_utils::frame_id::BaseLink(this),
        tf2::TimePointZero);
  } catch (const tf2::TransformException &e) {
    RCLCPP_ERROR(get_logger(), "Could not lookup transform %s to %s",
                 hippo_common::tf2_utils::frame_id::BaseLink(this).c_str(),
                 params_.bundle_frame.c_str());
    return;
  }
  geometry_msgs::msg::PoseWithCovarianceStamped vision_pose;
  vision_pose.header.frame_id =
      hippo_common::tf2_utils::frame_id::InertialFrame();
  vision_pose.header.stamp = _msg->header.stamp;
  vision_pose.pose.pose.position.x = t.transform.translation.x;
  vision_pose.pose.pose.position.y = t.transform.translation.y;
  vision_pose.pose.pose.position.z = t.transform.translation.z;
  vision_pose.pose.pose.orientation = t.transform.rotation;
  vision_pose_pub_->publish(vision_pose);

  // two steps:
  //   - convert enu to ned inertial frame
  //   - change target frame of pose from flu to frd
  geometry_msgs::msg::PoseStamped enu_pose;
  enu_pose.pose = _msg->detections.at(0).pose.pose.pose;
  enu_pose.header.stamp = _msg->header.stamp;
  enu_pose.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  geometry_msgs::msg::PoseStamped px4_pose = tf_buffer_->transform(
      enu_pose, hippo_common::tf2_utils::frame_id::InertialFramePX4());
  px4_pose.pose = hippo_common::tf2_utils::PoseFLUtoFRD(px4_pose.pose);

  px4_msgs::msg::VehicleOdometry visual_odometry;
  visual_odometry.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  visual_odometry.position = {(float)px4_pose.pose.position.x,
                              (float)px4_pose.pose.position.y,
                              (float)px4_pose.pose.position.z};
  visual_odometry.q = {
      (float)px4_pose.pose.orientation.w, (float)px4_pose.pose.orientation.x,
      (float)px4_pose.pose.orientation.y, (float)px4_pose.pose.orientation.z};
  visual_odometry.timestamp = now().nanoseconds() * 1e-3;
  visual_odometry.timestamp_sample =
      px4_pose.header.stamp.nanosec * 1e-3 + px4_pose.header.stamp.sec * 1e6;

  visual_odometry_pub_->publish(visual_odometry);
}

void AprilTagSimple::OnUpdateOdometry() {
  if (!px4_position_update_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                          "No position received from px4.");
    return;
  }
  if (!px4_attitude_update_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                          "No attitude received from px4.");
    return;
  }
  px4_position_update_ = px4_attitude_update_ = false;

  rclcpp::Time stamp = now();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFramePX4();
  hippo_common::convert::EigenToRos(position_px4_, pose.pose.position);
  hippo_common::convert::EigenToRos(orientation_px4_, pose.pose.orientation);
  pose.pose = hippo_common::tf2_utils::PoseFRDtoFLU(pose.pose);
  pose = tf_buffer_->transform(
      pose, hippo_common::tf2_utils::frame_id::InertialFrame());

  geometry_msgs::msg::Vector3Stamped velocity;
  velocity.header.stamp = stamp;
  velocity.header.frame_id =
      hippo_common::tf2_utils::frame_id::InertialFramePX4();
  hippo_common::convert::EigenToRos(velocity_px4_, velocity.vector);
  velocity = tf_buffer_->transform(
      velocity, hippo_common::tf2_utils::frame_id::InertialFrame());

  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.child_frame_id = hippo_common::tf2_utils::frame_id::BaseLink(this);
  odometry.pose.pose = pose.pose;
  odometry.twist.twist.linear = velocity.vector;
  odometry_pub_->publish(odometry);
}

void AprilTagSimple::OnLocalPosition(
    px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr _msg) {
  px4_position_update_ = true;
  position_px4_.x() = _msg->x;
  position_px4_.y() = _msg->y;
  position_px4_.z() = _msg->z;
  velocity_px4_.x() = _msg->vx;
  velocity_px4_.y() = _msg->vy;
  velocity_px4_.z() = _msg->vz;
}

void AprilTagSimple::OnAttitude(
    px4_msgs::msg::VehicleAttitude::ConstSharedPtr _msg) {
  px4_attitude_update_ = true;
  orientation_px4_.w() = _msg->q[0];
  orientation_px4_.x() = _msg->q[1];
  orientation_px4_.y() = _msg->q[2];
  orientation_px4_.z() = _msg->q[3];
}
}  // namespace vision
