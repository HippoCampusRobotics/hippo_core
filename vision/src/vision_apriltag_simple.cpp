#include <tf2/time.h>

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <hippo_common/tf2_utils.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vision/vision_apriltag_simple.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(vision::AprilTagSimple)

namespace vision {
AprilTagSimple::AprilTagSimple(rclcpp::NodeOptions const &_options)
    : Node("apriltag_simple", _options) {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  rclcpp::QoS px4_qos{1};
  px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  px4_qos.history(rclcpp::HistoryPolicy::KeepLast);
  vision_delay_pub_ = create_publisher<std_msgs::msg::Float64>(
      "vision_delay", rclcpp::SystemDefaultsQoS());
  visual_odometry_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
      "fmu/in/vehicle_visual_odometry", px4_qos);
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
      "fmu/out/vehicle_attitude", px4_qos,
      std::bind(&AprilTagSimple::OnAttitude, this, std::placeholders::_1));

  update_timer_ =
      rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(20),
                           std::bind(&AprilTagSimple::OnUpdateOdometry, this));
}

void AprilTagSimple::OnAprilTagDetections(
    apriltag_ros::msg::AprilTagDetectionArray::ConstSharedPtr _msg) {
  if (_msg->detections.empty()) {
    return;
  }

  // by vision pose we mean the pose of the vehicle's base link determined
  // by the apriltag pose reconstruction. Note that the detection itself is
  // relative to the camera frame
  geometry_msgs::msg::PoseStamped vision_pose;
  if (!GetVisionPose(vision_pose)) {
    return;
  }
  vision_pose.header.stamp = _msg->header.stamp;
  if (!PublishVisionPose(vision_pose)) {
    RCLCPP_ERROR(get_logger(), "Failed to publish vision pose.");
    return;
  }
  if (!PublishVisualOdometry(vision_pose)) {
    RCLCPP_ERROR(get_logger(), "Failed to publish visual odometry.");
    return;
  }
}

void AprilTagSimple::OnUpdateOdometry() {
  if (!px4_attitude_update_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                          "No attitude received from px4.");
    return;
  }
  if (!px4_position_update_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                          "No position received from px4.");
    return;
  }
  px4_position_update_ = px4_attitude_update_ = false;

  rclcpp::Time stamp = now();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(position_px4_, pose.pose.position);
  hippo_common::convert::EigenToRos(orientation_px4_, pose.pose.orientation);
  pose.pose = hippo_common::tf2_utils::PosePx4ToRos(pose.pose);

  geometry_msgs::msg::Vector3Stamped velocity;
  velocity.header.stamp = stamp;
  velocity.header.frame_id =
      hippo_common::tf2_utils::frame_id::InertialFramePX4();
  hippo_common::convert::EigenToRos(velocity_px4_, velocity.vector);
  try {
    velocity = tf_buffer_->transform(
        velocity, hippo_common::tf2_utils::frame_id::InertialFrame());
  } catch (const tf2::TransformException &e) {
    RCLCPP_ERROR(get_logger(), "Could not lookup transform. %s", e.what());
    return;
  }

  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  odometry.child_frame_id = hippo_common::tf2_utils::frame_id::BaseLink(this);
  odometry.pose.pose = pose.pose;
  odometry.twist.twist.linear = velocity.vector;
  odometry_pub_->publish(odometry);

  // publish transformation for vehicle pose
  geometry_msgs::msg::TransformStamped t;
  hippo_common::convert::VectorPoint(odometry.pose.pose.position,
                                     t.transform.translation);
  t.transform.rotation = odometry.pose.pose.orientation;
  t.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  t.child_frame_id = hippo_common::tf2_utils::frame_id::BaseLink(this);
  tf_broadcaster_->sendTransform(t);
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

bool AprilTagSimple::GetVisionPose(
    geometry_msgs::msg::PoseStamped &_vision_pose) {
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer_->lookupTransform(
        params_.bundle_frame, hippo_common::tf2_utils::frame_id::BaseLink(this),
        rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException &e) {
    RCLCPP_ERROR(get_logger(), "Could not lookup transform %s to %s",
                 hippo_common::tf2_utils::frame_id::BaseLink(this).c_str(),
                 params_.bundle_frame.c_str());
    return false;
  }
  _vision_pose.header.frame_id =
      hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::VectorPoint(t.transform.translation,
                                     _vision_pose.pose.position);
  _vision_pose.pose.orientation = t.transform.rotation;
  return true;
}

bool AprilTagSimple::PublishVisionPose(
    const geometry_msgs::msg::PoseStamped _pose) {
  geometry_msgs::msg::PoseWithCovarianceStamped vision_pose;
  vision_pose.header.frame_id =
      hippo_common::tf2_utils::frame_id::InertialFrame();
  vision_pose.header.stamp = _pose.header.stamp;
  vision_pose.pose.pose = _pose.pose;
  vision_pose_pub_->publish(vision_pose);
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
    t.child_frame_id = hippo_common::tf2_utils::frame_id::VisionBaseLink(this);
    t.header.stamp = vision_pose.header.stamp;
    hippo_common::convert::VectorPoint(vision_pose.pose.pose.position,
                                       t.transform.translation);
    t.transform.rotation = vision_pose.pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);
  }
  std_msgs::msg::Float64 delay;
  rclcpp::Time stamp{vision_pose.header.stamp};
  delay.data = (now().nanoseconds() - stamp.nanoseconds()) * 1e-9;
  vision_delay_pub_->publish(delay);
  return true;
}

bool AprilTagSimple::PublishVisualOdometry(
    const geometry_msgs::msg::PoseStamped _pose) {
  geometry_msgs::msg::Pose px4_pose =
      hippo_common::tf2_utils::PoseRosToPx4(_pose.pose);
  px4_msgs::msg::VehicleOdometry visual_odometry;
  visual_odometry.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  visual_odometry.position = {(float)px4_pose.position.x,
                              (float)px4_pose.position.y,
                              (float)px4_pose.position.z};
  visual_odometry.q = {
      (float)px4_pose.orientation.w, (float)px4_pose.orientation.x,
      (float)px4_pose.orientation.y, (float)px4_pose.orientation.z};
  visual_odometry.timestamp = now().nanoseconds() * 1e-3;
  visual_odometry.timestamp_sample =
      _pose.header.stamp.nanosec * 1e-3 + _pose.header.stamp.sec * 1e6;
  visual_odometry.velocity = {NAN, NAN, NAN};
  visual_odometry.angular_velocity = {NAN, NAN, NAN};

  visual_odometry_pub_->publish(visual_odometry);
  return true;
}

bool AprilTagSimple::ToPX4Pose(const geometry_msgs::msg::PoseStamped &_pose_in,
                               geometry_msgs::msg::PoseStamped &_pose_out) {
  try {
    _pose_out = tf_buffer_->transform(
        _pose_in, hippo_common::tf2_utils::frame_id::InertialFramePX4());
  } catch (const tf2::TransformException &e) {
    RCLCPP_ERROR(get_logger(), "Could not lookup transform %s to %s",
                 hippo_common::tf2_utils::frame_id::BaseLink(this).c_str(),
                 params_.bundle_frame.c_str());
    return false;
  }
  _pose_out.pose = hippo_common::tf2_utils::PoseFLUtoFRD(_pose_out.pose);
  return true;
}
}  // namespace vision
