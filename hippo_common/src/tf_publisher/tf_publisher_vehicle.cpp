#include "tf_publisher_vehicle.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <hippo_common/param_utils.hpp>
#include <hippo_common/tf2_utils.hpp>

namespace hippo_common {

TfPublisherVehicle::TfPublisherVehicle(rclcpp::NodeOptions const &_options)
    : Node("tf_publisher", _options) {
  static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  DeclareParameters();
  BroadCastStatic();

  std::string topic;
  topic = "odometry";
  rclcpp::SensorDataQoS qos;
  qos.keep_last(1);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      topic, qos, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        OnOdometry(msg);
      });

  topic = "vision_pose_cov";
  vision_pose_cov_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          topic, qos,
          [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
                     msg) { OnVisionPoseCovariance(msg); });
}

void TfPublisherVehicle::OnOdometry(
    const nav_msgs::msg::Odometry::SharedPtr _msg) {
  // simply publish the base link TF based on the odometry msg
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = now();
  tf.header.frame_id = _msg->header.frame_id;
  tf.child_frame_id = hippo_common::tf2_utils::frame_id::BaseLink(this);
  tf.transform.translation.x = _msg->pose.pose.position.x;
  tf.transform.translation.y = _msg->pose.pose.position.y;
  tf.transform.translation.z = _msg->pose.pose.position.z;
  tf.transform.rotation = _msg->pose.pose.orientation;

  if (tf_broadcaster_ == nullptr) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Broadcaster not available. Won't publish transformation %s -> %s",
        tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
    return;
  }
  tf_broadcaster_->sendTransform(tf);
}

void TfPublisherVehicle::OnVisionPoseCovariance(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr _msg) {
  // simply publish the base link TF based on the vision pose
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = now();
  tf.header.frame_id = _msg->header.frame_id;
  tf.child_frame_id = hippo_common::tf2_utils::frame_id::VisionBaseLink(this);
  tf.transform.translation.x = _msg->pose.pose.position.x;
  tf.transform.translation.y = _msg->pose.pose.position.y;
  tf.transform.translation.z = _msg->pose.pose.position.z;
  tf.transform.rotation = _msg->pose.pose.orientation;

  if (tf_broadcaster_ == nullptr) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Broadcaster not available. Won't publish transformation %s -> %s",
        tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
    return;
  }
  tf_broadcaster_->sendTransform(tf);
}

void TfPublisherVehicle::DeclareParameters() {
  DeclareVerticalCameraParameters();
  DeclareFrontCameraParameters();
}

void TfPublisherVehicle::DeclareCameraParameters(std::string _camera_name,
                                                 Pose &_pose_param) {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  name = _camera_name + ".x";
  descr_text = "Camera offset in x direction relative to base_link";
  descr = param_utils::Description(descr_text, true);
  _pose_param.x = declare_parameter(name, _pose_param.x, descr);

  name = _camera_name + ".y";
  descr_text = "Camera offset in y direction relative to base_link";
  descr = param_utils::Description(descr_text, true);
  _pose_param.y = declare_parameter(name, _pose_param.y, descr);

  name = _camera_name + ".z";
  descr_text = "Camera offset in z direction relative to base_link";
  descr = param_utils::Description(descr_text, true);
  _pose_param.z = declare_parameter(name, _pose_param.z, descr);

  name = _camera_name + ".qw";
  descr_text =
      "Quaternion component of the camera orientation relative to base_link";
  descr = param_utils::Description(descr_text, true);
  _pose_param.qw = declare_parameter(name, _pose_param.qw, descr);

  name = _camera_name + ".qx";
  descr_text =
      "Quaternion component of the camera orientation relative to base_link";
  descr = param_utils::Description(descr_text, true);
  _pose_param.qx = declare_parameter(name, _pose_param.qx, descr);

  name = _camera_name + ".qy";
  descr_text =
      "Quaternion component of the camera orientation relative to base_link";
  descr = param_utils::Description(descr_text, true);
  _pose_param.qy = declare_parameter(name, _pose_param.qy, descr);

  name = _camera_name + ".qz";
  descr_text =
      "Quaternion component of the camera orientation relative to base_link";
  descr = param_utils::Description(descr_text, true);
  _pose_param.qz = declare_parameter(name, _pose_param.qz, descr);
}

void TfPublisherVehicle::DeclareVerticalCameraParameters() {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  name = "has_vertical_camera";
  descr_text = "If vehicle has vertical camera";
  descr = param_utils::Description(descr_text, true);
  params_.has_vertical_camera = declare_parameter(name, false, descr);
  if (params_.has_vertical_camera) {
    DeclareCameraParameters("vertical_camera", params_.vertical_camera);
  }
}

void TfPublisherVehicle::DeclareFrontCameraParameters() {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  name = "has_front_camera";
  descr_text = "If vehicle has front camera";
  descr = param_utils::Description(descr_text, true);
  params_.has_front_camera = declare_parameter(name, false, descr);
  if (params_.has_front_camera) {
    DeclareCameraParameters("front_camera", params_.front_camera);
  }
}

void TfPublisherVehicle::BroadCastStatic() {
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  {
    geometry_msgs::msg::TransformStamped t;
    t.transform = hippo_common::tf2_utils::FLUtoFRD();
    std::string child_frame =
        hippo_common::tf2_utils::frame_id::BaseLinkFrd(this);
    t.header.frame_id = hippo_common::tf2_utils::frame_id::BaseLink(this);
    t.child_frame_id = child_frame;
    transforms.push_back(t);
  }
  // general transformation between a camera coordinate system (z in forward
  // direction) and the frd-baselink
  {
    geometry_msgs::msg::TransformStamped t;
    t.transform = hippo_common::tf2_utils::FLUtoFRD();
    t.header.frame_id = hippo_common::tf2_utils::frame_id::VisionBaseLink(this);
    t.child_frame_id =
        hippo_common::tf2_utils::frame_id::VisionBaseLinkFrd(this);
    transforms.push_back(t);
  }
  // transformation between baselink and the vertical camera
  if (params_.has_vertical_camera) {
    RCLCPP_INFO_STREAM(get_logger(),
                       "Publishing vertical camera transformation.");
    geometry_msgs::msg::TransformStamped t;
    t.transform.translation.x = params_.vertical_camera.x;
    t.transform.translation.y = params_.vertical_camera.y;
    t.transform.translation.z = params_.vertical_camera.z;
    t.transform.rotation.w = params_.vertical_camera.qw;
    t.transform.rotation.x = params_.vertical_camera.qx;
    t.transform.rotation.y = params_.vertical_camera.qy;
    t.transform.rotation.z = params_.vertical_camera.qz;
    t.header.frame_id = tf2_utils::frame_id::BaseLink(this);
    t.child_frame_id = tf2_utils::frame_id::VerticalCameraLink(this);
    transforms.push_back(t);
    // same transformation for the vision subtree
    t.header.frame_id = tf2_utils::frame_id::VisionBaseLink(this);
    t.child_frame_id = tf2_utils::frame_id::VisionVerticalCameraLink(this);
    transforms.push_back(t);

    geometry_msgs::msg::TransformStamped t2;
    t2.transform = hippo_common::tf2_utils::CameraLinkToCameraFrame();
    t2.header.frame_id =
        hippo_common::tf2_utils::frame_id::VerticalCameraLink(this);
    t2.child_frame_id =
        hippo_common::tf2_utils::frame_id::VerticalCameraFrame(this);
    transforms.push_back(t2);
    // same transformation for the vision subtree
    t2.header.frame_id = tf2_utils::frame_id::VisionVerticalCameraLink(this);
    t2.child_frame_id = tf2_utils::frame_id::VisionVerticalCameraFrame(this);
    transforms.push_back(t2);
  }

  if (params_.has_front_camera) {
    RCLCPP_INFO_STREAM(get_logger(), "Publishing front camera transformation.");
    geometry_msgs::msg::TransformStamped t;
    t.transform.translation.x = params_.front_camera.x;
    t.transform.translation.y = params_.front_camera.y;
    t.transform.translation.z = params_.front_camera.z;
    t.transform.rotation.w = params_.front_camera.qw;
    t.transform.rotation.x = params_.front_camera.qx;
    t.transform.rotation.y = params_.front_camera.qy;
    t.transform.rotation.z = params_.front_camera.qz;
    t.header.frame_id = hippo_common::tf2_utils::frame_id::BaseLink(this);
    t.child_frame_id = hippo_common::tf2_utils::frame_id::FrontCameraLink(this);
    transforms.push_back(t);
    // same transformation for the vision subtree
    t.header.frame_id = tf2_utils::frame_id::VisionBaseLink(this);
    t.child_frame_id = tf2_utils::frame_id::FrontCameraName(this);
    transforms.push_back(t);

    geometry_msgs::msg::TransformStamped t2;
    t2.transform = hippo_common::tf2_utils::CameraLinkToCameraFrame();
    t2.header.frame_id =
        hippo_common::tf2_utils::frame_id::FrontCameraLink(this);
    t2.child_frame_id =
        hippo_common::tf2_utils::frame_id::FrontCameraFrame(this);
    transforms.push_back(t2);
  }
  static_broadcaster_->sendTransform(transforms);
}
}  // namespace hippo_common

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hippo_common::TfPublisherVehicle)
