#pragma once
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <hippo_common/convert.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
namespace hippo_common {
namespace tf2_utils {
static constexpr double kPi = 3.141592653589793238463;
Eigen::Quaterniond EulerToQuaternion(double _roll, double _pitch, double _yaw);
/// @brief Computes euler angles as roll, pitch, yaw from quaternion.
/// @param _q Has to be a normalized quaternion!
/// @return
inline Eigen::Vector3d QuaternionToEuler(const Eigen::Quaterniond &_q) {
  return _q.toRotationMatrix().eulerAngles(0, 1, 2);
}

/// @brief Make sure both vectors have unit length. Returns a quaternion
/// describing the rotation between both vectors so _v2 = q.rotate(_v1)
/// @param _v1
/// @param _v2
/// @return
Eigen::Quaterniond RotationBetweenNormalizedVectors(const Eigen::Vector3d &_v1,
                                                    const Eigen::Vector3d &_v2);
geometry_msgs::msg::Transform ENUtoNED() {
  geometry_msgs::msg::Transform t;
  auto q = EulerToQuaternion(kPi, 0, 0.5 * kPi);
  hippo_common::convert::EigenToRos(q, t.rotation);
  return t;
}

geometry_msgs::msg::Transform NEDtoENU() {
  geometry_msgs::msg::Transform t;
  Eigen::Quaterniond q = EulerToQuaternion(kPi, 0.0, 0.5 * kPi).inverse();
  hippo_common::convert::EigenToRos(q, t.rotation);
  return t;
}

geometry_msgs::msg::Transform FLUtoFRD() {
  geometry_msgs::msg::Transform t;
  Eigen::Quaterniond q = EulerToQuaternion(kPi, 0.0, 0.0);
  hippo_common::convert::EigenToRos(q, t.rotation);
  return t;
}

geometry_msgs::msg::Transform FRDtoFLU() {
  geometry_msgs::msg::Transform t;
  Eigen::Quaterniond q = EulerToQuaternion(kPi, 0.0, 0.0).inverse();
  hippo_common::convert::EigenToRos(q, t.rotation);
  return t;
}

geometry_msgs::msg::Transform CameraLinkToCameraFrame() {
  geometry_msgs::msg::Transform t;
  Eigen::Quaternion q = EulerToQuaternion(-0.5 * kPi, 0.0, -0.5 * kPi);
  hippo_common::convert::EigenToRos(q, t.rotation);
  return t;
}

geometry_msgs::msg::Transform CameraFrameToCameraLink() {
  geometry_msgs::msg::Transform t;
  Eigen::Quaternion q =
      EulerToQuaternion(-0.5 * kPi, 0.0, -0.5 * kPi).inverse();
  hippo_common::convert::EigenToRos(q, t.rotation);
  return t;
}

namespace frame_id {
static constexpr char kBarometerName[] = "barometer";
static constexpr char kBaseLinkName[] = "base_link";
static constexpr char kInertialName[] = "map";
static constexpr char kVerticalCameraLinkName[] = "vertical_camera_link";
static constexpr char kVerticalCameraFrameName[] = "vertical_camera_frame";

inline std::string Prefix(rclcpp::Node *_node) {
  std::string name = _node->get_namespace();
  // remove leading slash as tf2 does not like leading slashes.
  name.erase(0, name.find_first_not_of('/'));
  return name;
}
inline std::string Barometer(rclcpp::Node *_node) {
  return Prefix(_node) + "/" + kBarometerName;
}
inline std::string BaseLink(rclcpp::Node *_node) {
  return Prefix(_node) + "/" + kBaseLinkName;
}
inline std::string VerticalCameraLink(rclcpp::Node *_node) {
  return Prefix(_node) + "/" + kVerticalCameraLinkName;
}
inline std::string VerticalCameraFrame(rclcpp::Node *_node) {
  return Prefix(_node) + "/" + kVerticalCameraFrameName;
}

inline std::string InertialFrame() { return kInertialName; }

}  // namespace frame_id
}  // namespace tf2_utils
}  // namespace hippo_common
