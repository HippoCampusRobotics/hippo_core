// Copyright (C) 2023 Thies Lennart Alff
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
// USA

#pragma once
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
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

Eigen::Quaterniond QuaternionFromHeading(const Eigen::Vector3d &_heading,
                                         double _roll);

inline double AngleOfRotation(const Eigen::Quaterniond &q) {
  return atan2(q.vec().norm(), q.w());
}

/// @brief Make sure both vectors have unit length. Returns a quaternion
/// describing the shortest rotation between both vectors so _v2 = q.rotate(_v1)
/// @param _v1
/// @param _v2
/// @return
Eigen::Quaterniond RotationBetweenNormalizedVectors(const Eigen::Vector3d &_v1,
                                                    const Eigen::Vector3d &_v2);
geometry_msgs::msg::Transform ENUtoNED();

geometry_msgs::msg::Transform NEDtoENU();

geometry_msgs::msg::Transform FLUtoFRD();

geometry_msgs::msg::Transform FRDtoFLU();

geometry_msgs::msg::Transform CameraLinkToCameraFrame();

geometry_msgs::msg::Transform CameraFrameToCameraLink();

geometry_msgs::msg::Quaternion RotateByQuaternion(
    const geometry_msgs::msg::Quaternion &_orientation,
    const geometry_msgs::msg::Quaternion &_rotation);

geometry_msgs::msg::Pose PoseRosToPx4(const geometry_msgs::msg::Pose &_pose);

geometry_msgs::msg::Pose PosePx4ToRos(const geometry_msgs::msg::Pose &_pose);

namespace frame_id {
static constexpr char kBarometerName[] = "barometer";
static constexpr char kBaseLinkName[] = "base_link";
static constexpr char kInertialName[] = "map";
static constexpr char kVerticalCameraName[] = "vertical_camera";
static constexpr char kVerticalCameraLinkName[] = "vertical_camera_link";
static constexpr char kVerticalCameraFrameName[] = "vertical_camera_frame";
static constexpr char kFrontCameraName[] = "front_camera";
static constexpr char kFrontCameraLinkName[] = "front_camera_link";
static constexpr char kFrontCameraFrameName[] = "front_camera_frame";

inline std::string Prefix(rclcpp::Node *_node) {
  std::string name = _node->get_namespace();
  // remove leading slash as tf2 does not like leading slashes.
  name.erase(0, name.find_first_not_of('/'));
  return name;
}

inline std::string VisionPrefix(rclcpp::Node *_node) {
  return Prefix(_node) + "/vision";
}
inline std::string Barometer(rclcpp::Node *_node) {
  return Prefix(_node) + "/" + kBarometerName;
}
/// @brief Pose of the base link is defined by the vehicle's odometry, i.e. the
/// state estimation
/// @param _node Reference to the node living in the vehicle's namespace.
/// Usually provided by passing `this`.
/// @return
inline std::string BaseLink(rclcpp::Node *_node) {
  return Prefix(_node) + "/" + kBaseLinkName;
}
inline std::string BaseLinkFrd(rclcpp::Node *_node) {
  return BaseLink(_node) + "_frd";
}
/**
 * @brief Similar to BaseLink(rclcpp::Node *_node), but directly based in the
 * pose estimation of the vision system
 *
 * @param _node Reference to the node living in the vehicle's namespace. Usually
 * provided by passing `this`.
 * @return std::string
 */
inline std::string VisionBaseLink(rclcpp::Node *_node) {
  return VisionPrefix(_node) + "/" + kBaseLinkName;
}
inline std::string VisionBaseLinkFrd(rclcpp::Node *_node) {
  return VisionBaseLink(_node) + "_frd";
}
inline std::string VisionVerticalCameraLink(rclcpp::Node *_node) {
  return VisionPrefix(_node) + "/" + kVerticalCameraLinkName;
}
inline std::string VisionVerticalCameraFrame(rclcpp::Node *_node) {
  return Prefix(_node) + "/" + kVerticalCameraName;
}
inline std::string VerticalCameraLink(rclcpp::Node *_node) {
  return Prefix(_node) + "/" + kVerticalCameraLinkName;
}
inline std::string VerticalCameraFrame(rclcpp::Node *_node) {
  return Prefix(_node) + "/" + kVerticalCameraFrameName;
}
inline std::string FrontCameraName(rclcpp::Node *_node) {
  return Prefix(_node) + "/" + kFrontCameraName;
}
inline std::string FrontCameraLink(rclcpp::Node *_node) {
  return Prefix(_node) + "/" + kFrontCameraLinkName;
}
inline std::string FrontCameraFrame(rclcpp::Node *_node) {
  return Prefix(_node) + "/" + kFrontCameraFrameName;
}

inline std::string InertialFrame() { return kInertialName; }

inline std::string InertialFramePX4() {
  return std::string{kInertialName} + "_ned";
}

}  // namespace frame_id
}  // namespace tf2_utils
}  // namespace hippo_common
