#include "hippo_common/tf2_utils.hpp"

namespace hippo_common {
namespace tf2_utils {
Eigen::Quaterniond EulerToQuaternion(double _roll, double _pitch, double _yaw) {
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(_yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(_pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(_roll, Eigen::Vector3d::UnitX());
  return q;
}

Eigen::Quaterniond RotationBetweenNormalizedVectors(
    const Eigen::Vector3d &_v1, const Eigen::Vector3d &_v2) {
  // https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
  Eigen::Vector3d n = _v1.cross(_v2);
  Eigen::Quaterniond q;
  q.x() = n.x();
  q.y() = n.y();
  q.z() = n.z();
  q.w() = 1 + _v1.dot(_v2);
  q.normalize();
  return q;
}

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

geometry_msgs::msg::Quaternion RotateByQuaternion(
    const geometry_msgs::msg::Quaternion &_orientation,
    const geometry_msgs::msg::Quaternion &_rotation) {
  Eigen::Quaterniond q_orig{_orientation.w, _orientation.x, _orientation.y,
                            _orientation.z};
  Eigen::Quaterniond q_rot{_rotation.w, _rotation.x, _rotation.y, _rotation.z};
  geometry_msgs::msg::Quaternion q_new;
  hippo_common::convert::EigenToRos(q_orig * q_rot, q_new);
  return q_new;
}

geometry_msgs::msg::Pose PoseFLUtoFRD(const geometry_msgs::msg::Pose &_pose) {
  geometry_msgs::msg::Transform t{FLUtoFRD()};

  geometry_msgs::msg::Pose new_pose;
  new_pose.orientation = RotateByQuaternion(_pose.orientation, t.rotation);
  // no difference in position between FLU and FRD.
  new_pose.position = _pose.position;
  return new_pose;
}

geometry_msgs::msg::Pose PoseFRDtoFLU(const geometry_msgs::msg::Pose &_pose) {
  geometry_msgs::msg::Transform t{FRDtoFLU()};
  geometry_msgs::msg::Pose new_pose;
  new_pose.orientation = RotateByQuaternion(_pose.orientation, t.rotation);
  // no difference in position between FLU and FRD.
  new_pose.position = _pose.position;
  return new_pose;
}

namespace frame_id {}  // namespace frame_id
}  // namespace tf2_utils
}  // namespace hippo_common
