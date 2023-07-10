#pragma once
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace hippo_common {
namespace convert {

template <typename T>
constexpr bool always_false = false;

template <typename In, typename Out>
void EigenToRos(const In &, Out &) {
  static_assert(always_false<In>, "This conversion is not implemented");
};

template <>
void EigenToRos(const Eigen::Ref<const Eigen::Vector3d> &_eigen, geometry_msgs::msg::Point &_ros);

template <>
void EigenToRos(const Eigen::Ref<const Eigen::Vector3d> &_eigen,
                geometry_msgs::msg::Vector3 &_ros);

template <>
void EigenToRos(const Eigen::Vector3d &_eigen, geometry_msgs::msg::Point &_ros);

template <>
void EigenToRos(const Eigen::Vector3d &_eigen,
                geometry_msgs::msg::Vector3 &_ros);

template <>
void EigenToRos(const Eigen::Quaterniond &_eigen,
                geometry_msgs::msg::Quaternion &_ros);

template <typename In, typename Out>
void RosToEigen(const In &, Out &) {
  static_assert(always_false<In>, "This conversion is not implemented");
};

template <>
void RosToEigen(const geometry_msgs::msg::Point &_ros, Eigen::Vector3d &_eigen);

template <>
void RosToEigen(const geometry_msgs::msg::Vector3 &_ros,
                Eigen::Vector3d &_eigen);

template <typename In, typename Out>
void RosToEigenRef(const In &, Out) {
    static_assert(always_false<In>, "This conversion is not implemented");
};

template <>
void RosToEigenRef(const geometry_msgs::msg::Point &_ros, Eigen::Ref<Eigen::Vector3d> _eigen);

template <>
void RosToEigenRef(const geometry_msgs::msg::Vector3 &_ros,
                Eigen::Ref<Eigen::Vector3d> _eigen);

template <>
void RosToEigen(const geometry_msgs::msg::Quaternion &_ros,
                Eigen::Quaterniond &_eigen);

template <typename In, typename Out>
Out VectorPoint(const In &in, Out &out) {
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
  return out;
}
}  // namespace convert
}  // namespace hippo_common
