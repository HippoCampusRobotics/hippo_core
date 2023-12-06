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
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <hippo_msgs/msg/actuator_setpoint.hpp>

namespace hippo_common {
namespace convert {

template <typename T>
constexpr bool always_false = false;

template <typename In, typename Out>
inline void EigenToRos(const In &, Out &) {
  static_assert(always_false<In>, "This conversion is not implemented");
};

template <>
inline void EigenToRos(const Eigen::Vector3d &_eigen,
                       geometry_msgs::msg::Point &_ros) {
  _ros.x = _eigen.x();
  _ros.y = _eigen.y();
  _ros.z = _eigen.z();
}
template <>
inline void EigenToRos(const Eigen::Ref<const Eigen::Vector3d> &_eigen,
                       geometry_msgs::msg::Point &_ros) {
  _ros.x = _eigen.x();
  _ros.y = _eigen.y();
  _ros.z = _eigen.z();
}

template <>
inline void EigenToRos(const Eigen::Vector3d &_eigen,
                       geometry_msgs::msg::Vector3 &_ros) {
  _ros.x = _eigen.x();
  _ros.y = _eigen.y();
  _ros.z = _eigen.z();
}

template <>
inline void EigenToRos(const Eigen::Ref<const Eigen::Vector3d> &_eigen,
                       geometry_msgs::msg::Vector3 &_ros) {
  _ros.x = _eigen.x();
  _ros.y = _eigen.y();
  _ros.z = _eigen.z();
}

template <>
inline void EigenToRos(const Eigen::Quaterniond &_eigen,
                       geometry_msgs::msg::Quaternion &_ros) {
  _ros.w = _eigen.w();
  _ros.x = _eigen.x();
  _ros.y = _eigen.y();
  _ros.z = _eigen.z();
}

template <>
inline void EigenToRos(const Eigen::Vector3d &in,
                       hippo_msgs::msg::ActuatorSetpoint &out) {
  out.x = in.x();
  out.y = in.y();
  out.z = in.z();
}

template <typename In, typename Out>
inline void RosToEigen(const In &, Out &) {
  static_assert(always_false<In>, "This conversion is not implemented");
};

template <>
inline void RosToEigen(const geometry_msgs::msg::Point &_ros,
                       Eigen::Vector3d &_eigen) {
  _eigen.x() = _ros.x;
  _eigen.y() = _ros.y;
  _eigen.z() = _ros.z;
}

template <>
inline void RosToEigen(const geometry_msgs::msg::Vector3 &_ros,
                       Eigen::Vector3d &_eigen) {
  _eigen.x() = _ros.x;
  _eigen.y() = _ros.y;
  _eigen.z() = _ros.z;
}

template <>
inline void RosToEigen(const geometry_msgs::msg::Quaternion &_ros,
                       Eigen::Quaterniond &_eigen) {
  _eigen.w() = _ros.w;
  _eigen.x() = _ros.x;
  _eigen.y() = _ros.y;
  _eigen.z() = _ros.z;
}

template <typename In, typename Out>
void RosToEigenRef(const In &, Out) {
  static_assert(always_false<In>, "This conversion is not implemented");
};

template <>
inline void RosToEigenRef(const geometry_msgs::msg::Point &_ros,
                          Eigen::Ref<Eigen::Vector3d> _eigen) {
  _eigen.x() = _ros.x;
  _eigen.y() = _ros.y;
  _eigen.z() = _ros.z;
}

template <>
inline void RosToEigenRef(const geometry_msgs::msg::Vector3 &_ros,
                          Eigen::Ref<Eigen::Vector3d> _eigen) {
  _eigen.x() = _ros.x;
  _eigen.y() = _ros.y;
  _eigen.z() = _ros.z;
}

template <>
inline void RosToEigen(const hippo_msgs::msg::ActuatorSetpoint &in,
                       Eigen::Vector3d &out) {
  out.x() = in.x;
  out.y() = in.y;
  out.z() = in.z;
}

template <typename In, typename Out>
inline Out VectorPoint(const In &in, Out &out) {
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
  return out;
}
}  // namespace convert
}  // namespace hippo_common
