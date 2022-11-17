#include <hippo_common/convert.hpp>

namespace hippo_common {
namespace convert {

template <>
void EigenToRos(const Eigen::Vector3d &_eigen,
                geometry_msgs::msg::Point &_ros) {
  _ros.x = _eigen.x();
  _ros.y = _eigen.y();
  _ros.z = _eigen.z();
}

template <>
void EigenToRos(const Eigen::Vector3d &_eigen,
                geometry_msgs::msg::Vector3 &_ros) {
  _ros.x = _eigen.x();
  _ros.y = _eigen.y();
  _ros.z = _eigen.z();
}

template <>
void EigenToRos(const Eigen::Quaterniond &_eigen,
                geometry_msgs::msg::Quaternion &_ros) {
  _ros.w = _eigen.w();
  _ros.x = _eigen.x();
  _ros.y = _eigen.y();
  _ros.z = _eigen.z();
}

template <>
void RosToEigen(const geometry_msgs::msg::Point &_ros,
                Eigen::Vector3d &_eigen) {
  _eigen.x() = _ros.x;
  _eigen.y() = _ros.y;
  _eigen.z() = _ros.z;
}

template <>
void RosToEigen(const geometry_msgs::msg::Vector3 &_ros,
                Eigen::Vector3d &_eigen) {
  _eigen.x() = _ros.x;
  _eigen.y() = _ros.y;
  _eigen.z() = _ros.z;
}

template <>
void RosToEigen(const geometry_msgs::msg::Quaternion &_ros,
                Eigen::Quaterniond &_eigen) {
  _eigen.w() = _ros.w;
  _eigen.x() = _ros.x;
  _eigen.y() = _ros.y;
  _eigen.z() = _ros.z;
}
}  // namespace convert
}  // namespace hippo_common
