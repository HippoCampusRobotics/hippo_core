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
namespace frame_id {}  // namespace frame_id
}  // namespace tf2_utils
}  // namespace hippo_common
