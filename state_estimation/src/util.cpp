#include <state_estimation/util.h>

double KahanSummation(double sum, double input, double &accumulator) {
  const double y = input - accumulator;
  const double t = sum + y;
  accumulator = (t - sum) - y;
  return t;
}

Eigen::Matrix3d TaitBryan312ToRotationMatrix(const Eigen::Vector3d &euler) {
  // Calculate the frame2 to frame 1 rotation matrix from a 312 Tait-Bryan
  // rotation sequence
  const float c2 = cosf(euler(2));  // third rotation is pitch
  const float s2 = sinf(euler(2));
  const float s1 = sinf(euler(1));  // second rotation is roll
  const float c1 = cosf(euler(1));
  const float s0 = sinf(euler(0));  // first rotation is yaw
  const float c0 = cosf(euler(0));

  Eigen::Matrix3d R;
  R(0, 0) = c0 * c2 - s0 * s1 * s2;
  R(1, 1) = c0 * c1;
  R(2, 2) = c2 * c1;
  R(0, 1) = -c1 * s0;
  R(0, 2) = s2 * c0 + c2 * s1 * s0;
  R(1, 0) = c2 * s0 + s2 * s1 * c0;
  R(1, 2) = s0 * s2 - s1 * c0 * c2;
  R(2, 0) = -s2 * c1;
  R(2, 1) = s1;

  return R;
}

Eigen::Matrix3d UpdateYawInRotationMatrix(double yaw,
                                          const Eigen::Matrix3d &R) {
  if (ShouldUse321RotationSequence(R)) {
    Eigen::Vector3d rpy = R.eulerAngles(0, 1, 2);
    rpy(2) = yaw;
    Eigen::AngleAxisd roll(rpy(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(rpy(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(rpy(2), Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yaw * pitch * roll;
    return q.matrix();
  } else {
    const Eigen::Vector3d ypr(yaw, asin(R(2, 1)), atan2(-R(2, 0), R(2, 2)));
    return TaitBryan312ToRotationMatrix(ypr);
  }
}

Eigen::Quaterniond EulerToQuaternion(double _roll, double _pitch, double _yaw) {
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(_yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(_pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(_roll, Eigen::Vector3d::UnitX());
  return q;
}
