#pragma once

#include <eigen3/Eigen/Dense>
namespace rapid_trajectories {
namespace minimum_jerk {
class Target {
 public:
  Target(const Eigen::Vector3d &_p0, const Eigen::Vector3d &_v0,
         const Eigen::Vector3d &_a0, const Eigen::Quaterniond &_q0);
  virtual Eigen::Vector3d Position(double _t) = 0;
  virtual Eigen::Quaterniond Orientation(double _t) = 0;

 protected:
  Eigen::Vector3d p0_;
  Eigen::Vector3d v0_;
  Eigen::Vector3d a0_;
  Eigen::Quaterniond q0_;
};

class TargetUniform : public Target {
 public:
  TargetUniform(const Eigen::Vector3d &_p0, const Eigen::Vector3d &_v0,
                const Eigen::Vector3d &_a0, const Eigen::Quaterniond &_q0);
  Eigen::Vector3d Position(double _t);
  Eigen::Quaterniond Orientation(double _t);
};
}  // namespace minimum_jerk
}  // namespace rapid_trajectories
