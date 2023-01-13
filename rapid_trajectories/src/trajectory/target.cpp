#include <rapid_trajectories/trajectory/target.hpp>

namespace rapid_trajectories {
namespace minimum_jerk {

Target::Target(const Eigen::Vector3d &_p0, const Eigen::Vector3d &_v0,
               const Eigen::Vector3d &_a0, const Eigen::Quaterniond &_q0)
    : p0_(_p0), v0_(_v0), a0_(_a0), q0_(_q0) {}

TargetUniform::TargetUniform(const Eigen::Vector3d &_p0,
                             const Eigen::Vector3d &_v0,
                             const Eigen::Vector3d &_a0,
                             const Eigen::Quaterniond &_q0)
    : Target(_p0, _v0, _a0, _q0) {}

Eigen::Vector3d TargetUniform::Position(double _t) {
  return 0.5 * _t * _t * a0_ + v0_ * _t + p0_;
}

Eigen::Quaterniond TargetUniform::Orientation(double _t) { return q0_; }

}  // namespace minimum_jerk
}  // namespace rapid_trajectories
