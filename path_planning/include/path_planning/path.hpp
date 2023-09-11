// Copyright (C) 2023 Thies Lennart Alff

// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
// USA

#pragma once

#include <eigen3/Eigen/Dense>
#include <hippo_common/yaml.hpp>
#include <vector>

namespace path_planning {

class Path {
 public:
  Path() {}
  explicit Path(const std::vector<Eigen::Vector3d> &waypoints,
                bool loop = true);

  void SetLookAhead(double _distance) { look_ahead_distance_ = _distance; }
  inline std::size_t Size() const { return waypoints_.size(); }
  inline const std::vector<Eigen::Vector3d> &waypoints() const {
    return waypoints_;
  };
  bool Update(const Eigen::Vector3d &_position);
  bool UpdateMotorFailure(const Eigen::Vector3d &_position);
  inline Eigen::Vector3d TargetPoint() const { return target_point_; }
  void Move(const Eigen::Vector3d &_offset);
  void Scale(double _scale);
  void Rotate(const Eigen::Quaterniond &_rotation);
  void SwapXY();
  bool WriteToYAML(const std::string &_path);
  void LoadFromYAML(const std::string &_path);
  YAML::Node ToYAML();
  void AddToYAML(YAML::Node &_node);
  inline bool &ignore_z() { return ignore_z_; }

 private:
  Eigen::Vector3d target_point_{0.0, 0.0, 0.0};
  std::vector<Eigen::Vector3d> waypoints_;
  std::size_t target_index_{0};
  double look_ahead_distance_;
  bool ignore_z_{false};
  bool loop_;
};
}  // namespace path_planning
