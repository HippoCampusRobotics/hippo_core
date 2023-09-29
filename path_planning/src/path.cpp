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

#include "path_planning/path.hpp"

#include <fstream>

namespace path_planning {

Path::Path(const std::vector<Eigen::Vector3d> &_waypoints, bool _loop)
    : waypoints_(_waypoints),
      target_index_(0),
      look_ahead_distance_(0.3),
      loop_(_loop) {}

bool Path::Update(const Eigen::Vector3d &_position) {
  double look_ahead_square = look_ahead_distance_ * look_ahead_distance_;
  std::size_t index{target_index_};
  bool success{false};

  if (waypoints_.size() < 1) {
    return false;
  }

  for (std::size_t i = 0; i < waypoints_.size(); ++i) {
    if (!(index < waypoints_.size())) {
      index = 0;
      if (!loop_) {
        break;
      }
    }

    Eigen::Vector3d d_vec{waypoints_[index] - _position};
    double d_square = (d_vec).squaredNorm() -
                      static_cast<double>(ignore_z_) * d_vec.z() * d_vec.z();
    if (d_square > look_ahead_square) {
      success = true;
      break;
    }
    ++index;
  }
  if (!success && !loop_) {
    index = waypoints_.size() - 1;
  }
  target_index_ = index;
  target_point_ = waypoints_[target_index_];
  return success;
}

void Path::Move(const Eigen::Vector3d &_offset) {
  for (auto &waypoint : waypoints_) {
    waypoint += _offset;
  }
}

void Path::Scale(double _scale) {
  for (auto &waypoint : waypoints_) {
    waypoint *= _scale;
  }
}

void Path::Rotate(const Eigen::Quaterniond &_rotation) {
  for (auto &waypoint : waypoints_) {
    waypoint = _rotation * waypoint;
  }
}

void Path::SwapXY() {
  for (auto &waypoint : waypoints_) {
    double tmp = waypoint.x();
    waypoint.x() = waypoint.y();
    waypoint.y() = tmp;
  }
}

bool Path::WriteToYAML(const std::string &_path) {
  std::ofstream f;
  f.open(_path);
  if (f.fail()) {
    f.close();
    return false;
  }
  YAML::Node node = ToYAML();
  f << node;
  f.close();
  return true;
}

void Path::LoadFromYAML(const std::string &_path) {
  YAML::Node node = YAML::LoadFile(_path);
  loop_ = node["loop"].as<bool>();
  waypoints_ = node["waypoints"].as<hippo_common::yaml::Waypoints>();
}

YAML::Node Path::ToYAML() {
  YAML::Node node;
  node["loop"] = loop_;
  node["waypoints"] = waypoints_;
  return node;
}

void Path::AddToYAML(YAML::Node &_node) {
  _node["loop"] = loop_;
  _node["waypoints"] = waypoints_;
}

}  // namespace path_planning
