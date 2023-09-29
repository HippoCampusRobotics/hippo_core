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

#include "path_planning/rviz_helper.hpp"

#include <hippo_common/convert.hpp>
#include <hippo_common/tf2_utils.hpp>

namespace path_planning {
RvizHelper::RvizHelper(rclcpp::Node::SharedPtr _node) {
  node_ = _node;
  if (_node == nullptr) {
    return;
  }
  using visualization_msgs::msg::MarkerArray;
  rviz_pub_ = node_->create_publisher<MarkerArray>("marker", 10);

  InitPathMarker();
  InitTargetMarker();
}

void RvizHelper::PublishPath(const std::shared_ptr<Path> _path) {
  if (rviz_pub_ == nullptr) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Marker publisher not initialized");
    return;
  }
  geometry_msgs::msg::Point point;
  const auto target_point = _path->TargetPoint();
  hippo_common::convert::EigenToRos(target_point, point);
  target_marker_.pose.position = point;
  path_marker_.points.clear();
  path_marker_.points.reserve(_path->Size());
  for (std::size_t i = 0; i < _path->Size(); ++i) {
    hippo_common::convert::EigenToRos(_path->waypoints()[i], point);
    path_marker_.points.push_back(point);
  }
  marker_array_.markers = {path_marker_, target_marker_};
  rviz_pub_->publish(marker_array_);
}

void RvizHelper::InitPathMarker() {
  using visualization_msgs::msg::Marker;
  using namespace hippo_common::tf2_utils;
  path_marker_.header.frame_id = frame_id::InertialFrame();
  path_marker_.ns = std::string{node_->get_name()} + "_path";
  path_marker_.action = Marker::ADD;
  path_marker_.id = 0;
  path_marker_.type = Marker::LINE_STRIP;
  path_marker_.color.a = 0.7;
  path_marker_.color.r = 0.0;
  path_marker_.color.g = 1.0;
  path_marker_.color.b = 0.0;
  path_marker_.scale.x = 0.02;
  path_marker_.scale.y = 0.02;
  path_marker_.scale.z = 0.02;
  // make sure rviz does not complain about invalid quaternion
  path_marker_.pose.orientation.w = 1.0;
}

void RvizHelper::InitTargetMarker() {
  using visualization_msgs::msg::Marker;
  using namespace hippo_common::tf2_utils;
  Marker &m = target_marker_;
  m.header.frame_id = frame_id::InertialFrame();
  m.ns = std::string{node_->get_name()} + "_target";
  m.action = Marker::ADD;
  m.id = 0;
  m.type = Marker::SPHERE;
  m.color.a = 0.7;
  m.color.r = 1.0;
  m.color.g = 0.0;
  m.color.b = 0.0;
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.pose.orientation.w = 1.0;
}
}  // namespace path_planning
