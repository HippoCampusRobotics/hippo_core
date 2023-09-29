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
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "path_planning/path.hpp"

namespace path_planning {
class RvizHelper {
 public:
  RvizHelper(rclcpp::Node::SharedPtr node);
  void PublishPath(const std::shared_ptr<Path> _path);

 private:
  void InitPathMarker();
  void InitTargetMarker();
  rclcpp::Node::SharedPtr node_{nullptr};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_pub_{
      nullptr};

  visualization_msgs::msg::Marker path_marker_;
  visualization_msgs::msg::Marker target_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
};
}  // namespace path_planning
