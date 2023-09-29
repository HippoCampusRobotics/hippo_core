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

#include <hippo_common/param_utils.hpp>

#include "path_follower_node.hpp"

namespace path_planning {
void PathFollowerNode::DeclareParams() {
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(look_ahead_distance);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(depth_gain);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(ignore_z_distance);

  HIPPO_COMMON_DECLARE_PARAM_READONLY(path_file);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(mode);

  HIPPO_COMMON_DECLARE_PARAM_READONLY(static_axis.position.x);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(static_axis.position.y);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(static_axis.position.z);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(static_axis.heading.x);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(static_axis.heading.y);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(static_axis.heading.z);

  HIPPO_COMMON_DECLARE_PARAM_READONLY(static_heading.x);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(static_heading.y);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(static_heading.z);

  switch (params_.mode) {
    case mode::kStaticAxis:
      SetDesiredStaticAxis();
      break;
    case mode::kStaticHeading:
      SetStaticHeading();
      break;
    default:
      break;
  }

  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(left_wall);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(right_wall);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(bottom_wall);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(surface);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(domain_end);

  params_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        return OnParameters(params);
      });
}

rcl_interfaces::msg::SetParametersResult PathFollowerNode::OnParameters(
    const std::vector<rclcpp::Parameter> &_parameters) {
  std::string text;
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "unhandled";
  result.successful = true;
  bool updated{false};

  for (const auto &parameter : _parameters) {
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(look_ahead_distance, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(depth_gain, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(ignore_z_distance, updated, text);

    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(left_wall, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(right_wall, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(bottom_wall, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(surface, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(domain_end, updated, text);
  }
  if (updated) {
    path_->SetLookAhead(params_.look_ahead_distance);
    path_->ignore_z() = params_.ignore_z_distance;
  }
  return result;
}
}  // namespace path_planning
