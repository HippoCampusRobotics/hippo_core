// Copyright (C) 2023 Thies Lennart Alff
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
// USA

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include "hippo_control/trajectory_tracking/tracking_controller.hpp"

namespace hippo_control {
namespace trajectory_tracking {

void TrackingControllerNode::DeclareParams() {
  std::string name;
  std::string description;
  rcl_interfaces::msg::ParameterDescriptor descriptor;

  name = "position_gain";
  description = "Gain of the position error.";
  descriptor = hippo_common::param_utils::Description(description);
  {
    auto &param = params_.position_gain;
    param = declare_parameter(name, param, descriptor);
  }

  name = "velocity_gain";
  description = "Gain of the velocity error.";
  descriptor = hippo_common::param_utils::Description(description);
  {
    auto &param = params_.velocity_gain;
    param = declare_parameter(name, param, descriptor);
  }

  params_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &_params) {
        return OnParams(_params);
      });
}

rcl_interfaces::msg::SetParametersResult TrackingControllerNode::OnParams(
    const std::vector<rclcpp::Parameter> &_parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "Unhandled";
  result.successful = true;
  std::string name;
  std::string result_text;
  for (const rclcpp::Parameter &parameter : _parameters) {
    name = "position_gain";
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, name, params_.position_gain, result_text)) {
      result.reason = result_text;
      RCLCPP_INFO_STREAM(get_logger(), result_text);
      continue;
    }
    name = "velocity_gain";
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, name, params_.velocity_gain, result_text)) {
      result.reason = result_text;
      RCLCPP_INFO_STREAM(get_logger(), result_text);
      continue;
    }
  }
  if (result.reason != "Unhandled") {
    params_.updated = true;
  }
  return result;
}

}  // namespace trajectory_tracking
}  // namespace hippo_control
