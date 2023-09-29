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

#include <hippo_common/param_utils.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include "quaternion_control_node.hpp"

namespace hippo_control {
namespace attitude_control {

void QuaternionControlNode::DeclareParams() {
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gain);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(roll_weight);

  SetControllerGains();

  params_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        return OnParameters(params);
      });
}

rcl_interfaces::msg::SetParametersResult QuaternionControlNode::OnParameters(
    const std::vector<rclcpp::Parameter> &_parameters) {
  std::string text;
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "unhandled";
  result.successful = true;
  bool updated{false};
  for (const auto &parameter : _parameters) {
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gain, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(roll_weight, updated, text);
  }
  if (updated) {
    // put everything here that is required to handle updated parameters.
    SetControllerGains();
  }
  return result;
}

}  // namespace attitude_control
}  // namespace hippo_control
