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

#include "geometric_control_node.hpp"

namespace hippo_control {
namespace attitude_control {

void GeometricControlNode::DeclareParams() {
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gain.roll.p);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gain.roll.d);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gain.pitch.p);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gain.pitch.d);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gain.yaw.p);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gain.yaw.d);
  SetControllerGains();

  gain_params_cb_handle_ = add_on_set_parameters_callback(std::bind(
      &GeometricControlNode::OnGainParameters, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult GeometricControlNode::OnGainParameters(
    const std::vector<rclcpp::Parameter> &_parameters) {
  std::string text;
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "unhandled";
  result.successful = true;
  bool updated{false};
  for (const auto &parameter : _parameters) {
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gain.roll.p, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gain.roll.d, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gain.pitch.p, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gain.pitch.d, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gain.yaw.p, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gain.yaw.d, updated, text);
  }
  if (updated) {
    SetControllerGains();
  }
  return result;
}

}  // namespace attitude_control
}  // namespace hippo_control
