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

#include "joystick_component.hpp"

namespace remote_control {
namespace joystick {
void JoyStick::DeclareParams() {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  name = "gains.torque.x";
  RCLCPP_INFO(get_logger(), "Declaring '%s'.", name.c_str());
  descr_text = "Gain for joystick input";
  descr = hippo_common::param_utils::Description(descr_text);
  {
    auto &param = params_.gains.torque.x;
    param = declare_parameter(name, param, descr);
  }

  name = "gains.torque.y";
  RCLCPP_INFO(get_logger(), "Declaring '%s'.", name.c_str());
  descr_text = "Gain for joystick input";
  descr = hippo_common::param_utils::Description(descr_text);
  {
    auto &param = params_.gains.torque.y;
    param = declare_parameter(name, param, descr);
  }

  name = "gains.torque.z";
  RCLCPP_INFO(get_logger(), "Declaring '%s'.", name.c_str());
  descr_text = "Gain for joystick input";
  descr = hippo_common::param_utils::Description(descr_text);
  {
    auto &param = params_.gains.torque.z;
    param = declare_parameter(name, param, descr);
  }

  name = "gains.thrust.x";
  RCLCPP_INFO(get_logger(), "Declaring '%s'.", name.c_str());
  descr_text = "Gain for joystick input";
  descr = hippo_common::param_utils::Description(descr_text);
  {
    auto &param = params_.gains.thrust.x;
    param = declare_parameter(name, param, descr);
  }

  name = "gains.thrust.y";
  RCLCPP_INFO(get_logger(), "Declaring '%s'.", name.c_str());
  descr_text = "Gain for joystick input";
  descr = hippo_common::param_utils::Description(descr_text);
  {
    auto &param = params_.gains.thrust.y;
    param = declare_parameter(name, param, descr);
  }

  name = "gains.thrust.z";
  RCLCPP_INFO(get_logger(), "Declaring '%s'.", name.c_str());
  descr_text = "Gain for joystick input";
  descr = hippo_common::param_utils::Description(descr_text);
  {
    auto &param = params_.gains.thrust.z;
    param = declare_parameter(name, param, descr);
  }

  gain_params_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&JoyStick::OnGainParams, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult JoyStick::OnGainParams(
    const std::vector<rclcpp::Parameter> _parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "Unhandled";
  result.successful = true;
  std::string result_text;

  for (const rclcpp::Parameter &parameter : _parameters) {
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "gains.torque.x", params_.gains.torque.x, result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "gains.torque.y", params_.gains.torque.y,
                   result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "gains.torque.z", params_.gains.torque.z,
                   result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "gains.thrust.x", params_.gains.thrust.x,
                   result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "gains.thrust.y", params_.gains.thrust.y,
                   result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "gains.thrust.z", params_.gains.thrust.z,
                   result_text)) {
      result.reason = result_text;
    }
    RCLCPP_INFO_STREAM(get_logger(), result_text);
  }
  return result;
}
}  // namespace joystick
}  // namespace remote_control
