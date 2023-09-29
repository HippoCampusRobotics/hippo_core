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

#include "carrot_controller.hpp"

// assumes parameters are stored in this.params_ and names are identical.
#define DECLARE_PARAM_DESCR(x)                        \
  do {                                                \
    auto &param = params_.x;                          \
    param = declare_parameter(#x, param, descriptor); \
  } while (false)
#define DECLARE_PARAM(x)                  \
  do {                                    \
    auto &param = params_.x;              \
    param = declare_parameter(#x, param); \
  } while (false)

#define ASSIGN_SIMPLE_LOG(x, y)                                                \
  if (hippo_common::param_utils::AssignIfMatch(parameter, #x, params_.x, y)) { \
    RCLCPP_INFO_STREAM(get_logger(), y);                                       \
    result.reason = text;                                                      \
    params_.updated = true;                                                    \
    continue;                                                                  \
  }

namespace hippo_control {
namespace carrot_control {
void CarrotController::DeclareParams() {
  std::string name;
  std::string description;
  rcl_interfaces::msg::ParameterDescriptor descriptor;

  DECLARE_PARAM(look_ahead_distance);
  DECLARE_PARAM(path_file);
  DECLARE_PARAM(depth_gain);

  params_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&CarrotController::OnParams, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult CarrotController::OnParams(
    const std::vector<rclcpp::Parameter> &_parameters) {
  std::string text;
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "unhandled";
  result.successful = true;
  for (const auto &parameter : _parameters) {
    ASSIGN_SIMPLE_LOG(look_ahead_distance, text);
    ASSIGN_SIMPLE_LOG(depth_gain, text);
  }
  return result;
}
}  // namespace carrot_control
}  // namespace hippo_control
