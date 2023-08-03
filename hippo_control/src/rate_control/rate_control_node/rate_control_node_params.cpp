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
#include "rate_control_node.hpp"

namespace hippo_control {
namespace rate_control {

void RateControlNode::DeclareParams() {
  DeclareGainParams();
  DeclareIntegralLimitParams();

  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(zero_integral_threshold);

  params_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&RateControlNode::OnParams, this, std::placeholders::_1));
}

void RateControlNode::DeclareGainParams() {
  //////////////////////////////////////////////////////////////////////////////
  // roll gains
  //////////////////////////////////////////////////////////////////////////////
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gains.roll.p);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gains.roll.i);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gains.roll.d);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gains.roll.feed_forward);

  //////////////////////////////////////////////////////////////////////////////
  // pitch gains
  //////////////////////////////////////////////////////////////////////////////
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gains.pitch.p);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gains.pitch.i);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gains.pitch.d);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gains.pitch.feed_forward);

  //////////////////////////////////////////////////////////////////////////////
  // yaw gains
  //////////////////////////////////////////////////////////////////////////////
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gains.yaw.p);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gains.yaw.i);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gains.yaw.d);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(gains.yaw.feed_forward);

  gains_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&RateControlNode::OnGainParams, this, std::placeholders::_1));
}
void RateControlNode::DeclareIntegralLimitParams() {
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(integral_limits.roll);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(integral_limits.pitch);
  HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(integral_limits.yaw);

  integral_limits_cb_handle_ = add_on_set_parameters_callback(std::bind(
      &RateControlNode::OnIntegralLimitParams, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult RateControlNode::OnGainParams(
    const std::vector<rclcpp::Parameter> &_parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "unhandled";
  result.successful = true;
  bool updated{false};
  std::string text;
  for (const rclcpp::Parameter &parameter : _parameters) {
    ////////////////////////////////////////////////////////////////////////////
    // roll gains
    ////////////////////////////////////////////////////////////////////////////
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gains.roll.p, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gains.roll.i, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gains.roll.d, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gains.roll.feed_forward, updated, text);

    ////////////////////////////////////////////////////////////////////////////
    // pitch gains
    ////////////////////////////////////////////////////////////////////////////
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gains.pitch.p, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gains.pitch.i, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gains.pitch.d, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gains.pitch.feed_forward, updated, text);

    ////////////////////////////////////////////////////////////////////////////
    // yaw gains
    ////////////////////////////////////////////////////////////////////////////
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gains.yaw.p, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gains.yaw.i, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gains.yaw.d, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(gains.yaw.feed_forward, updated, text);
  }
  if (updated) {
    UpdateAllControllerParams();
  }
  return result;
}

rcl_interfaces::msg::SetParametersResult RateControlNode::OnIntegralLimitParams(
    const std::vector<rclcpp::Parameter> &_parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "unhandled";
  result.successful = true;
  std::string text;
  bool updated{false};
  for (const rclcpp::Parameter &parameter : _parameters) {
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(integral_limits.roll, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(integral_limits.pitch, updated, text);
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(integral_limits.yaw, updated, text);
  }
  if (updated) {
    UpdateAllControllerParams();
  }
  return result;
}

rcl_interfaces::msg::SetParametersResult RateControlNode::OnParams(
    const std::vector<rclcpp::Parameter> &_parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "unhandled";
  result.successful = true;
  std::string text;
  bool updated{false};
  for (const rclcpp::Parameter &parameter : _parameters) {
    HIPPO_COMMON_ASSIGN_SIMPLE_LOG(zero_integral_threshold, updated, text);
  }
  if (updated) {
    controller_.SetZeroIntegralThreshold(params_.zero_integral_threshold);
  }
  return result;
}
}  // namespace rate_control
}  // namespace hippo_control
