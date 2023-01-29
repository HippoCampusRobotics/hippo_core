#include <hippo_common/param_utils.hpp>
#include <hippo_control/rate_control/rate_controller.hpp>

namespace hippo_control {
namespace rate_control {

void RateController::DeclareParams() {
  DeclareGainParams();
  DeclareIntegralLimitParams();

  std::string name;
  std::string description;
  rcl_interfaces::msg::ParameterDescriptor descriptor;

  name = "zero_integral_threshold";
  description =
      "Determines scaler for integral part: "
      "scaler=1-(body_rate/zero_integral_threshold)^2";
  descriptor = hippo_common::param_utils::Description(description, false);
  rcl_interfaces::msg::FloatingPointRange range;
  range.set__from_value(0.0).set__to_value(6.28).set__step(0.01);
  descriptor.floating_point_range = {range};
  {
    auto &param = params_.zero_integral_threshold;
    param = declare_parameter(name, param, descriptor);
  }
  params_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&RateController::OnParams, this, std::placeholders::_1));
}

void RateController::DeclareGainParams() {
  std::string name;
  std::string description;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  rcl_interfaces::msg::FloatingPointRange range;

  //////////////////////////////////////////////////////////////////////////////
  // roll gains
  //////////////////////////////////////////////////////////////////////////////
  name = "gains.roll.p";
  description = "Proportional gain for roll rate.";
  descriptor = hippo_common::param_utils::Description(description, false);
  range.set__from_value(0.01).set__to_value(100.0).set__step(0.1);
  descriptor.floating_point_range = {range};
  {
    auto &param = params_.gains.roll.p;
    param = declare_parameter(name, param, descriptor);
  }

  name = "gains.roll.i";
  description = "Integral gain for roll rate.";
  descriptor = hippo_common::param_utils::Description(description, false);
  range.set__from_value(0.0).set__to_value(20.0).set__step(0.1);
  descriptor.floating_point_range = {range};
  {
    auto &param = params_.gains.roll.i;
    param = declare_parameter(name, param, descriptor);
  }

  name = "gains.roll.d";
  description = "Derivative gain for roll rate.";
  descriptor = hippo_common::param_utils::Description(description, false);
  range.set__from_value(0.0).set__to_value(0.1).set__step(0.001);
  descriptor.floating_point_range = {range};
  {
    auto &param = params_.gains.roll.d;
    param = declare_parameter(name, param, descriptor);
  }

  name = "gains.roll.feed_forward";
  description = "Feed forward gain for roll rate.";
  descriptor = hippo_common::param_utils::Description(description, false);
  range.set__from_value(0.0).set__to_value(5.0).set__step(0.01);
  descriptor.floating_point_range = {range};
  {
    auto &param = params_.gains.roll.feed_forward;
    param = declare_parameter(name, param, descriptor);
  }

  //////////////////////////////////////////////////////////////////////////////
  // pitch gains
  //////////////////////////////////////////////////////////////////////////////
  name = "gains.pitch.p";
  description = "Proportional gain for pitch rate.";
  descriptor = hippo_common::param_utils::Description(description, false);
  range.set__from_value(0.01).set__to_value(100.0).set__step(0.1);
  descriptor.floating_point_range = {range};
  {
    auto &param = params_.gains.pitch.p;
    param = declare_parameter(name, param, descriptor);
  }

  name = "gains.pitch.i";
  description = "Integral gain for pitch rate.";
  descriptor = hippo_common::param_utils::Description(description, false);
  range.set__from_value(0.0).set__to_value(20.0).set__step(0.1);
  descriptor.floating_point_range = {range};
  {
    auto &param = params_.gains.pitch.i;
    param = declare_parameter(name, param, descriptor);
  }

  name = "gains.pitch.d";
  description = "Derivative gain for pitch rate.";
  descriptor = hippo_common::param_utils::Description(description, false);
  range.set__from_value(0.0).set__to_value(0.1).set__step(0.001);
  descriptor.floating_point_range = {range};
  {
    auto &param = params_.gains.pitch.d;
    param = declare_parameter(name, param, descriptor);
  }

  name = "gains.pitch.feed_forward";
  description = "Feed forward gain for pitch rate.";
  descriptor = hippo_common::param_utils::Description(description, false);
  range.set__from_value(0.0).set__to_value(5.0).set__step(0.01);
  descriptor.floating_point_range = {range};
  {
    auto &param = params_.gains.pitch.feed_forward;
    param = declare_parameter(name, param, descriptor);
  }

  //////////////////////////////////////////////////////////////////////////////
  // yaw gains
  //////////////////////////////////////////////////////////////////////////////
  name = "gains.yaw.p";
  description = "Proportional gain for yaw rate.";
  descriptor = hippo_common::param_utils::Description(description, false);
  range.set__from_value(0.01).set__to_value(100.0).set__step(0.1);
  descriptor.floating_point_range = {range};
  {
    auto &param = params_.gains.yaw.p;
    param = declare_parameter(name, param, descriptor);
  }

  name = "gains.yaw.i";
  description = "Integral gain for yaw rate.";
  descriptor = hippo_common::param_utils::Description(description, false);
  range.set__from_value(0.0).set__to_value(20.0).set__step(0.1);
  descriptor.floating_point_range = {range};
  {
    auto &param = params_.gains.yaw.i;
    param = declare_parameter(name, param, descriptor);
  }

  name = "gains.yaw.d";
  description = "Derivative gain for yaw rate.";
  descriptor = hippo_common::param_utils::Description(description, false);
  range.set__from_value(0.0).set__to_value(0.1).set__step(0.001);
  descriptor.floating_point_range = {range};
  {
    auto &param = params_.gains.yaw.d;
    param = declare_parameter(name, param, descriptor);
  }

  name = "gains.yaw.feed_forward";
  description = "Feed forward gain for yaw rate.";
  descriptor = hippo_common::param_utils::Description(description, false);
  range.set__from_value(0.0).set__to_value(5.0).set__step(0.01);
  descriptor.floating_point_range = {range};
  {
    auto &param = params_.gains.yaw.feed_forward;
    param = declare_parameter(name, param, descriptor);
  }
  gains_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&RateController::OnGainParams, this, std::placeholders::_1));
}
void RateController::DeclareIntegralLimitParams() {
  std::string name;
  std::string description;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  rcl_interfaces::msg::FloatingPointRange range;

  name = "integral_limits.roll";
  description = "Limit of the integral for the roll rate.";
  descriptor = hippo_common::param_utils::Description(description, false);
  range.set__from_value(0.0).set__to_value(50.0).set__step(0.1);
  descriptor.floating_point_range = {range};
  {
    auto &param = params_.integral_limits.roll;
    param = declare_parameter(name, param, descriptor);
  }

  name = "integral_limits.pitch";
  description = "Limit of the integral for the pitch rate.";
  descriptor = hippo_common::param_utils::Description(description, false);
  range.set__from_value(0.0).set__to_value(50.0).set__step(0.1);
  descriptor.floating_point_range = {range};
  {
    auto &param = params_.integral_limits.pitch;
    param = declare_parameter(name, param, descriptor);
  }

  name = "integral_limits.yaw";
  description = "Limit of the integral for the yaw rate.";
  descriptor = hippo_common::param_utils::Description(description, false);
  range.set__from_value(0.0).set__to_value(50.0).set__step(0.1);
  descriptor.floating_point_range = {range};
  {
    auto &param = params_.integral_limits.yaw;
    param = declare_parameter(name, param, descriptor);
  }
  integral_limits_cb_handle_ = add_on_set_parameters_callback(std::bind(
      &RateController::OnIntegralLimitParams, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult RateController::OnGainParams(
    const std::vector<rclcpp::Parameter> &_parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "Unhandled";
  result.successful = true;
  std::string name;
  std::string result_text;
  for (const rclcpp::Parameter &parameter : _parameters) {
    ////////////////////////////////////////////////////////////////////////////
    // roll gains
    ////////////////////////////////////////////////////////////////////////////
    name = "gains.roll.p";
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, name, params_.gains.roll.p, result_text)) {
      result.reason = result_text;
      RCLCPP_INFO_STREAM(get_logger(), result_text);
      continue;
    }

    name = "gains.roll.i";
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, name, params_.gains.roll.i, result_text)) {
      result.reason = result_text;
      RCLCPP_INFO_STREAM(get_logger(), result_text);
      continue;
    }

    name = "gains.roll.d";
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, name, params_.gains.roll.d, result_text)) {
      result.reason = result_text;
      RCLCPP_INFO_STREAM(get_logger(), result_text);
      continue;
    }

    name = "gains.roll.feed_forward";
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, name, params_.gains.roll.feed_forward, result_text)) {
      result.reason = result_text;
      RCLCPP_INFO_STREAM(get_logger(), result_text);
      continue;
    }

    ////////////////////////////////////////////////////////////////////////////
    // pitch gains
    ////////////////////////////////////////////////////////////////////////////
    name = "gains.pitch.p";
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, name, params_.gains.pitch.p, result_text)) {
      result.reason = result_text;
      RCLCPP_INFO_STREAM(get_logger(), result_text);
      continue;
    }

    name = "gains.pitch.i";
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, name, params_.gains.pitch.i, result_text)) {
      result.reason = result_text;
      RCLCPP_INFO_STREAM(get_logger(), result_text);
      continue;
    }

    name = "gains.pitch.d";
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, name, params_.gains.pitch.d, result_text)) {
      result.reason = result_text;
      RCLCPP_INFO_STREAM(get_logger(), result_text);
      continue;
    }

    name = "gains.pitch.feed_forward";
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, name, params_.gains.pitch.feed_forward, result_text)) {
      result.reason = result_text;
      RCLCPP_INFO_STREAM(get_logger(), result_text);
      continue;
    }

    ////////////////////////////////////////////////////////////////////////////
    // yaw gains
    ////////////////////////////////////////////////////////////////////////////
    name = "gains.yaw.p";
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, name, params_.gains.yaw.p, result_text)) {
      result.reason = result_text;
      RCLCPP_INFO_STREAM(get_logger(), result_text);
      continue;
    }

    name = "gains.yaw.i";
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, name, params_.gains.yaw.i, result_text)) {
      result.reason = result_text;
      RCLCPP_INFO_STREAM(get_logger(), result_text);
      continue;
    }

    name = "gains.yaw.d";
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, name, params_.gains.yaw.d, result_text)) {
      result.reason = result_text;
      RCLCPP_INFO_STREAM(get_logger(), result_text);
      continue;
    }

    name = "gains.yaw.feed_forward";
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, name, params_.gains.yaw.feed_forward, result_text)) {
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

rcl_interfaces::msg::SetParametersResult RateController::OnIntegralLimitParams(
    const std::vector<rclcpp::Parameter> &_parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "Unhandled";
  result.successful = true;
  std::string name, result_text;
  for (const rclcpp::Parameter &parameter : _parameters) {
    name = "integral_limits.roll";
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, name, params_.integral_limits.roll, result_text)) {
      result.reason = result_text;
      RCLCPP_INFO_STREAM(get_logger(), result_text);
      continue;
    }

    name = "integral_limits.pitch";
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, name, params_.integral_limits.pitch, result_text)) {
      result.reason = result_text;
      RCLCPP_INFO_STREAM(get_logger(), result_text);
      continue;
    }

    name = "integral_limits.yaw";
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, name, params_.integral_limits.yaw, result_text)) {
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

rcl_interfaces::msg::SetParametersResult RateController::OnParams(
    const std::vector<rclcpp::Parameter> &_parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "Unhandled";
  result.successful = true;
  std::string name, result_text;
  for (const rclcpp::Parameter &parameter : _parameters) {
    name = "zero_integral_threshold";
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, name, params_.zero_integral_threshold, result_text)) {
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
}  // namespace rate_control
}  // namespace hippo_control
