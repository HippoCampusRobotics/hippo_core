#include "actuator_mixer_bluerov_node.hpp"

namespace hippo_control {
namespace mixer_bluerov {
void ActuatorMixerNode::DeclareParams() {
  size_t matrix_size;
  matrix_size =
      mixer_bluerov::kOutputChannels * mixer_bluerov::InputChannels::kCount;

  std::string name;
  name = "geometry.alpha_f";
  double alpha_f = declare_parameter<double>(
      name,
      hippo_common::param_utils::Description("Thruster angle front", true));
  name = "geometry.alpha_r";
  double alpha_r = declare_parameter<double>(
      name,
      hippo_common::param_utils::Description("Thruster angle rear", true));
  name = "geometry.l_hf";
  double l_hf = declare_parameter<double>(
      name, hippo_common::param_utils::Description(
                "Thruster lever horizontal front", true));
  name = "geometry.l_hr";
  double l_hr = declare_parameter<double>(
      name, hippo_common::param_utils::Description(
                "Thruster lever horizontal rear", true));
  name = "geometry.l_vx";
  double l_vx =
      declare_parameter<double>(name, hippo_common::param_utils::Description(
                                          "Thruster lever vertical x", true));
  name = "geometry.l_vy";
  double l_vy =
      declare_parameter<double>(name, hippo_common::param_utils::Description(
                                          "Thruster lever vertical y", true));
  mixer_.InitializeMixerMatrix(alpha_f, alpha_r, l_hf, l_hr, l_vx, l_vy);

  name = "input_limits";
  auto limit_matrix = declare_parameter<std::vector<double>>(
      name, hippo_common::param_utils::Description("Input Limits", true));
  matrix_size =
      mixer_bluerov::kOutputChannels * mixer_bluerov::InputChannels::kCount;
  if (limit_matrix.size() != matrix_size) {
    throw std::runtime_error(
        "Invalid size of input_limits parameter. Expected " +
        std::to_string(matrix_size) + " but got " +
        std::to_string(limit_matrix.size()));
  }

  std::string descr;
  rcl_interfaces::msg::ParameterDescriptor param;

  name = "output_scalings";
  descr =
      "Scaling factor for motor signals after normalization to [-1.0, 1.0].";
  param = hippo_common::param_utils::Description(descr);
  auto output_scalings = declare_parameter<std::vector<double>>(name, param);
  if (output_scalings.size() != kOutputChannels) {
    throw std::runtime_error(
        "Invalid size for output_scalings parameter. Expected " +
        std::to_string(kOutputChannels) + " but got " +
        std::to_string(output_scalings.size()));
  }

  static constexpr int cols = mixer_bluerov::InputChannels::kCount;
  static constexpr int rows = mixer_bluerov::kOutputChannels;
  for (int i_out = 0; i_out < rows; ++i_out) {
    mixer_bluerov::Mapping mapping;
    for (int i_in = 0; i_in < cols; ++i_in) {
      mapping.input_limits[i_in] = limit_matrix[i_out * cols + i_in];
    }
    mapping.output_scaling = output_scalings[i_out];
    mixer_.SetMapping(i_out, mapping);
  }

  name = "zero_throttle_threshold";
  descr = "Thrust threshold until which zero output is sent.";
  param = hippo_common::param_utils::Description(descr);
  auto zero_thrust_threshold = declare_parameter<double>(name, param);
  mixer_.SetZeroThrustThreshold(zero_thrust_threshold);

  name = "compensate_deadzone";
  descr = "Decides if the deadzone should be actively compensated.";
  param = hippo_common::param_utils::Description(descr);
  auto compensate_deadzone = declare_parameter<bool>(name, param);
  mixer_.SetCompensateDeadZone(compensate_deadzone);

  std::unordered_map<int, std::string> prefixes;
  prefixes[mixer_bluerov::ThrustDirection::forward] = "forward.";
  prefixes[mixer_bluerov::ThrustDirection::backward] = "backward.";
  std::vector<int> idxs = {mixer_bluerov::ThrustDirection::forward,
                           mixer_bluerov::ThrustDirection::backward};
  for (auto i : idxs) {
    name = prefixes.at(i) + "constant_coefficient";
    descr = "Constant coefficient c of thrust function F(n) = ax^2 + bx + c.";
    param = hippo_common::param_utils::Description(descr);
    auto constant_coefficient = declare_parameter<double>(name, param);
    mixer_.SetConstantCoefficient(constant_coefficient, i);

    name = prefixes.at(i) + "linear_coefficient";
    descr = "Linear coefficient b of thrust function F(n) = ax^2 + bx + c.";
    param = hippo_common::param_utils::Description(descr);
    auto linear_coefficient = declare_parameter<double>(name, param);
    mixer_.SetLinearCoefficient(linear_coefficient, i);

    name = prefixes.at(i) + "quadratic_coefficient";
    descr = "Quadratic coefficient a of thrust function F(n) = ax^2 + bx + c.";
    param = hippo_common::param_utils::Description(descr);
    auto quadratic_coefficient = declare_parameter<double>(name, param);
    mixer_.SetQuadraticCoefficient(quadratic_coefficient, i);

    name = prefixes.at(i) + "minimum";
    descr = "Minimum value for returned thrust";
    param = hippo_common::param_utils::Description(descr);
    auto minimum = declare_parameter<double>(name, param);
    mixer_.SetMinimum(minimum, i);

    name = prefixes.at(i) + "deadzone_minimum";
    descr = "Minimum deadzone bound";
    param = hippo_common::param_utils::Description(descr);
    auto deadzone_minimum = declare_parameter<double>(name, param);
    mixer_.SetDeadzoneMinimum(deadzone_minimum, i);
  }

  name = "weighting_last_input";
  descr = "Weighting factor ";
  param = hippo_common::param_utils::Description(descr);
  auto weighting_factor = declare_parameter<double>(name, param);
  mixer_.SetWeightingLastInput(weighting_factor);

  name = "scaling_saturation_limit_up";
  descr = "Upper limit for nullspace scaling of thrusters";
  param = hippo_common::param_utils::Description(descr);
  auto scaling_saturation_limit_up = declare_parameter<double>(name, param);
  mixer_.SetScalingSaturationLimitUp(scaling_saturation_limit_up);

  name = "scaling_saturation_limit_low";
  descr = "Lower limit for nullspace scaling of thrusters";
  param = hippo_common::param_utils::Description(descr);
  auto scaling_saturation_limit_low = declare_parameter<double>(name, param);
  mixer_.SetScalingSaturationLimitLow(scaling_saturation_limit_low);

  name = "max_rotations_per_second";
  descr = "The thrusters maximum rotations per second used for normalization.";
  param = hippo_common::param_utils::Description(descr);
  auto max_rotations_per_second = declare_parameter<double>(name, param);
  mixer_.SetMaxRotationsPerSecond(max_rotations_per_second);
}

void ActuatorMixerNode::InitializeParamCallbacks() {
  thrust_param_cb_handle_ = this->add_on_set_parameters_callback(std::bind(
      &ActuatorMixerNode::OnThrustParams, this, std::placeholders::_1));
  weighting_cb_handle_ = this->add_on_set_parameters_callback(std::bind(
      &ActuatorMixerNode::OnWeightingFactor, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult ActuatorMixerNode::OnThrustParams(
    const std::vector<rclcpp::Parameter> &_parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "Undhandled";
  std::unordered_map<int, std::string> prefixes;
  prefixes[mixer_bluerov::ThrustDirection::forward] = "forward.";
  prefixes[mixer_bluerov::ThrustDirection::backward] = "backward.";
  std::vector<int> idxs = {mixer_bluerov::ThrustDirection::forward,
                           mixer_bluerov::ThrustDirection::backward};
  for (const rclcpp::Parameter &parameter : _parameters) {
    double tmp_double;
    bool found = false;
    for (auto i : idxs) {
      if (hippo_common::param_utils::AssignIfMatch(
              parameter, prefixes.at(i) + "linear_coefficient", tmp_double)) {
        mixer_.SetLinearCoefficient(tmp_double, i);
        RCLCPP_INFO(this->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(tmp_double))
                        .c_str());
        result.reason = "Set linear_coefficient.";
        found = true;
        break;
      }

      if (hippo_common::param_utils::AssignIfMatch(
              parameter, prefixes.at(i) + "quadratic_coefficient",
              tmp_double)) {
        mixer_.SetQuadraticCoefficient(tmp_double, i);
        RCLCPP_INFO(this->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(tmp_double))
                        .c_str());
        result.reason = "Set quadratic_coefficient.";
        found = true;
        break;
      }

      if (hippo_common::param_utils::AssignIfMatch(
              parameter, prefixes.at(i) + "constant_coefficient", tmp_double)) {
        mixer_.SetConstantCoefficient(tmp_double, i);
        RCLCPP_INFO(this->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(tmp_double))
                        .c_str());
        result.reason = "Set constant_coefficient.";
        found = true;
        break;
      }

      if (hippo_common::param_utils::AssignIfMatch(
              parameter, prefixes.at(i) + "minimum", tmp_double)) {
        mixer_.SetMinimum(tmp_double, i);
        RCLCPP_INFO(this->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(tmp_double))
                        .c_str());
        result.reason = "Set minimum value.";
        found = true;
        break;
      }

      if (hippo_common::param_utils::AssignIfMatch(
              parameter, prefixes.at(i) + "deadzone_minimum", tmp_double)) {
        mixer_.SetDeadzoneMinimum(tmp_double, i);
        RCLCPP_INFO(this->get_logger(), "%s",
                    ("Changed parameter " + parameter.get_name() + " to " +
                     std::to_string(tmp_double))
                        .c_str());
        result.reason = "Set deadzone minimum value.";
        found = true;
        break;
      }
    }
    if (found) {
      continue;
    }

    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "zero_thrust_threshold", tmp_double)) {
      mixer_.SetZeroThrustThreshold(tmp_double);
      RCLCPP_INFO(this->get_logger(), "%s",
                  ("Changed parameter " + parameter.get_name() + " to " +
                   std::to_string(tmp_double))
                      .c_str());
      result.reason = "Set zero_thrust_threshold";
      continue;
    }
  }
  return result;
}

rcl_interfaces::msg::SetParametersResult ActuatorMixerNode::OnWeightingFactor(
    const std::vector<rclcpp::Parameter> &_parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "Undhandled";
  for (const rclcpp::Parameter &parameter : _parameters) {
    double tmp_double;
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "weighting_last_input", tmp_double)) {
      mixer_.SetWeightingLastInput(tmp_double);
      RCLCPP_INFO(this->get_logger(), "%s",
                  ("Changed parameter " + parameter.get_name() + " to " +
                   std::to_string(tmp_double))
                      .c_str());
      result.reason = "Set Weighting Parameter for last Input.";
      break;
    }
  }
  return result;
}

}  // namespace mixer_bluerov
}  // namespace hippo_control
