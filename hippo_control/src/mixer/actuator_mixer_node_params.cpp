#include "actuator_mixer_node.hpp"

namespace hippo_control {
namespace mixer {
void ActuatorMixerNode::DeclareParams() {
  std::string name = "mixer_matrix";
  size_t matrix_size;
  auto mixer_matrix = declare_parameter<std::vector<double>>(
      name, hippo_common::param_utils::Description("Mixer Matrix", true));

  matrix_size = mixer::kOutputChannels * mixer::InputChannels::kCount;
  if (mixer_matrix.size() != matrix_size) {
    throw std::runtime_error("Invalid size of Mixer Matrix. Expected " +
                             std::to_string(matrix_size) + " but got " +
                             std::to_string(mixer_matrix.size()));
  }

  name = "input_limits";
  auto limit_matrix = declare_parameter<std::vector<double>>(
      name, hippo_common::param_utils::Description("Input Limits", true));
  matrix_size = mixer::kOutputChannels * mixer::InputChannels::kCount;
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

  static constexpr int cols = mixer::InputChannels::kCount;
  static constexpr int rows = mixer::kOutputChannels;
  for (int i_out = 0; i_out < rows; ++i_out) {
    mixer::Mapping mapping;
    for (int i_in = 0; i_in < cols; ++i_in) {
      mapping.input_scalings[i_in] = mixer_matrix[i_out * cols + i_in];
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

  name = "constant_coefficient";
  descr = "Constant coefficient c of thrust function F(n) = ax^2 + bx + c.";
  param = hippo_common::param_utils::Description(descr);
  auto constant_coefficient = declare_parameter<double>(name, param);
  mixer_.SetConstantCoefficient(constant_coefficient);

  name = "linear_coefficient";
  descr = "Linear coefficient b of thrust function F(n) = ax^2 + bx + c.";
  param = hippo_common::param_utils::Description(descr);
  auto linear_coefficient = declare_parameter<double>(name, param);
  mixer_.SetLinearCoefficient(linear_coefficient);

  name = "quadratic_coefficient";
  descr = "Quadratic coefficient a of thrust function F(n) = ax^2 + bx + c.";
  param = hippo_common::param_utils::Description(descr);
  auto quadratic_coefficient = declare_parameter<double>(name, param);
  mixer_.SetQuadraticCoefficient(quadratic_coefficient);

  name = "max_rotations_per_second";
  descr = "The thrusters maximum rotations per second used for normalization.";
  param = hippo_common::param_utils::Description(descr);
  auto max_rotations_per_second = declare_parameter<double>(name, param);
  mixer_.SetMaxRotationsPerSecond(max_rotations_per_second);
}

rcl_interfaces::msg::SetParametersResult ActuatorMixerNode::OnThrustParams(
    const std::vector<rclcpp::Parameter> &_parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "Undhandled";
  for (const rclcpp::Parameter &parameter : _parameters) {
    double tmp_double;
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "linear_coefficient", tmp_double)) {
      mixer_.SetLinearCoefficient(tmp_double);
      result.reason = "Set linear_coefficient.";
      continue;
    }

    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "quadratic_coefficient", tmp_double)) {
      mixer_.SetQuadraticCoefficient(tmp_double);
      result.reason = "Set quadratic_coefficient.";
      continue;
    }

    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "constant_coefficient", tmp_double)) {
      mixer_.SetConstantCoefficient(tmp_double);
      result.reason = "Set constant_coefficient.";
      continue;
    }

    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "zero_thrust_threshold", tmp_double)) {
      mixer_.SetZeroThrustThreshold(tmp_double);
      result.reason = "Set zero_thrust_threshold";
      continue;
    }
  }
  return result;
}
}  // namespace mixer
}  // namespace hippo_control
