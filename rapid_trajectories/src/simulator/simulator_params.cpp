#include <hippo_common/param_utils.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include "simulator.hpp"
namespace rapid_trajectories {
namespace simulator {
void Simulator::DeclareParams() {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  std::string description;

  name = "damping";
  description = "Damping coefficient";
  descriptor = hippo_common::param_utils::Description(description, false);
  {
    auto &param = params_.damping;
    param = declare_parameter(name, param, descriptor);
  }

  name = "mass";
  description = "Mass of the vehicle";
  descriptor = hippo_common::param_utils::Description(description, false);
  {
    auto &param = params_.mass;
    param = declare_parameter(name, param, descriptor);
  }

  name = "timestep_ms";
  description = "Simulation time step. [timeste_ms] = ms";
  descriptor = hippo_common::param_utils::Description(description, false);
  {
    auto &param = params_.timestep_ms;
    param = declare_parameter(name, param, descriptor);
  }

  name = "speed_factor";
  description = "Factor by which the simulation is slowed down/sped up.";
  descriptor = hippo_common::param_utils::Description(description, false);
  {
    auto &param = params_.speed_factor;
    param = declare_parameter(name, param, descriptor);
  }

  params_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&Simulator::OnSetParams, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult Simulator::OnSetParams(
    const std::vector<rclcpp::Parameter> &_parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "Unhandled";
  result.successful = true;
  std::string text;

  for (const rclcpp::Parameter &parameter : _parameters) {
    if (hippo_common::param_utils::AssignIfMatch(parameter, "damping",
                                                 params_.damping, text)) {
      result.reason = text;
      RCLCPP_INFO_STREAM(get_logger(), text);
      continue;
    }

    if (hippo_common::param_utils::AssignIfMatch(parameter, "mass",
                                                 params_.mass, text)) {
      result.reason = text;
      RCLCPP_INFO_STREAM(get_logger(), text);
      continue;
    }

    if (hippo_common::param_utils::AssignIfMatch(parameter, "timestep_ms",
                                                 params_.timestep_ms, text)) {
      result.reason = text;
      RCLCPP_INFO_STREAM(get_logger(), text);
      continue;
    }

    if (hippo_common::param_utils::AssignIfMatch(parameter, "speed_factor",
                                                 params_.speed_factor, text)) {
      result.reason = text;
      RCLCPP_INFO_STREAM(get_logger(), text);
      continue;
    }
  }
  return result;
}

}  // namespace simulator
}  // namespace rapid_trajectories
