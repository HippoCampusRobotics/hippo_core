#include "simple_tracker.hpp"
namespace rapid_trajectories {
namespace tracking {

void SimpleTracker::DeclareParams() {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  name = "thrust_min";
  descr_text = "Minimum allowed thrust [f_min] = N";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.thrust_min;
    param = declare_parameter(name, param, descr);
  }

  name = "thrust_min_at_target";
  descr_text = "Minimum thrust used in final state during sampling in [N].";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.thrust_min_at_target;
    param = declare_parameter(name, param, descr);
  }
  
  name = "thrust_max_at_target";
  descr_text = "Maximum thrust used in final state during sampling in [N].";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.thrust_max_at_target;
    param = declare_parameter(name, param, descr);
  }

  name = "thrust_max";
  descr_text = "Maximum allowed thrust [f_max] = N";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.thrust_max;
    param = declare_parameter(name, param, descr);
  }

  name = "body_rate_max";
  descr_text =
      "Maximum allowed rotational rate in body frame [omega_max] = rad/s";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.body_rate_max;
    param = declare_parameter(name, param, descr);
  }

  name = "lookahead_thrust";
  descr_text =
      "Lookahead factor k for thrust sampling. t_sample = t_now + k * "
      "dt_update";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.lookahead_thrust;
    param = declare_parameter(name, param, descr);
  }

  name = "lookahead_attitude";
  descr_text =
      "Lookahead factor k for attitude sampling. t_sample = t_now + k * "
      "dt_update";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.lookahead_attitude;
    param = declare_parameter(name, param, descr);
  }

  name = "lookahead_body_rate";
  descr_text =
      "Lookahead factor k for body rate sampling. t_sample = t_now + k * "
      "dt_update";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.lookahead_body_rate;
    param = declare_parameter(name, param, descr);
  }

  name = "enable_position_check";
  descr_text = "Enable/Disable position feasibility check.";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.enable_position_check;
    param = declare_parameter(name, param, descr);
  }

  name = "mass_rb";
  descr_text = "Rigid body mass [m] = kg";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.mass_rb;
    param = declare_parameter(name, param, descr);
  }

  name = "mass_added";
  descr_text = "Added mass due to hydrodynamics [m] = kg";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.mass_added;
    param = declare_parameter(name, param, descr);
  }

  name = "damping";
  descr_text =
      "Coeffient to compute the damping force F=v*damping [damping] = Ns/m";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.damping;
    param = declare_parameter(name, param, descr);
  }

  name = "t_final";
  descr_text = "Time horizon for the trajectory [t_final] = s";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.t_final;
    param = declare_parameter(name, param, descr);
  }

  name = "timestep_min";
  descr_text =
      "Minimum timestep for solving the feasibility. Should match the "
      "update/control loop frequency. [timestep_min] = s";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.timestep_min;
    param = declare_parameter(name, param, descr);
  }

  name = "continuous";
  descr_text = "Should the trajectory be recalculated in each time step?";
  descr = hippo_common::param_utils::Description(descr_text, true);
  {
    auto &param = trajectory_params_.continuous;
    param = declare_parameter(name, param, descr);
    if (trajectory_params_.continuous) {
      RCLCPP_INFO(get_logger(),
                  "Starting in continuous trajectory recalculation mode.");
    } else {
      RCLCPP_INFO(get_logger(),
                  "Starting in NON-continuous trajectory recalculation mode.");
    }
  }

  name = "use_attitude_control";
  descr_text = "Use either attitude control or rate control";
  descr = hippo_common::param_utils::Description(descr_text, true);
  {
    auto &param = trajectory_params_.continuous;
    param = declare_parameter(name, param, descr);
    if (trajectory_params_.use_attitude_control) {
      RCLCPP_INFO(get_logger(), "Attitude mode enabled.");
    } else {
      RCLCPP_INFO(get_logger(), "Rate mode enabled.");
    }
  }

  name = "generation_update_period";
  descr_text =
      "Time interval in which new trajectories get generated. "
      "[generation_update_period] = s";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.generation_update_period;
    param = declare_parameter(name, param, descr);
  }

  name = "open_loop_threshold_time";
  descr_text =
      "Time threshold under which the closed-loop continuous mode switches to "
      "open loop to avoid infeasibilities caused by the small timespan left "
      "for the trajectory.";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.open_loop_threshold_time;
    param = declare_parameter(name, param, descr);
  }

  // name = "lookahead_time";
  // descr_text = "Time to look ahead in current newly calculated trajectory.";
  // descr = hippo_common::param_utils::Description(descr_text, false);
  // {
  //   auto &param = trajectory_params_.lookahead_time;
  //   param = declare_parameter(name, param, descr);
  // }

  name = "min_wall_distance.x";
  descr_text =
      "Minimum required distance to walls in x-direction for a trajectory to "
      "be feasible.";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.min_wall_distance.x;
    param = declare_parameter(name, param, descr);
  }

  name = "min_wall_distance.y";
  descr_text =
      "Minimum required distance to walls in y-direction for a trajectory to "
      "be feasible.";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.min_wall_distance.y;
    param = declare_parameter(name, param, descr);
  }

  name = "min_wall_distance.z";
  descr_text =
      "Minimum required distance to walls in z-direction for a trajectory to "
      "be feasible.";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.min_wall_distance.z;
    param = declare_parameter(name, param, descr);
  }

  name = "gravity.x";
  descr_text = "Gravitational force in the respective axis direction.";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.gravity.x;
    param = declare_parameter(name, param, descr);
  }

  name = "gravity.y";
  descr_text = "Gravitational force in the respective axis direction.";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.gravity.y;
    param = declare_parameter(name, param, descr);
  }

  name = "gravity.z";
  descr_text = "Gravitational force in the respective axis direction.";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.gravity.z;
    param = declare_parameter(name, param, descr);
  }

  name = "homing_thrust";
  descr_text = "Homing thrust in [N].";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.homing_thrust;
    param = declare_parameter(name, param, descr);
  }

  name = "home_tolerance";
  descr_text =
      "Home is reached, if distance to home postion is less than the tolerance "
      "in [m].";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.home_tolerance;
    param = declare_parameter(name, param, descr);
  }

  name = "home_yaw";
  descr_text = "Orientation for the home position [rad]";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.home_yaw;
    param = declare_parameter(name, param, descr);
  }

  name = "home_axis_tolerance";
  descr_text =
      "Home orientation is reached if (axis_desired - axis_curent).norm < "
      "tolerance.";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.home_axis_tolerance;
    param = declare_parameter(name, param, descr);
  }

  name = "home_position.x";
  descr_text = "Axis component of home position.";
  descr = hippo_common::param_utils::Description(descr_text, true);
  {
    auto &param = trajectory_params_.home_position.x;
    param = declare_parameter(name, param, descr);
  }

  name = "home_position.y";
  descr_text = "Axis component of home position.";
  descr = hippo_common::param_utils::Description(descr_text, true);
  {
    auto &param = trajectory_params_.home_position.y;
    param = declare_parameter(name, param, descr);
  }

  name = "home_position.z";
  descr_text = "Axis component of home position.";
  descr = hippo_common::param_utils::Description(descr_text, true);
  {
    auto &param = trajectory_params_.home_position.z;
    param = declare_parameter(name, param, descr);
  }

  name = "target_p0.x";
  descr_text = "Intial target position.";
  descr = hippo_common::param_utils::Description(descr_text, true);
  {
    auto &param = trajectory_params_.target_p0.x;
    param = declare_parameter(name, param, descr);
  }

  name = "target_p0.y";
  descr_text = "Intial target position.";
  descr = hippo_common::param_utils::Description(descr_text, true);
  {
    auto &param = trajectory_params_.target_p0.y;
    param = declare_parameter(name, param, descr);
  }

  name = "target_p0.z";
  descr_text = "Intial target position.";
  descr = hippo_common::param_utils::Description(descr_text, true);
  {
    auto &param = trajectory_params_.target_p0.z;
    param = declare_parameter(name, param, descr);
  }

  name = "target_v0.x";
  descr_text = "Initial target velocity.";
  descr = hippo_common::param_utils::Description(descr_text, true);
  {
    auto &param = trajectory_params_.target_v0.x;
    param = declare_parameter(name, param, descr);
  }

  name = "target_v0.y";
  descr_text = "Initial target velocity.";
  descr = hippo_common::param_utils::Description(descr_text, true);
  {
    auto &param = trajectory_params_.target_v0.y;
    param = declare_parameter(name, param, descr);
  }

  name = "target_v0.z";
  descr_text = "Initial target velocity.";
  descr = hippo_common::param_utils::Description(descr_text, true);
  {
    auto &param = trajectory_params_.target_v0.z;
    param = declare_parameter(name, param, descr);
  }

  trajectory_params_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&SimpleTracker::OnSetTrajectoryParams, this, _1));
}

rcl_interfaces::msg::SetParametersResult SimpleTracker::OnSetTrajectoryParams(
    const std::vector<rclcpp::Parameter> &_parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "Unhandled";
  result.successful = true;
  std::string result_text;
  for (const rclcpp::Parameter &parameter : _parameters) {
    if (hippo_common::param_utils::AssignIfMatch(parameter, "thrust_min",
                                                 trajectory_params_.thrust_min,
                                                 result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "thrust_min_at_target",
                   trajectory_params_.thrust_min_at_target, result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "thrust_max_at_target",
                   trajectory_params_.thrust_max_at_target, result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "thrust_max", trajectory_params_.thrust_max,
                   result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "body_rate_max", trajectory_params_.body_rate_max,
                   result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "lookahead_thrust",
                   trajectory_params_.lookahead_thrust, result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "lookahead_attitude",
                   trajectory_params_.lookahead_attitude, result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "lookahead_body_rate",
                   trajectory_params_.lookahead_body_rate, result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "enable_position_check",
                   trajectory_params_.enable_position_check, result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "mass_rb", trajectory_params_.mass_rb,
                   result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "mass_added", trajectory_params_.mass_added,
                   result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "damping", trajectory_params_.damping,
                   result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "t_final", trajectory_params_.t_final,
                   result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "timestep_min", trajectory_params_.timestep_min,
                   result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "open_loop_threshold_time",
                   trajectory_params_.open_loop_threshold_time, result_text)) {
      result.reason = "Set open_loop_threshold_time";
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "homing_thrust", trajectory_params_.homing_thrust,
                   result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "home_yaw", trajectory_params_.home_yaw,
                   result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "home_axis_tolerance",
                   trajectory_params_.home_axis_tolerance, result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "min_wall_distance.x",
                   trajectory_params_.min_wall_distance.x, result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "min_wall_distance.y",
                   trajectory_params_.min_wall_distance.y, result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "min_wall_distance.z",
                   trajectory_params_.min_wall_distance.z, result_text)) {
      result.reason = result_text;
    } else if (hippo_common::param_utils::AssignIfMatch(
                   parameter, "generation_update_period",
                   trajectory_params_.generation_update_period, result_text)) {
      result.reason = result_text;
    }
    RCLCPP_INFO_STREAM(get_logger(), result_text);
  }
  return result;
}

}  // namespace tracking
}  // namespace rapid_trajectories
