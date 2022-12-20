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

  name = "mass";
  descr_text = "Effective mass including added mass (hydrodynamics) [m] = kg";
  descr = hippo_common::param_utils::Description(descr_text, false);
  {
    auto &param = trajectory_params_.mass;
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

  trajectory_params_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&SimpleTracker::OnSetTrajectoryParams, this, _1));
}

rcl_interfaces::msg::SetParametersResult SimpleTracker::OnSetTrajectoryParams(
    const std::vector<rclcpp::Parameter> &_parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.reason = "Unhandled";
  result.successful = true;
  for (const rclcpp::Parameter &parameter : _parameters) {
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "thrust_min", trajectory_params_.thrust_min)) {
      result.reason = "Set thrust_min.";
      // nothing else to do but to assign the value;
      continue;
    }
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "thrust_max", trajectory_params_.thrust_max)) {
      result.reason = "Set thrust_max.";
      // nothing else to do but to assign the value;
      continue;
    }
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "body_rate_max", trajectory_params_.body_rate_max)) {
      result.reason = "Set body_rate_max.";
      // nothing else to do but to assign the value;
      continue;
    }
    if (hippo_common::param_utils::AssignIfMatch(parameter, "mass",
                                                 trajectory_params_.mass)) {
      result.reason = "Set mass.";
      // nothing else to do but to assign the value;
      continue;
    }
    if (hippo_common::param_utils::AssignIfMatch(parameter, "damping",
                                                 trajectory_params_.damping)) {
      result.reason = "Set damping.";
      // nothing else to do but to assign the value;
      continue;
    }
    if (hippo_common::param_utils::AssignIfMatch(parameter, "t_final",
                                                 trajectory_params_.t_final)) {
      result.reason = "Set t_final.";
      // nothing else to do but to assign the value;
      continue;
    }
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "timestep_min", trajectory_params_.timestep_min)) {
      result.reason = "Set timestep_min.";
      // nothing else to do but to assign the value;
      continue;
    }
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "open_loop_threshold_time",
            trajectory_params_.open_loop_threshold_time)) {
      result.reason = "Set open_loop_threshold_time";
    }
    // if (hippo_common::param_utils::AssignIfMatch(
    //         parameter, "lookahead_time", trajectory_params_.lookahead_time)) {
    //   result.reason = "Set lookahead_time.";
    // }
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "min_wall_distance.x",
            trajectory_params_.min_wall_distance.x)) {
      result.reason = "Set min_wall_distance.x";
      continue;
    }
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "min_wall_distance.y",
            trajectory_params_.min_wall_distance.y)) {
      result.reason = "Set min_wall_distance.y";
      continue;
    }
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "min_wall_distance.z",
            trajectory_params_.min_wall_distance.z)) {
      result.reason = "Set min_wall_distance.z";
      continue;
    }
    if (hippo_common::param_utils::AssignIfMatch(
            parameter, "generation_update_period",
            trajectory_params_.generation_update_period)) {
      result.reason = "Set generation_update_period";
      continue;
    }
  }
  return result;
}

}  // namespace tracking
}  // namespace rapid_trajectories
