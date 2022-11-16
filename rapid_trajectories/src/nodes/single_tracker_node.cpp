#include "single_tracker_node.hpp"

#include <hippo_common/tf2_utils.hpp>

namespace rapid_trajectories {
namespace single_tracking {

SingleTrackerNode::SingleTrackerNode()
    : Node("single_tracker"),
      selected_trajectory_(target_position_, target_velocity_,
                           target_acceleration_, trajectory_params_.mass,
                           trajectory_params_.damping) {
  RCLCPP_INFO(get_logger(), "Node created.");
  rclcpp::Node::SharedPtr rviz_node = create_sub_node("visualization");
  rviz_helper_ = std::make_shared<RvizHelper>(rviz_node);
  InitPublishers();
  InitSubscribers();
  DeclareParams();
  t_start_ = now();
  target_positions_ = {Eigen::Vector3d{0.5, 2.0, -1.0},
                       Eigen::Vector3d{1.5, 2.0, -1.0}};
  target_velocities_ = {Eigen::Vector3d{0.0, 0.5, 0.0},
                        Eigen::Vector3d{0.0, -0.5, 0.0}};
  target_accelerations_ = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

  update_timer_ =
      create_wall_timer(20ms, std::bind(&SingleTrackerNode::Update, this));
}
void SingleTrackerNode::InitPublishers() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

  topic = "attitude_target";
  attitude_target_pub_ = create_publisher<AttitudeTarget>(topic, qos);

  topic = "~/target_trajectory";
  target_trajectory_pub_ = create_publisher<TrajectoryStamped>(topic, qos);
}

void SingleTrackerNode::InitSubscribers() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

  topic = "odometry";
  state_sub_ = create_subscription<Odometry>(
      topic, qos, std::bind(&SingleTrackerNode::OnOdometry, this, _1));

  topic = "~/setpoint";
  target_sub_ = create_subscription<TargetState>(
      topic, qos, std::bind(&SingleTrackerNode::OnTarget, this, _1));
}

void SingleTrackerNode::DeclareParams() {
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

  trajectory_params_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&SingleTrackerNode::OnSetTrajectoryParams, this, _1));
}

void SingleTrackerNode::Update() {
  rclcpp::Time t_now = now();
  double t = (t_now - t_start_).nanoseconds() * 1e-9;
  if (t > selected_trajectory_.GetFinalTime()) {
    trajectory_finished_ = true;
  }
  // TODO(lennartalff): does trajectory_finished need to be a class member?
  if (trajectory_finished_) {
    trajectory_finished_ = false;
    target_index_ =
        (target_index_ + 1 >= target_positions_.size()) ? 0 : target_index_ + 1;
    RCLCPP_INFO_STREAM(get_logger(), "Switching to next target: "
                                         << std::to_string(target_index_));
    target_position_ = target_positions_.at(target_index_);
    target_velocity_ = target_velocities_.at(target_index_);
    target_acceleration_ = target_accelerations_.at(target_index_);
    UpdateTrajectories();

    try {
      selected_trajectory_ = generators_.at(0);
    } catch (const std::exception &e) {
      RCLCPP_WARN_STREAM(get_logger(), e.what() << '\n');
      trajectory_finished_ = true;
      return;
    }
    t_start_ = t_now;
  }
  t = (t_now - t_start_).nanoseconds() * 1e-9;

  Eigen::Vector3d trajectory_axis = selected_trajectory_.GetNormalVector(t);
  Eigen::Vector3d unit_x = Eigen::Vector3d::UnitX();
  // https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
  Eigen::Vector3d n = unit_x.cross(trajectory_axis);
  Eigen::Quaterniond q;
  q.x() = n.x();
  q.y() = n.y();
  q.z() = n.z();
  q.w() = 1 + trajectory_axis.dot(unit_x);
  q.normalize();
  Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);

  AttitudeTarget msg;
  msg.header.frame_id = "map";
  msg.header.stamp = t_now;
  msg.thrust = selected_trajectory_.GetThrust(t);
  msg.mask = msg.IGNORE_RATES;
  msg.attitude.x = rpy.x();
  msg.attitude.y = rpy.y();
  msg.attitude.z = rpy.z();
  attitude_target_pub_->publish(msg);

  rviz_helper_->PublishTrajectory(selected_trajectory_);
  rviz_helper_->PublishTarget(target_position_);
  rviz_helper_->PublishStart(position_);
  rviz_helper_->PublishHeading(selected_trajectory_.GetPosition(t), q);
}

void SingleTrackerNode::UpdateTrajectories() {
  std::vector<Eigen::Vector3d> target_positions{target_position_};
  std::vector<Eigen::Vector3d> target_velocities{target_velocity_};
  std::vector<Eigen::Vector3d> target_accelerations{target_acceleration_};
  DeleteTrajectories();
  GenerateTrajectories(trajectory_params_.t_final, target_positions,
                       target_velocities, target_accelerations);
}

void SingleTrackerNode::OnOdometry(const Odometry::SharedPtr _msg) {
  if (_msg->header.frame_id != "map") {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Odometry message's frame id is [%s], but only "
                         "[map] is handled. Ignoring...",
                         _msg->header.frame_id.c_str());
    return;
  }
  position_.x() = _msg->pose.pose.position.x;
  position_.y() = _msg->pose.pose.position.y;
  position_.z() = _msg->pose.pose.position.z;

  velocity_.x() = _msg->twist.twist.linear.x;
  velocity_.y() = _msg->twist.twist.linear.y;
  velocity_.z() = _msg->twist.twist.linear.z;
}

void SingleTrackerNode::OnTarget(const TargetState::SharedPtr _msg) {
  // TODO(lennartalff): implement
}

rcl_interfaces::msg::SetParametersResult
SingleTrackerNode::OnSetTrajectoryParams(
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
  }
  return result;
}

void SingleTrackerNode::GenerateTrajectories(
    double _duration, std::vector<Eigen::Vector3d> &_target_positions,
    std::vector<Eigen::Vector3d> &_target_velocities,
    std::vector<Eigen::Vector3d> &_target_accelerations) {
  // make sure all target vectors have the same size
  assert(_target_positions.size() == _target_velocities.size() &&
         _target_positions.size() == _target_accelerations.size());

  RapidTrajectoryGenerator::InputFeasibilityResult input_feasibility;

  for (unsigned int i = 0; i < _target_positions.size(); ++i) {
    RapidTrajectoryGenerator trajectory{
        position_, velocity_, Eigen::Vector3d::Zero(), trajectory_params_.mass,
        trajectory_params_.damping};
    trajectory.SetGoalPosition(_target_positions[i]);
    trajectory.SetGoalVelocity(_target_velocities[i]);
    trajectory.SetGoalAcceleration(_target_accelerations[i]);
    trajectory.Generate(_duration);
    input_feasibility = trajectory.CheckInputFeasibility(
        trajectory_params_.thrust_min, trajectory_params_.thrust_max,
        trajectory_params_.body_rate_max, trajectory_params_.timestep_min);
    if (input_feasibility ==
        RapidTrajectoryGenerator::InputFeasibilityResult::InputFeasible) {
      generators_.push_back(trajectory);
    } else {
      RCLCPP_WARN(get_logger(), "Infeasible reason: %d", input_feasibility);
    }
  }
}
}  // namespace single_tracking
}  // namespace rapid_trajectories

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<
               rapid_trajectories::single_tracking::SingleTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
