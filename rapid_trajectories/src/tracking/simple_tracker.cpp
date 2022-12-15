#include "rapid_trajectories/tracking/simple_tracker.hpp"

#include <hippo_common/convert.hpp>
#include <hippo_common/tf2_utils.hpp>

namespace rapid_trajectories {
namespace tracking {

static constexpr int kUpdatePeriodMs = 20;

SimpleTracker::SimpleTracker(rclcpp::NodeOptions const &_options)
    : Node("single_tracker", _options),
      selected_trajectory_(target_position_, target_velocity_,
                           target_acceleration_, trajectory_params_.mass,
                           trajectory_params_.damping) {
  RCLCPP_INFO(get_logger(), "Node created.");
  rclcpp::Node::SharedPtr rviz_node = create_sub_node("visualization");
  rviz_helper_ = std::make_shared<RvizHelper>(rviz_node);
  InitPublishers();
  InitSubscribers();
  DeclareParams();
  t_start_section_ = t_final_section_ = t_start_current_trajectory_ = now();
  t_last_odometry_ = now();

  // circular shape
  target_positions_ = {Eigen::Vector3d{0.5, 2.0, -1.0},
                       Eigen::Vector3d{1.5, 2.0, -1.0}};
  target_velocities_ = {Eigen::Vector3d{0.0, 0.5, 0.0},
                        Eigen::Vector3d{0.0, -0.25, 0.0}};
  target_accelerations_ = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

  // s-curve shape
  // target_positions_ = {
  //     Eigen::Vector3d{1.0, 3.0, -0.75}, Eigen::Vector3d{1.0, 2.0, -1.0},
  //     Eigen::Vector3d{1.0, 1.0, -0.75}, Eigen::Vector3d{1.0, 2.0, -0.5}};
  // target_velocities_ = {
  //     Eigen::Vector3d{0.3, 0.0, 0.0}, Eigen::Vector3d{-0.3, -0.5, 0.0},
  //     Eigen::Vector3d{0.3, 0.0, 0.0}, Eigen::Vector3d{-0.3, 0.5, 0.0}};

  // target_accelerations_ = {
  //     Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d{0.0, 0.0, 0.0},
  //     Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d{0.0, 0.0, 0.0}};

  update_timer_ = create_wall_timer(std::chrono::milliseconds(kUpdatePeriodMs),
                                    std::bind(&SimpleTracker::Update, this));
}
void SimpleTracker::InitPublishers() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

  topic = "attitude_target";
  attitude_target_pub_ = create_publisher<AttitudeTarget>(topic, qos);

  topic = "~/target_trajectory";
  target_trajectory_pub_ = create_publisher<TrajectoryStamped>(topic, qos);

  topic = "~/target_pose";
  target_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(topic, qos);

  topic = "rates_setpoint";
  rates_target_pub_ =
      create_publisher<hippo_msgs::msg::RatesTarget>(topic, qos);

  topic = "thrust_setpoint";
  thrust_pub_ = create_publisher<hippo_msgs::msg::ActuatorSetpoint>(
      topic, rclcpp::SensorDataQoS());

  topic = "~/state_debug";
  state_debug_pub_ =
      create_publisher<rapid_trajectories_msgs::msg::CurrentStateDebug>(
          topic, rclcpp::SensorDataQoS());
}

void SimpleTracker::InitSubscribers() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

  topic = "odometry";
  state_sub_ = create_subscription<Odometry>(
      topic, qos, std::bind(&SimpleTracker::OnOdometry, this, _1));

  topic = "~/setpoint";
  target_sub_ = create_subscription<TargetState>(
      topic, qos, std::bind(&SimpleTracker::OnTarget, this, _1));

  topic = "acceleration";
  linear_acceleration_sub_ =
      create_subscription<geometry_msgs::msg::Vector3Stamped>(
          topic, qos,
          std::bind(&SimpleTracker::OnLinearAcceleration, this, _1));
}

bool SimpleTracker::ShouldGenerateNewTrajectory(const rclcpp::Time &_t_now) {
  if (section_finished_) {
    return true;
  }
  if (!trajectory_params_.continuous) {
    if (_t_now > t_final_section_) {
      return true;
    }
    return false;
  }

  // we are in continuous mode.
  double dt_since_start = TimeOnTrajectory(_t_now);
  if (TimeOnSectionLeft(_t_now) < trajectory_params_.open_loop_threshold_time) {
    return false;
  }
  if ((dt_since_start > trajectory_params_.generation_update_period)) {
    return true;
  }
  return false;
}

bool SimpleTracker::SectionFinished(const rclcpp::Time &_t_now) {
  section_finished_ = _t_now > t_final_section_;
  return section_finished_;
}

void SimpleTracker::SwitchToNextTarget() {
  ++target_index_;
  if (target_index_ >= target_positions_.size()) {
    target_index_ = 0;
  }
  target_position_ = target_positions_.at(target_index_);
  target_velocity_ = target_velocities_.at(target_index_);
  target_acceleration_ = target_accelerations_.at(target_index_);
}

void SimpleTracker::SwitchToPreviousTarget() {
  if (target_index_ == 0) {
    target_index_ = target_positions_.size() - 1;
  } else {
    --target_index_;
  }
  target_position_ = target_positions_.at(target_index_);
  target_velocity_ = target_velocities_.at(target_index_);
  target_acceleration_ = target_accelerations_.at(target_index_);
}

void SimpleTracker::Update() {
  static int update_counter = 0;
  update_counter++;
  rclcpp::Time t_now = now();

  if (SectionFinished(t_now)) {
    section_finished_ = false;
    SwitchToNextTarget();
    if (!UpdateTrajectories(trajectory_params_.t_final, t_now)) {
      RCLCPP_WARN(get_logger(),
                  "Section finished, but no valid next trajectory.");
      SwitchToPreviousTarget();
      return;
    }
    t_start_section_ = t_now;
    std::chrono::milliseconds duration((int)(trajectory_params_.t_final * 1e3));
    t_final_section_ = t_start_section_ + rclcpp::Duration(duration);

    double t_trajectory = TimeOnTrajectory(t_now);
    PublishControlInput(t_trajectory, t_now);
    return;
  }

  if (ShouldGenerateNewTrajectory(t_now)) {
    UpdateTrajectories(TimeOnSectionLeft(t_now), t_now);
  }
  double t_trajectory = TimeOnTrajectory(t_now);
  PublishControlInput(t_trajectory, t_now);

  Eigen::Vector3d trajectory_axis =
      selected_trajectory_.GetNormalVector(t_trajectory);
  Eigen::Vector3d unit_x = Eigen::Vector3d::UnitX();
  Eigen::Quaterniond q =
      hippo_common::tf2_utils::RotationBetweenNormalizedVectors(
          unit_x, trajectory_axis);
  Eigen::Vector3d rpy = hippo_common::tf2_utils::QuaternionToEuler(q);

  AttitudeTarget attitude_target_msg;
  attitude_target_msg.header.frame_id = "map";
  attitude_target_msg.header.stamp = t_now;
  attitude_target_msg.thrust = selected_trajectory_.GetThrust(t_trajectory);
  attitude_target_msg.mask = attitude_target_msg.IGNORE_RATES;
  hippo_common::convert::EigenToRos(rpy, attitude_target_msg.attitude);
  attitude_target_pub_->publish(attitude_target_msg);

  rviz_helper_->PublishTrajectory(selected_trajectory_);
  rviz_helper_->PublishTarget(target_position_);
  rviz_helper_->PublishStart(position_);
  rviz_helper_->PublishHeading(selected_trajectory_.GetPosition(t_trajectory),
                               q);
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header = attitude_target_msg.header;
  hippo_common::convert::EigenToRos(q, pose_msg.pose.orientation);
  hippo_common::convert::EigenToRos(
      selected_trajectory_.GetPosition(t_trajectory), pose_msg.pose.position);
  target_pose_pub_->publish(pose_msg);
}

void SimpleTracker::PublishControlInput(double _t_trajectory,
                                        const rclcpp::Time &_t_now) {
  hippo_msgs::msg::RatesTarget rates_target_msg;
  rates_target_msg.header.stamp = _t_now;
  Eigen::Vector3d rates_world =
      selected_trajectory_.GetOmega(_t_trajectory, kUpdatePeriodMs * 1e-3);
  Eigen::Vector3d rates_local = orientation_.inverse() * rates_world;
  rates_target_msg.roll = rates_local.x();
  rates_target_msg.pitch = rates_local.y();
  rates_target_msg.yaw = rates_local.z();
  rates_target_pub_->publish(rates_target_msg);

  hippo_msgs::msg::ActuatorSetpoint thrust_msg;
  thrust_msg.header.stamp = _t_now;
  thrust_msg.x = selected_trajectory_.GetThrust(_t_trajectory);
  thrust_pub_->publish(thrust_msg);
}

bool SimpleTracker::UpdateTrajectories(double _duration,
                                       const rclcpp::Time &_t_now) {
  std::vector<Eigen::Vector3d> target_positions{target_position_};
  std::vector<Eigen::Vector3d> target_velocities{target_velocity_};
  std::vector<Eigen::Vector3d> target_accelerations{target_acceleration_};
  DeleteTrajectories();
  GenerateTrajectories(_duration, target_positions, target_velocities,
                       target_accelerations);
  if (!generators_.empty()) {
    selected_trajectory_ = generators_.at(0);
    t_start_current_trajectory_ = _t_now;
    return true;
  }
  return false;
}

void SimpleTracker::PublishCurrentStateDebug(double _t_trajectory,
                                             const rclcpp::Time &_t_now) {
  rapid_trajectories_msgs::msg::CurrentStateDebug msg;
  msg.header.stamp = _t_now;
  hippo_common::convert::EigenToRos(position_, msg.p_current);
  hippo_common::convert::EigenToRos(velocity_, msg.v_current);
  hippo_common::convert::EigenToRos(acceleration_, msg.a_current);
  hippo_common::convert::EigenToRos(
      selected_trajectory_.GetPosition(_t_trajectory), msg.p_desired);
  hippo_common::convert::EigenToRos(
      selected_trajectory_.GetVelocity(_t_trajectory), msg.v_desired);
  hippo_common::convert::EigenToRos(
      selected_trajectory_.GetAcceleration(_t_trajectory), msg.a_desired);
  state_debug_pub_->publish(msg);
}

void SimpleTracker::OnOdometry(const Odometry::SharedPtr _msg) {
  if (_msg->header.frame_id != "map") {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Odometry message's frame id is [%s], but only "
                         "[map] is handled. Ignoring...",
                         _msg->header.frame_id.c_str());
    return;
  }
  hippo_common::convert::RosToEigen(_msg->pose.pose.position, position_);
  hippo_common::convert::RosToEigen(_msg->twist.twist.linear, velocity_);
  hippo_common::convert::RosToEigen(_msg->pose.pose.orientation, orientation_);

  auto t_now = now();
  PublishCurrentStateDebug(TimeOnTrajectory(t_now), t_now);
}

void SimpleTracker::OnTarget(const TargetState::SharedPtr _msg) {
  // TODO(lennartalff): implement
}

void SimpleTracker::OnLinearAcceleration(
    const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr _msg) {
  hippo_common::convert::RosToEigen(_msg->vector, acceleration_);
}

RapidTrajectoryGenerator::StateFeasibilityResult
SimpleTracker::CheckWallCollision(RapidTrajectoryGenerator &_trajectory) {
  static constexpr size_t n_walls = 6;
  std::array<Eigen::Vector3d, n_walls> boundary_points = {
      Eigen::Vector3d{0 + trajectory_params_.min_wall_distance.x, 0.0, 0.0},
      Eigen::Vector3d{2.0 - trajectory_params_.min_wall_distance.x, 0.0, 0.0},
      Eigen::Vector3d{0.0, 0.0 + trajectory_params_.min_wall_distance.y, 0.0},
      Eigen::Vector3d{0.0, 4.0 - trajectory_params_.min_wall_distance.y, 0.0},
      Eigen::Vector3d{0.0, 0.0, -1.5 + trajectory_params_.min_wall_distance.z},
      Eigen::Vector3d{0.0, 0.0, 0.0 - trajectory_params_.min_wall_distance.y}};
  std::array<Eigen::Vector3d, n_walls> boundary_normals = {
      Eigen::Vector3d{1.0, 0.0, 0.0}, Eigen::Vector3d{-1.0, 0.0, 0.0},
      Eigen::Vector3d{0.0, 1.0, 0.0}, Eigen::Vector3d{0.0, -1.0, 0.0},
      Eigen::Vector3d{0.0, 0.0, 1.0}, Eigen::Vector3d{0.0, 0.0, -1.0}};

  // check for collision with all six walls. If a single check fails, the
  // trajectory is invalid.
  for (size_t i = 0; i < n_walls; ++i) {
    RapidTrajectoryGenerator::StateFeasibilityResult result;
    result = _trajectory.CheckPositionFeasibility(boundary_points.at(i),
                                                  boundary_normals.at(i));
    if (result ==
        RapidTrajectoryGenerator::StateFeasibilityResult::StateInfeasible) {
      return RapidTrajectoryGenerator::StateFeasibilityResult::StateInfeasible;
    }
  }
  return RapidTrajectoryGenerator::StateFeasibilityResult::StateFeasible;
}

void SimpleTracker::GenerateTrajectories(
    double _duration, std::vector<Eigen::Vector3d> &_target_positions,
    std::vector<Eigen::Vector3d> &_target_velocities,
    std::vector<Eigen::Vector3d> &_target_accelerations) {
  // make sure all target vectors have the same size
  assert(_target_positions.size() == _target_velocities.size() &&
         _target_positions.size() == _target_accelerations.size());

  RapidTrajectoryGenerator::InputFeasibilityResult input_feasibility;
  RapidTrajectoryGenerator::StateFeasibilityResult position_feasibility;

  for (unsigned int i = 0; i < _target_positions.size(); ++i) {
    RapidTrajectoryGenerator trajectory{position_, velocity_, acceleration_,
                                        trajectory_params_.mass,
                                        trajectory_params_.damping};
    trajectory.SetGoalPosition(_target_positions[i]);
    trajectory.SetGoalVelocity(_target_velocities[i]);
    trajectory.SetGoalAcceleration(_target_accelerations[i]);
    trajectory.Generate(_duration);

    // check for collision with any wall.
    // position_feasibility = CheckWallCollision(trajectory);
    // if (!(position_feasibility ==
    //       RapidTrajectoryGenerator::StateFeasibilityResult::StateFeasible)) {
    //   RCLCPP_WARN(get_logger(), "Generated trajectory collides with walls.");
    //   continue;
    // }

    // check if required vehicle inputs are valid.
    input_feasibility = trajectory.CheckInputFeasibility(
        trajectory_params_.thrust_min, trajectory_params_.thrust_max,
        trajectory_params_.body_rate_max, trajectory_params_.timestep_min);
    if (!(input_feasibility ==
          RapidTrajectoryGenerator::InputFeasibilityResult::InputFeasible)) {
      RCLCPP_INFO(get_logger(), "Infeasible: %d", input_feasibility);
      continue;
    }

    // if we come here, all checks have been passed
    generators_.push_back(trajectory);
  }
}
}  // namespace tracking
}  // namespace rapid_trajectories

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rapid_trajectories::tracking::SimpleTracker)
