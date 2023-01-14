#include "simple_tracker.hpp"

#include <hippo_common/convert.hpp>
#include <hippo_common/tf2_utils.hpp>

namespace rapid_trajectories {
namespace tracking {

static constexpr int kUpdatePeriodMs = 20;

SimpleTracker::SimpleTracker(rclcpp::NodeOptions const &_options)
    : Node("single_tracker", _options),
      trajectory_(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                  Eigen::Vector3d::Zero(),
                  Eigen::Vector3d{trajectory_params_.gravity.x,
                                  trajectory_params_.gravity.y,
                                  trajectory_params_.gravity.z},
                  trajectory_params_.mass_rb, trajectory_params_.mass_added,
                  trajectory_params_.damping,
                  Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
      t_last_odometry_(now()),
      dt_odometry_average_(trajectory_params_.timestep_min),
      target_(Eigen::Vector3d{trajectory_params_.target_p0.x,
                              trajectory_params_.target_p0.y,
                              trajectory_params_.target_p0.z},
              Eigen::Vector3d{trajectory_params_.target_v0.x,
                              trajectory_params_.target_v0.y,
                              trajectory_params_.target_v0.z},
              Eigen::Vector3d::Zero(),
              Eigen::Quaterniond{
                  Eigen::AngleAxis{3.14 / 2.0, Eigen::Vector3d::UnitZ()}}) {
  RCLCPP_INFO(get_logger(), "Node created.");
  rclcpp::Node::SharedPtr rviz_node = create_sub_node("visualization");
  rviz_helper_ = std::make_shared<RvizHelper>(rviz_node);
  RCLCPP_INFO(get_logger(), "Init publishers.");
  InitPublishers();
  RCLCPP_INFO(get_logger(), "Init subscribers.");
  InitSubscribers();
  RCLCPP_INFO(get_logger(), "Declaring parameters.");
  DeclareParams();
  t_start_section_ = t_final_section_ = t_start_trajectory_ =
      t_final_trajecotry_ = t_last_odometry_ = now();
  target_ = TargetUniform(Eigen::Vector3d{trajectory_params_.target_p0.x,
                                          trajectory_params_.target_p0.y,
                                          trajectory_params_.target_p0.z},
                          Eigen::Vector3d{trajectory_params_.target_v0.x,
                                          trajectory_params_.target_v0.y,
                                          trajectory_params_.target_v0.z},
                          Eigen::Vector3d::Zero(),
                          Eigen::Quaterniond{Eigen::AngleAxis{
                              3.14 / 2.0, Eigen::Vector3d::UnitZ()}});
  RCLCPP_INFO(get_logger(), "Generating disc points.");
  GenerateDiscPoints();
  RCLCPP_INFO(get_logger(), "Generating normals.");
  GenerateNormals();

  // circular shape
  target_positions_ = {Eigen::Vector3d{0.5, 2.0, -1.0},
                       Eigen::Vector3d{1.5, 2.0, -1.0}};
  target_velocities_ = {Eigen::Vector3d{0.0, 0.5, 0.0},
                        Eigen::Vector3d{0.0, -0.25, 0.0}};
  target_accelerations_ = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
}
void SimpleTracker::InitPublishers() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

  topic = "attitude_target";
  attitude_target_pub_ = create_publisher<AttitudeTarget>(topic, qos);

  topic = "~/target_trajectory";
  target_trajectory_pub_ = create_publisher<TrajectoryStamped>(topic, qos);

  topic = "~/desired_pose";
  desired_pose_pub_ =
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

  topic = "~/ring_pose";
  ring_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(topic, qos);

  topic = "~/final_pose";
  final_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(topic, qos);

  topic = "~/trajectory_result";
  trajectory_result_pub_ =
      create_publisher<rapid_trajectories_msgs::msg::TrajectoryResult>(topic,
                                                                       qos);

  topic = "~/section_counter";
  section_counter_pub_ =
      create_publisher<hippo_msgs::msg::Int64Stamped>(topic, qos);
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

bool SimpleTracker::ShouldGenerateNewTrajectories(const rclcpp::Time &_t_now) {
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
  double dt_since_start = trajectory_.TimeOnTrajectory(_t_now.nanoseconds());
  if (TimeOnSectionLeft(_t_now) < trajectory_params_.open_loop_threshold_time) {
    return false;
  }
  if ((dt_since_start >= trajectory_params_.generation_update_period)) {
    return true;
  }
  return false;
}

bool SimpleTracker::SectionFinished(const rclcpp::Time &_t_now) {
  section_finished_ = (_t_now >= t_final_section_) ||
                      !trajectory_.TimeLeft((uint64_t)_t_now.nanoseconds());
  return section_finished_;
}

void SimpleTracker::GenerateDiscPoints() {
  constexpr double r_max =
      Sampling::kPositionRadius * Sampling::kPositionRadius;
  auto r_square =
      Eigen::Vector<double, Sampling::kPositionRadialSteps>::LinSpaced(
          r_max / Sampling::kPositionRadialSteps, r_max);
  constexpr double phi_max = Sampling::kPositionAngleDeg / 180.0 * 3.14;
  auto phi = Eigen::Vector<double, Sampling::kPositionAngleSteps>::LinSpaced(
      phi_max / Sampling::kPositionAngleSteps, phi_max);
  for (int i = 0; i < Sampling::kPositionRadialSteps; ++i) {
    for (int j = 0; j < Sampling::kPositionAngleSteps; ++j) {
      auto &p = disc_points_.at(i * Sampling::kPositionAngleSteps + j);
      p.x() = 0.0;
      p.y() = sqrt(r_square[i]) * cos(phi[j]);
      p.z() = sqrt(r_square[i]) * sin(phi[j]);
    }
  }
}

void SimpleTracker::GenerateNormals() {
  constexpr double phi1_max = Sampling::kNormalFirstAngleDeg / 180.0 * 3.14;
  constexpr double phi2_max = Sampling::kNormalSecondAngleDeg / 180.0 * 3.14;
  constexpr double delta_phi1 = phi1_max / Sampling::kNormalFirstAxisSteps;
  constexpr double delta_phi2 = phi2_max / Sampling::kNormalSecondAxisSteps;
  auto phi1 = Eigen::Vector<double, Sampling::kNormalFirstAxisSteps>::LinSpaced(
      delta_phi1, phi1_max);
  auto phi2 =
      Eigen::Vector<double, Sampling::kNormalSecondAxisSteps>::LinSpaced(
          delta_phi2, phi2_max);
  for (int i_1 = 0; i_1 < Sampling::kNormalFirstAxisSteps; ++i_1) {
    for (int i_2 = 0; i_2 < Sampling::kNormalSecondAxisSteps; ++i_2) {
      const Eigen::Quaterniond q1{
          Eigen::AngleAxisd{phi1[i_1], Eigen::Vector3d::UnitZ()}};
      const Eigen::Quaterniond q2{
          Eigen::AngleAxisd{phi2[i_2], Eigen::Vector3d::UnitX()}};
      normals_[i_1 * Sampling::kNormalSecondAxisSteps + i_2] =
          q2 * q1 * Eigen::Vector3d::UnitX() * Sampling::kNormalLength;
    }
  }
}

/**
 * @brief
 *
 * @param _t When the target should be reached with current's section begining
 * as t=0.0
 */
void SimpleTracker::GenerateTargetPoints(const double _t) {
  assert(disc_points_.size() == target_points_.size());
  for (std::size_t i = 0; i < disc_points_.size(); ++i) {
    target_points_[i] =
        target_.Orientation(_t) * disc_points_[i] + target_.Position(_t);
  }
}

bool SimpleTracker::CheckFeasibility(Trajectory &_traj) {
  Trajectory::InputFeasibilityResult input_result;
  input_result = _traj.CheckInputFeasibility(
      trajectory_params_.thrust_min, trajectory_params_.thrust_max,
      trajectory_params_.body_rate_max, trajectory_params_.timestep_min);
  if (input_result != Trajectory::InputFeasible) {
    return false;
  }
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
  // for (size_t i = 0; i < n_walls; ++i) {
  //   Trajectory::StateFeasibilityResult result;
  //   result = _traj.CheckPositionFeasibility(boundary_points.at(i),
  //                                           boundary_normals.at(i));
  //   if (result != Trajectory::StateFeasibilityResult::StateFeasible) {
  //     return false;
  //   }
  // }
  return true;
}

bool SimpleTracker::SampleTrajectories(const rclcpp::Time &_t_now) {
  double cost = std::numeric_limits<double>::max();
  const double t_left = TimeOnSectionLeft(_t_now);
  const double t_section = TimeOnSection(_t_now);
  const double t_start = t_left / Sampling::kTimeSteps;
  const double t_stop = t_left;
  auto t_vec =
      Eigen::Vector<double, Sampling::kTimeSteps>::LinSpaced(t_start, t_stop);

  auto f_vec = Eigen::Vector<double, Sampling::kThrustSteps>::LinSpaced(
      trajectory_params_.thrust_min_at_target, trajectory_params_.thrust_max);
  for (int i_time = 0; i_time < Sampling::kTimeSteps; ++i_time) {
    GenerateTargetPoints(t_section + t_vec[i_time]);
    Eigen::Quaterniond q = target_.Orientation(t_section + t_vec[i_time]);
    Eigen::Vector3d p = q.inverse() * position_;
    Eigen::Vector3d v = q.inverse() * velocity_;
    Eigen::Vector3d a = q.inverse() * acceleration_;
    Trajectory traj(p, v, a, Eigen::Vector3d::Zero(),
                    trajectory_params_.mass_rb, trajectory_params_.mass_added,
                    trajectory_params_.damping, q);
    for (int i_thrust = 0; i_thrust < Sampling::kThrustSteps; ++i_thrust) {
      for (int i_normals = 0; i_normals < Sampling::kNormals; ++i_normals) {
        for (int i_points = 0; i_points < Sampling::kPositions; ++i_points) {
          traj.Reset();
          Eigen::Vector3d p_final =
              q.inverse() * target_points_[i_points] + normals_[i_normals];
          Eigen::Vector3d v_final = -1.0 * normals_[i_normals] *
                                    f_vec[i_thrust] /
                                    trajectory_params_.damping;
          Eigen::Vector3d a_final{Eigen::Vector3d::Zero()};
          traj.SetGoalPosition(p_final);
          traj.SetGoalVelocity(v_final);
          // TODO(lennartalff): check if unspecified final acceleration is
          // better
          traj.SetGoalAcceleration(a_final);
          traj.Generate(t_vec[i_time], _t_now.nanoseconds());
          if (traj.GetCost() >= cost) {
            continue;
          }
          if (CheckFeasibility(traj)) {
            // whoop, whoop! We've found a cheaper and even feasible solution.
            cost = traj.GetCost();
            trajectory_ = traj;
          }
        }
      }
    }
  }
  return cost < std::numeric_limits<decltype(cost)>::max();
}

bool SimpleTracker::TargetHome() {
  Eigen::Vector3d home_position{trajectory_params_.home_position.x,
                                trajectory_params_.home_position.y,
                                trajectory_params_.home_position.z};
  Eigen::Vector3d d_vec = (home_position - position_);
  Eigen::Quaterniond q_desired =
      hippo_common::tf2_utils::RotationBetweenNormalizedVectors(
          Eigen::Vector3d::UnitX(), d_vec.normalized());
  Eigen::Vector3d attitude =
      hippo_common::tf2_utils::QuaternionToEuler(q_desired);

  hippo_msgs::msg::AttitudeTarget msg;
  msg.header.stamp = now();
  msg.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(attitude, msg.attitude);
  msg.mask = msg.IGNORE_RATES;
  msg.thrust = 0.0;
  attitude_target_pub_->publish(msg);

  Eigen::Vector3d axis_current = orientation_ * Eigen::Vector3d::UnitX();
  Eigen::Vector3d axis_desired = q_desired * Eigen::Vector3d::UnitX();

  double error = (axis_current - axis_desired).norm();

  if (error < trajectory_params_.home_axis_tolerance) {
    return true;
  }
  return false;
}

bool SimpleTracker::MoveHome() {
  Eigen::Vector3d home_position{trajectory_params_.home_position.x,
                                trajectory_params_.home_position.y,
                                trajectory_params_.home_position.z};
  Eigen::Vector3d d_vec = (home_position - position_);
  double xz_dist_square = d_vec.x() * d_vec.x() + d_vec.y() * d_vec.y();
  if (xz_dist_square <= trajectory_params_.home_tolerance) {
    // we reached the home position
    OrientateHome();
    return true;
  }
  Eigen::Quaterniond orientation =
      hippo_common::tf2_utils::RotationBetweenNormalizedVectors(
          Eigen::Vector3d::UnitX(), d_vec.normalized());
  Eigen::Vector3d attitude =
      hippo_common::tf2_utils::QuaternionToEuler(orientation);

  hippo_msgs::msg::AttitudeTarget msg;
  msg.header.stamp = now();
  msg.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(attitude, msg.attitude);
  msg.mask = msg.IGNORE_RATES;
  msg.thrust = trajectory_params_.homing_thrust;
  attitude_target_pub_->publish(msg);
  // obviously we have not reached the home position yet.
  return false;
}

bool SimpleTracker::OrientateHome() {
  static int counter = 0;
  hippo_msgs::msg::AttitudeTarget msg;
  msg.header.stamp = now();
  msg.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  msg.mask = msg.IGNORE_RATES;
  msg.thrust = 0.0;
  msg.attitude.x = 0.0;
  msg.attitude.y = 0.0;
  msg.attitude.z = trajectory_params_.home_yaw;
  attitude_target_pub_->publish(msg);

  Eigen::Vector3d axis_current = orientation_ * Eigen::Vector3d::UnitX();
  Eigen::Quaterniond q_desired = hippo_common::tf2_utils::EulerToQuaternion(
      0.0, 0.0, trajectory_params_.home_yaw);
  Eigen::Vector3d axis_desired = q_desired * Eigen::Vector3d::UnitX();

  double error = (axis_current - axis_desired).norm();

  if (error < trajectory_params_.home_axis_tolerance) {
    counter++;
    if (counter > 20) {
      counter = 0;
      return true;
    }
  }
  return false;
}

bool SimpleTracker::RunTrajectory(const rclcpp::Time &_t_now) {
  static double dt_avg = 0.0;
  if (initial_sampling_) {
    RCLCPP_INFO(get_logger(), "Computation time [ms]: %lf", dt_avg);
    acceleration_ = Eigen::Vector3d::Zero();
    velocity_ = Eigen::AngleAxis{trajectory_params_.home_yaw,
                                 Eigen::Vector3d::UnitZ()} *
                Eigen::Vector3d::UnitX() * 0.05;
    if (!SampleTrajectories(_t_now)) {
      RCLCPP_FATAL(get_logger(),
                   "Failed to find feasibile solution for initial trajectory.");
      return false;
    }
    initial_sampling_ = false;
  } else if (ShouldGenerateNewTrajectories(_t_now)) {
    auto tic = std::chrono::high_resolution_clock::now();
    SampleTrajectories(_t_now);
    auto toc = std::chrono::high_resolution_clock::now();
    double dt = std::chrono::duration<double, std::milli>(toc - tic).count();
    dt_avg = 0.1 * dt + 0.9 * dt_avg;
  }
  if (SectionFinished(_t_now)) {
    PublishTrajectoryResult(_t_now);
    return true;
  }

  double t_traj = trajectory_.TimeOnTrajectory(_t_now.nanoseconds());
  if (trajectory_params_.use_attitude_control) {
    PublishAttitudeTarget(t_traj, _t_now);
  } else {
    PublishControlInput(t_traj, _t_now);
  }
  PublishTrajectory(_t_now);
  PublishVisualizationTopics(_t_now);
  PublishCurrentStateDebug(t_traj, _t_now);
  return false;
}

void SimpleTracker::PublishTrajectory(const rclcpp::Time &_t_now) {
  rapid_trajectories_msgs::msg::TrajectoryStamped msg;
  msg.header.stamp = _t_now;
  hippo_common::convert::EigenToRos(trajectory_.GetAlphas(),
                                    msg.trajectory.alpha);
  hippo_common::convert::EigenToRos(trajectory_.GetBetas(),
                                    msg.trajectory.beta);
  hippo_common::convert::EigenToRos(trajectory_.GetGammas(),
                                    msg.trajectory.gamma);
  msg.trajectory.mass_rb = trajectory_.GetMassRigidBody();
  msg.trajectory.mass_added = trajectory_.GetMassAdded();
  msg.trajectory.damping = trajectory_.GetDamping();
  msg.trajectory.duration = trajectory_.GetFinalTime();
  msg.trajectory.t_start_abs_ns = trajectory_.GetStartTimeNs();
  hippo_common::convert::EigenToRos(trajectory_.GetPosition(0.0),
                                    msg.trajectory.p0);
  hippo_common::convert::EigenToRos(trajectory_.GetVelocity(0.0),
                                    msg.trajectory.v0);
  hippo_common::convert::EigenToRos(trajectory_.GetAcceleration(0.0),
                                    msg.trajectory.a0);
  hippo_common::convert::EigenToRos(trajectory_.GetRotation(),
                                    msg.trajectory.rotation);
  target_trajectory_pub_->publish(msg);
}

void SimpleTracker::PublishTrajectoryResult(const rclcpp::Time &_t_now) {
  rapid_trajectories_msgs::msg::TrajectoryResult msg;
  msg.header.stamp = _t_now;
  double t_final = trajectory_.GetFinalTime();
  double t_ring = (_t_now - t_start_section_).nanoseconds() * 1e-9;
  Eigen::Vector3d tmp;
  tmp = trajectory_.ToWorld(trajectory_.GetPosition(t_final));
  hippo_common::convert::EigenToRos(tmp, msg.state_desired.position);
  tmp = trajectory_.ToWorld(trajectory_.GetVelocity(t_final));
  hippo_common::convert::EigenToRos(tmp, msg.state_desired.velocity);
  tmp = trajectory_.ToWorld(trajectory_.GetAcceleration(t_final));
  hippo_common::convert::EigenToRos(tmp, msg.state_desired.acceleration);
  hippo_common::convert::EigenToRos(position_, msg.state_actual.position);
  hippo_common::convert::EigenToRos(velocity_, msg.state_actual.velocity);
  hippo_common::convert::EigenToRos(acceleration_,
                                    msg.state_actual.acceleration);
  hippo_common::convert::EigenToRos(orientation_, msg.orientation);
  msg.target_radius = Sampling::kPositionRadius;
  hippo_common::convert::EigenToRos(target_.Position(t_ring),
                                    msg.target_position);
  hippo_common::convert::EigenToRos(target_.Orientation(t_ring),
                                    msg.target_orientation);
  msg.average_computation_time = -1.0;
  msg.success = true;
  trajectory_result_pub_->publish(msg);
}

void SimpleTracker::PublishVisualizationTopics(const rclcpp::Time &_t_now) {
  rviz_helper_->PublishTrajectory(trajectory_);

  double t_traj = trajectory_.TimeOnTrajectory(_t_now.nanoseconds());
  double t_final = trajectory_.GetFinalTime();
  double t_ring =
      (trajectory_.GetFinalTimeAbsNs() - t_start_section_.nanoseconds()) * 1e-9;
  Eigen::Vector3d axis_now =
      trajectory_.ToWorld(trajectory_.GetNormalVector(t_traj));
  Eigen::Vector3d axis_final =
      trajectory_.ToWorld(trajectory_.GetNormalVector(t_final));

  Eigen::Quaterniond q_now =
      hippo_common::tf2_utils::RotationBetweenNormalizedVectors(
          Eigen::Vector3d::UnitX(), axis_now);
  Eigen::Quaterniond q_final =
      hippo_common::tf2_utils::RotationBetweenNormalizedVectors(
          Eigen::Vector3d::UnitX(), axis_final);

  Eigen::Vector3d p_now = trajectory_.ToWorld(trajectory_.GetPosition(t_traj));
  Eigen::Vector3d p_final =
      trajectory_.ToWorld(trajectory_.GetPosition(t_final));

  geometry_msgs::msg::PoseStamped desired_msg;
  desired_msg.header.stamp = _t_now;
  desired_msg.header.frame_id =
      hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(q_now, desired_msg.pose.orientation);
  hippo_common::convert::EigenToRos(p_now, desired_msg.pose.position);
  desired_pose_pub_->publish(desired_msg);

  geometry_msgs::msg::PoseStamped final_msg;
  final_msg.header = desired_msg.header;
  hippo_common::convert::EigenToRos(q_final, final_msg.pose.orientation);
  hippo_common::convert::EigenToRos(p_final, final_msg.pose.position);
  final_pose_pub_->publish(final_msg);

  geometry_msgs::msg::PoseStamped ring_msg;
  ring_msg.header = desired_msg.header;
  hippo_common::convert::EigenToRos(target_.Orientation(t_ring),
                                    ring_msg.pose.orientation);
  hippo_common::convert::EigenToRos(target_.Position(t_ring),
                                    ring_msg.pose.position);
  ring_pose_pub_->publish(ring_msg);
}

void SimpleTracker::Update() {
  rclcpp::Time t_now = now();

  switch (mission_state_) {
    case MissionState::TARGET_HOME:
      if (TargetHome()) {
        mission_state_ = MissionState::HOMING;
        RCLCPP_INFO(get_logger(), "Looking to home position.");
      }
      break;
    case MissionState::HOMING:
      if (MoveHome()) {
        mission_state_ = MissionState::ROTATING;
        RCLCPP_INFO(get_logger(), "Home position reached.");
      }
      break;
    case MissionState::ROTATING:
      if (OrientateHome()) {
        RCLCPP_INFO(get_logger(), "Home orientatin reached.");
        std::chrono::milliseconds duration(
            (int)(trajectory_params_.t_final * 1e3));
        t_start_section_ = t_now;
        t_final_section_ = t_now + rclcpp::Duration(duration);
        mission_state_ = MissionState::TRAJECTORY;
        initial_sampling_ = true;

        // publish section counter
        hippo_msgs::msg::Int64Stamped msg;
        msg.header.stamp = t_now;
        msg.data = section_counter_;
        section_counter_pub_->publish(msg);
      }
      break;
    case MissionState::TRAJECTORY:
      if (RunTrajectory(t_now)) {
        mission_state_ = MissionState::TARGET_HOME;
        RCLCPP_INFO(get_logger(), "Trajectory finished.");
        section_counter_++;
      }
      break;
    default:
      RCLCPP_FATAL(get_logger(), "Stuck in unknown mission state!");
      break;
  }
}

void SimpleTracker::PublishControlInput(double _t_trajectory,
                                        const rclcpp::Time &_t_now) {
  hippo_msgs::msg::RatesTarget rates_target_msg;
  rates_target_msg.header.stamp = _t_now;
  Eigen::Vector3d rates_world = trajectory_.ToWorld(
      trajectory_.GetOmega(_t_trajectory, dt_odometry_average_));
  Eigen::Vector3d rates_local = orientation_.inverse() * rates_world;
  rates_target_msg.roll = rates_local.x();
  rates_target_msg.pitch = rates_local.y();
  rates_target_msg.yaw = rates_local.z();
  rates_target_pub_->publish(rates_target_msg);

  hippo_msgs::msg::ActuatorSetpoint thrust_msg;
  thrust_msg.header.stamp = _t_now;
  thrust_msg.x = trajectory_.GetThrust(_t_trajectory + dt_odometry_average_);
  thrust_pub_->publish(thrust_msg);
}

void SimpleTracker::PublishAttitudeTarget(double _t_trajectory,
                                          const rclcpp::Time &_t_now) {
  Eigen::Vector3d trajectory_axis =
      trajectory_.ToWorld(trajectory_.GetNormalVector(_t_trajectory));
  Eigen::Quaterniond q =
      hippo_common::tf2_utils::RotationBetweenNormalizedVectors(
          Eigen::Vector3d::UnitX(), trajectory_axis);
  Eigen::Vector3d rpy = hippo_common::tf2_utils::QuaternionToEuler(q);

  AttitudeTarget attitude_target_msg;
  attitude_target_msg.header.frame_id =
      hippo_common::tf2_utils::frame_id::InertialFrame();
  attitude_target_msg.header.stamp = _t_now;
  attitude_target_msg.thrust =
      trajectory_.GetThrust(_t_trajectory + dt_odometry_average_);
  attitude_target_msg.mask = attitude_target_msg.IGNORE_RATES;
  hippo_common::convert::EigenToRos(rpy, attitude_target_msg.attitude);
  attitude_target_pub_->publish(attitude_target_msg);
}

void SimpleTracker::PublishCurrentStateDebug(double _t_trajectory,
                                             const rclcpp::Time &_t_now) {
  rapid_trajectories_msgs::msg::CurrentStateDebug msg;
  msg.header.stamp = _t_now;
  hippo_common::convert::EigenToRos(position_, msg.p_current);
  hippo_common::convert::EigenToRos(velocity_, msg.v_current);
  hippo_common::convert::EigenToRos(acceleration_, msg.a_current);
  hippo_common::convert::EigenToRos(
      trajectory_.ToWorld(trajectory_.GetPosition(_t_trajectory)),
      msg.p_desired);
  hippo_common::convert::EigenToRos(
      trajectory_.ToWorld(trajectory_.GetVelocity(_t_trajectory)),
      msg.v_desired);
  hippo_common::convert::EigenToRos(
      trajectory_.ToWorld(trajectory_.GetAcceleration(_t_trajectory)),
      msg.a_desired);
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
  auto t_now = now();
  double dt = (t_now - t_last_odometry_).nanoseconds() * 1e-9;
  if (dt < 1e-3) {
    RCLCPP_WARN(get_logger(), "Guarding against too small time interval: %lf",
                dt);
    return;
  }
  t_last_odometry_ = t_now;
  dt_odometry_average_ = 0.1 * dt + 0.9 * dt_odometry_average_;
  hippo_common::convert::RosToEigen(_msg->pose.pose.position, position_);
  hippo_common::convert::RosToEigen(_msg->twist.twist.linear, velocity_);
  hippo_common::convert::RosToEigen(_msg->pose.pose.orientation, orientation_);
  Update();
}

void SimpleTracker::OnTarget(const TargetState::SharedPtr _msg) {
  // TODO(lennartalff): implement
}

void SimpleTracker::OnLinearAcceleration(
    const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr _msg) {
  hippo_common::convert::RosToEigen(_msg->vector, acceleration_);
}

Trajectory::StateFeasibilityResult SimpleTracker::CheckWallCollision(
    Trajectory &_trajectory) {
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
    Trajectory::StateFeasibilityResult result;
    result = _trajectory.CheckPositionFeasibility(boundary_points.at(i),
                                                  boundary_normals.at(i));
    if (result == Trajectory::StateFeasibilityResult::StateInfeasible) {
      return Trajectory::StateFeasibilityResult::StateInfeasible;
    }
  }
  return Trajectory::StateFeasibilityResult::StateFeasible;
}

}  // namespace tracking
}  // namespace rapid_trajectories

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rapid_trajectories::tracking::SimpleTracker)
