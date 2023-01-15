#include <chrono>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <hippo_common/param_utils.hpp>
#include <hippo_msgs/msg/actuator_setpoint.hpp>
#include <hippo_msgs/msg/attitude_target.hpp>
#include <hippo_msgs/msg/int64_stamped.hpp>
#include <hippo_msgs/msg/rates_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rapid_trajectories/rviz_helper.hpp>
#include <rapid_trajectories/trajectory/generator.hpp>
#include <rapid_trajectories/trajectory/target.hpp>
#include <rapid_trajectories_msgs/msg/current_state_debug.hpp>
#include <rapid_trajectories_msgs/msg/target_state.hpp>
#include <rapid_trajectories_msgs/msg/trajectory_result.hpp>
#include <rapid_trajectories_msgs/msg/trajectory_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rapid_trajectories {
namespace tracking {

using namespace minimum_jerk;
using namespace hippo_msgs::msg;
using namespace nav_msgs::msg;
using namespace rapid_trajectories_msgs::msg;
using namespace std::chrono_literals;
using std::placeholders::_1;

namespace Sampling {
static constexpr int kTimeSteps = 10;
static constexpr int kThrustSteps = 5;
static constexpr int kPositionRadialSteps = 2;
static constexpr int kPositionAngleSteps = 5;
static constexpr int kPositions = kPositionRadialSteps * kPositionAngleSteps;
static constexpr int kNormalFirstAxisSteps = 5;
static constexpr int kNormalSecondAxisSteps = 5;
static constexpr int kNormals = kNormalFirstAxisSteps * kNormalSecondAxisSteps;
static constexpr int kSamples = kTimeSteps * kThrustSteps *
                                kPositionRadialSteps * kPositionAngleSteps *
                                kNormalFirstAxisSteps * kNormalSecondAxisSteps;
static constexpr double kNormalFirstAngleDeg = 5.0;
static constexpr double kNormalSecondAngleDeg = 360.0;
static constexpr double kNormalLength = 0.15;
static constexpr double kPositionAngleDeg = 360.0;
static constexpr double kPositionRadius = 0.15;
}  // namespace Sampling

enum class MissionState { TARGET_HOME, HOMING, ROTATING, TRAJECTORY };

class SimpleTracker : public rclcpp::Node {
 public:
  explicit SimpleTracker(rclcpp::NodeOptions const &_options);

 private:
  struct TrajectorySettings {
    double thrust_min{0.0};
    double thrust_min_at_target{0.0};
    double thrust_max_at_target{5.0};
    double thrust_max{16.0};
    double body_rate_max{3.0};
    double lookahead_thrust{0.0};
    double lookahead_attitude{2.0};
    double lookahead_body_rate{1.0};
    bool enable_position_check{true};
    double mass_rb{1.5};
    double mass_added{1.5};
    double damping{5.4};
    double t_final{10.0};
    double timestep_min{0.004};
    bool continuous{false};
    bool use_attitude_control{true};
    double generation_update_period{0.5};
    double open_loop_threshold_time{0.5};
    struct WallDistance {
      double x{0.2};
      double y{0.2};
      double z{0.1};
    } min_wall_distance;
    struct Gravity {
      double x{0.0};
      double y{0.0};
      double z{0.0};
    } gravity;
    struct HomePosition {
      double x{1.0};
      double y{3.0};
      double z{-0.8};
    } home_position;
    double homing_thrust{5.0};
    double home_tolerance{0.1};
    double home_yaw{3.14};
    double home_axis_tolerance{0.2};
    struct TargetPosition {
      double x{1.0};
      double y{0.8};
      double z{-0.3};
    } target_p0;
    struct TargetVelocity {
      double x{0.0};
      double y{0.0};
      double z{-0.1};
    } target_v0;
  } trajectory_params_;

 private:
  inline double TimeOnSectionLeft(const rclcpp::Time &_t_now) const {
    return (t_final_section_ - _t_now).nanoseconds() * 1e-9;
  }
  inline double TimeOnSection(const rclcpp::Time &_t_now) {
    return (_t_now - t_start_section_).nanoseconds() * 1e-9;
  }
  inline double SampleThrust(const Trajectory &_traj,
                             const rclcpp::Time &_t_now) const {
    return _traj.GetThrust(
        _t_now.nanoseconds() +
        (int64_t)(dt_odometry_average_ * trajectory_params_.lookahead_thrust *
                  1e9));
  }
  inline Eigen::Vector3d SampleDesiredAxis(const Trajectory &_traj,
                                           const rclcpp::Time &_t_now) const {
    return _traj.ToWorld(_traj.GetNormalVector(
        _t_now.nanoseconds() +
        (int64_t)(dt_odometry_average_ * trajectory_params_.lookahead_attitude *
                  1e9)));
  }
  inline Eigen::Vector3d SampleBodyRates(const Trajectory &_traj,
                                         const rclcpp::Time &_t_now) const {
    return _traj.ToWorld(_traj.GetOmega(
        _t_now.nanoseconds(),
        dt_odometry_average_ * trajectory_params_.lookahead_body_rate));
  }
  inline Eigen::Vector3d StartAxis() const {
    return Eigen::AngleAxis(trajectory_params_.home_yaw,
                            Eigen::Vector3d::UnitZ()) *
           Eigen::Vector3d::UnitX();
  }
  void InitPublishers();
  void InitSubscribers();
  void DeclareParams();
  void Update();
  void PublishControlInput(const rclcpp::Time &_t_now);
  void PublishAttitudeTarget(const rclcpp::Time &_t_now);
  void PublishCurrentStateDebug(double _t_trajectory,
                                const rclcpp::Time &_t_now);
  void PublishVisualizationTopics(const rclcpp::Time &_t_now);
  void PublishTrajectory(const rclcpp::Time &_t_now);
  void PublishTrajectoryResult(const rclcpp::Time &_t_now);
  void OnOdometry(const Odometry::SharedPtr _msg);
  void OnTarget(const TargetState::SharedPtr _msg);
  bool ShouldGenerateNewTrajectories(const rclcpp::Time &t_now);
  void GenerateDiscPoints();
  void GenerateTargetPoints(const double _t);
  void GenerateNormals();
  bool MoveHome();
  bool TargetHome();
  bool OrientateHome();
  bool RunTrajectory(const rclcpp::Time &_t_now);
  bool CheckFeasibility(Trajectory &_traj, const rclcpp::Time _t_now);
  bool SampleTrajectories(const rclcpp::Time &_t_now, bool initial = false);
  bool SectionFinished(const rclcpp::Time &t_now);
  void OnLinearAcceleration(
      const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr _msg);
  Trajectory::StateFeasibilityResult CheckWallCollision(
      Trajectory &_trajectory);
  Trajectory::StateFeasibilityResult CheckFunnelCollision(
      Trajectory &_trajectory, const rclcpp::Time &_t_now);
  rcl_interfaces::msg::SetParametersResult OnSetTrajectoryParams(
      const std::vector<rclcpp::Parameter> &_parameters);

 private:
  //////////////////////////////////////////////////////////////////////////////
  // publishers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<AttitudeTarget>::SharedPtr attitude_target_pub_;
  /// @brief Publisher for the current target trajectory. Rather a debugging
  /// topic.
  rclcpp::Publisher<TrajectoryStamped>::SharedPtr target_trajectory_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      desired_pose_pub_;
  rclcpp::Publisher<hippo_msgs::msg::RatesTarget>::SharedPtr rates_target_pub_;
  rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr thrust_pub_;
  rclcpp::Publisher<rapid_trajectories_msgs::msg::CurrentStateDebug>::SharedPtr
      state_debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ring_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr final_pose_pub_;
  rclcpp::Publisher<rapid_trajectories_msgs::msg::TrajectoryResult>::SharedPtr
      trajectory_result_pub_;
  rclcpp::Publisher<hippo_msgs::msg::Int64Stamped>::SharedPtr
      section_counter_pub_;

  //////////////////////////////////////////////////////////////////////////////
  // subscriptions
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Subscription to the vehicles state estimate.
  rclcpp::Subscription<Odometry>::SharedPtr state_sub_;
  /// @brief Subscription for the current target state setpoint.
  rclcpp::Subscription<TargetState>::SharedPtr target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      linear_acceleration_sub_;

  Trajectory trajectory_;
  Eigen::Vector3d position_{0.0, 0.0, 0.0};
  Eigen::Vector3d velocity_{0.0, 0.0, 0.0};
  Eigen::Vector3d acceleration_{0.0, 0.0, 0.0};
  Eigen::Quaterniond orientation_{1.0, 0.0, 0.0, 0.0};

  OnSetParametersCallbackHandle::SharedPtr trajectory_params_cb_handle_;

  rclcpp::TimerBase::SharedPtr update_timer_;

  rclcpp::Time t_start_section_;
  rclcpp::Time t_final_section_;
  rclcpp::Time t_start_trajectory_;
  rclcpp::Time t_final_trajecotry_;
  rclcpp::Time t_last_odometry_;
  bool section_finished_{true};
  std::array<Eigen::Vector3d, Sampling::kPositions> target_points_;
  std::array<Eigen::Vector3d, Sampling::kPositions> disc_points_;
  std::array<Eigen::Vector3d, Sampling::kNormals> normals_;
  std::vector<Eigen::Vector3d>::size_type target_index_;
  std::vector<Eigen::Vector3d> target_positions_;
  std::vector<Eigen::Vector3d> target_velocities_;
  std::vector<Eigen::Vector3d> target_accelerations_;

  int section_counter_{0};

  MissionState mission_state_{MissionState::TARGET_HOME};

  double dt_odometry_average_;
  bool initial_sampling_{true};

  TargetUniform target_;
  std::shared_ptr<RvizHelper> rviz_helper_;
};
}  // namespace tracking
}  // namespace rapid_trajectories
