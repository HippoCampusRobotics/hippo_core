#include <chrono>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <hippo_common/param_utils.hpp>
#include <hippo_msgs/msg/actuator_setpoint.hpp>
#include <hippo_msgs/msg/attitude_target.hpp>
#include <hippo_msgs/msg/rates_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rapid_trajectories/rviz_helper.hpp>
#include <rapid_trajectories/trajectory_generator/generator.hpp>
#include <rapid_trajectories_msgs/msg/current_state_debug.hpp>
#include <rapid_trajectories_msgs/msg/target_state.hpp>
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

class SimpleTracker : public rclcpp::Node {
 public:
  explicit SimpleTracker(rclcpp::NodeOptions const &_options);

 private:
  struct TrajectorySettings {
    double thrust_min{0.0};
    double thrust_max{16.0};
    double body_rate_max{3.0};
    double mass{2.6};
    double damping{5.4};
    double t_final{10.0};
    double timestep_min{0.004};
    bool continuous{false};
    double generation_update_period{0.5};
    double open_loop_threshold_time{0.5};
    struct WallDistance {
      double x{0.3};
      double y{0.6};
      double z{0.5};
    } min_wall_distance;
    struct Gravity {
      double x{0.0};
      double y{0.0};
      double z{-9.81};
    } gravity;
  } trajectory_params_;

 private:
  inline double TimeOnTrajectory(const rclcpp::Time &_t_now) {
    return (_t_now - t_start_current_trajectory_).nanoseconds() * 1e-9;
  }
  inline double TimeOnSectionLeft(const rclcpp::Time &_t_now) {
    return (t_final_section_ - _t_now).nanoseconds() * 1e-9;
  }
  void InitPublishers();
  void InitSubscribers();
  void DeclareParams();
  void Update();
  bool UpdateMovingTarget(double dt, const rclcpp::Time &_t_now);
  bool UpdateTrajectories(double _duration, const rclcpp::Time &_t_now);
  void PublishControlInput(double _t_trajectory, const rclcpp::Time &_t_now);
  void PublishCurrentStateDebug(double _t_trajectory,
                                const rclcpp::Time &_t_now);
  void OnOdometry(const Odometry::SharedPtr _msg);
  void OnTarget(const TargetState::SharedPtr _msg);
  bool ShouldGenerateNewTrajectory(const rclcpp::Time &t_now);
  bool SectionFinished(const rclcpp::Time &t_now);
  void SwitchToNextTarget();
  void SwitchToPreviousTarget();
  void OnLinearAcceleration(
      const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr _msg);
  Generator::StateFeasibilityResult CheckWallCollision(Generator &_trajectory);
  rcl_interfaces::msg::SetParametersResult OnSetTrajectoryParams(
      const std::vector<rclcpp::Parameter> &_parameters);
  void GenerateTrajectories(
      double _duration, std::vector<Eigen::Vector3d> &_target_positions,
      std::vector<Eigen::Vector3d> &_target_velocities,
      std::vector<Eigen::Vector3d> &_target_accelerations);
  void DeleteTrajectories() { generators_.clear(); };

 private:
  //////////////////////////////////////////////////////////////////////////////
  // publishers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<AttitudeTarget>::SharedPtr attitude_target_pub_;
  /// @brief Publisher for the current target trajectory. Rather a debugging
  /// topic.
  rclcpp::Publisher<TrajectoryStamped>::SharedPtr target_trajectory_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      target_pose_pub_;
  rclcpp::Publisher<hippo_msgs::msg::RatesTarget>::SharedPtr rates_target_pub_;
  rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr thrust_pub_;
  rclcpp::Publisher<rapid_trajectories_msgs::msg::CurrentStateDebug>::SharedPtr
      state_debug_pub_;

  //////////////////////////////////////////////////////////////////////////////
  // subscriptions
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Subscription to the vehicles state estimate.
  rclcpp::Subscription<Odometry>::SharedPtr state_sub_;
  /// @brief Subscription for the current target state setpoint.
  rclcpp::Subscription<TargetState>::SharedPtr target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      linear_acceleration_sub_;
  std::vector<Generator> generators_;
  Generator selected_trajectory_;
  Eigen::Vector3d position_{0.0, 0.0, 0.0};
  Eigen::Vector3d velocity_{0.0, 0.0, 0.0};
  Eigen::Vector3d acceleration_{0.0, 0.0, 0.0};
  Eigen::Quaterniond orientation_{1.0, 0.0, 0.0, 0.0};

  OnSetParametersCallbackHandle::SharedPtr trajectory_params_cb_handle_;

  rclcpp::TimerBase::SharedPtr update_timer_;

  rclcpp::Time t_start_section_;
  rclcpp::Time t_final_section_;
  rclcpp::Time t_start_current_trajectory_;
  rclcpp::Time t_final_current_trajecotry_;
  rclcpp::Time t_last_odometry_;
  bool section_finished_{true};
  std::vector<Eigen::Vector3d>::size_type target_index_;
  std::vector<Eigen::Vector3d> target_positions_;
  std::vector<Eigen::Vector3d> target_velocities_;
  std::vector<Eigen::Vector3d> target_accelerations_;

  double dt_odometry_average_;

  Eigen::Vector3d target_position_{0.0, 0.0, 0.0};
  bool use_position_{true};
  Eigen::Vector3d target_velocity_{1.0, 0.0, 0.0};
  bool use_velocity_{true};
  Eigen::Vector3d target_acceleration_{0.0, 0.0, 0.0};
  bool use_acceleration_{true};
  std::shared_ptr<RvizHelper> rviz_helper_;
};
}  // namespace tracking
}  // namespace rapid_trajectories
