#include <chrono>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <hippo_common/param_utils.hpp>
#include <hippo_msgs/msg/attitude_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rapid_trajectories/rviz_helper.hpp>
#include <rapid_trajectories/trajectory_generator/generator.hpp>
#include <rapid_trajectories_msgs/msg/target_state.hpp>
#include <rapid_trajectories_msgs/msg/trajectory_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rapid_trajectories {
namespace single_tracking {

using namespace trajectory_generator;
using namespace hippo_msgs::msg;
using namespace nav_msgs::msg;
using namespace rapid_trajectories_msgs::msg;
using namespace std::chrono_literals;
using std::placeholders::_1;

class SingleTrackerNode : public rclcpp::Node {
 public:
  SingleTrackerNode();

 private:
  struct TrajectorySettings {
    double thrust_min{0.0};
    double thrust_max{16.0};
    double body_rate_max{3.0};
    double mass{2.6};
    double damping{5.4};
    double t_final{5.0};
    double timestep_min{0.02};
    bool continuous{false};
    double open_loop_threshold_time{0.5};
    double lookahead_time{0.5};
    struct WallDistance {
      double x{0.3};
      double y{0.6};
      double z{0.5};
    } min_wall_distance;
  } trajectory_params_;

 private:
  void InitPublishers();
  void InitSubscribers();
  void DeclareParams();
  void Update();
  void UpdateTrajectories(double _t_final);
  void OnOdometry(const Odometry::SharedPtr _msg);
  void OnTarget(const TargetState::SharedPtr _msg);
  RapidTrajectoryGenerator::StateFeasibilityResult CheckWallCollision(
      RapidTrajectoryGenerator &_trajectory);
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

  //////////////////////////////////////////////////////////////////////////////
  // subscriptions
  //////////////////////////////////////////////////////////////////////////////
  /// @brief Subscription to the vehicles state estimate.
  rclcpp::Subscription<Odometry>::SharedPtr state_sub_;
  /// @brief Subscription for the current target state setpoint.
  rclcpp::Subscription<TargetState>::SharedPtr target_sub_;
  std::vector<RapidTrajectoryGenerator> generators_;
  RapidTrajectoryGenerator selected_trajectory_;
  Eigen::Vector3d position_{0.0, 0.0, 0.0};
  Eigen::Vector3d velocity_{0.0, 0.0, 0.0};
  Eigen::Vector3d acceleration_{0.0, 0.0, 0.0};
  rclcpp::Time t_last_odometry_;

  OnSetParametersCallbackHandle::SharedPtr trajectory_params_cb_handle_;

  rclcpp::TimerBase::SharedPtr update_timer_;

  rclcpp::Time t_start_section_;
  rclcpp::Time t_final_section_;
  bool trajectory_finished_{true};
  std::vector<Eigen::Vector3d>::size_type target_index_;
  std::vector<Eigen::Vector3d> target_positions_;
  std::vector<Eigen::Vector3d> target_velocities_;
  std::vector<Eigen::Vector3d> target_accelerations_;

  Eigen::Vector3d target_position_{0.0, 0.0, 0.0};
  bool use_position_{true};
  Eigen::Vector3d target_velocity_{1.0, 0.0, 0.0};
  bool use_velocity_{true};
  Eigen::Vector3d target_acceleration_{0.0, 0.0, 0.0};
  bool use_acceleration_{true};
  std::shared_ptr<RvizHelper> rviz_helper_;
};
}  // namespace single_tracking
}  // namespace rapid_trajectories
