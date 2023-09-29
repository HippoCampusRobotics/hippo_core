#pragma once

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <hippo_control/motor_failure/motor_failure.hpp>
#include <hippo_control/thruster_model.hpp>
#include <hippo_msgs/msg/actuator_controls.hpp>
#include <hippo_msgs/msg/actuator_setpoint.hpp>
#include <hippo_msgs/msg/failure_control_debug.hpp>
#include <hippo_msgs/msg/failure_control_mode_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace hippo_control {
namespace motor_failure {

class ControlNode : public rclcpp::Node {
 public:
  explicit ControlNode(const rclcpp::NodeOptions &_options);

 private:
  struct DOFs {
    double surge;
    double roll;
    double pitch;
    double yaw;
  };
  struct Model {
    struct {
      DOFs linear;
    } damping;
    DOFs inertia;
  };
  struct Gains {
    DOFs p;
  };
  struct Params {
    Gains gains;
    Model model;
    int motor_failures;
    std::vector<int64_t> phase_duration_ms;
    std::vector<int64_t> phase_order;
  };
  void DeclareParams();
  void InitPublishers();
  void InitSubscriptions();
  void InitServices();
  void StartUntangling();
  void StartMission();
  void UpdateMission();
  void CancelMission();
  void PublishThrusterCommand(const rclcpp::Time &now, Eigen::Vector4d &cmds);
  void SetControllerGains();
  void SetControllerModel();
  void PublishDebugMsg(const rclcpp::Time &now,
                       const Eigen::Vector4d &allocated_thrust);
  void PublishMode(const rclcpp::Time &now, mode::Mode mode);
  void SetRollWeightParameter(mode::Mode mode);

  void OnThrustSetpoint(const hippo_msgs::msg::ActuatorSetpoint::SharedPtr);
  void OnAngularVelocitySetpoint(
      const geometry_msgs::msg::Vector3Stamped::SharedPtr);
  void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr);

  rcl_interfaces::msg::SetParametersResult OnParameters(
      const std::vector<rclcpp::Parameter> &parameters);
  void ServeUntangling(const std_srvs::srv::Trigger_Request::SharedPtr _request,
                       std_srvs::srv::Trigger_Response::SharedPtr _response);
  void ServeStartMission(
      const std::shared_ptr<rmw_request_id_t> _header,
      const std_srvs::srv::Trigger_Request::SharedPtr _request);

  //////////////////////////////////////////////////////////////////////////////
  // Publishers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<hippo_msgs::msg::ActuatorControls>::SharedPtr
      actuator_controls_pub_;
  rclcpp::Publisher<hippo_msgs::msg::FailureControlDebug>::SharedPtr debug_pub_;
  rclcpp::Publisher<hippo_msgs::msg::FailureControlModeStamped>::SharedPtr
      mode_pub_;
  //////////////////////////////////////////////////////////////////////////////
  // Subscriptions
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Subscription<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr
      thrust_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      angular_velocity_setpoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr untangling_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mission_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_path_follower_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr
      set_att_ctrl_params_client_;

  OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;

  Params params_;
  Eigen::Vector3d angular_velocity_{0.0, 0.0, 0.0};
  Eigen::Vector3d linear_velocity_{0.0, 0.0, 0.0};
  Eigen::Vector3d angular_velocity_setpoint_{0.0, 0.0, 0.0};
  Eigen::Vector3d thrust_setpoint_{0.0, 0.0, 0.0};
  Eigen::Quaterniond orientation_{1.0, 0.0, 0.0, 0.0};

  rclcpp::TimerBase::SharedPtr mission_timer_;

  MotorFailure controller_;
  hippo_control::ThrusterModel thruster_model_;

  std::size_t phase_index_{0};
  std::size_t duration_index_{0};
  bool mission_running_{false};
};

}  // namespace motor_failure
}  // namespace hippo_control
