#include "control_node.hpp"

#include <hippo_common/convert.hpp>

namespace hippo_control {
namespace motor_failure {

ControlNode::ControlNode(rclcpp::NodeOptions const &_options)
    : Node("motor_failure_control", _options) {
  DeclareParams();
  InitPublishers();
  InitSubscriptions();
  InitServices();
  CancelMission();
}

void ControlNode::InitPublishers() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);

  topic = "thruster_command";
  using hippo_control_msgs::msg::ActuatorControls;
  actuator_controls_pub_ = create_publisher<ActuatorControls>(topic, qos);

  topic = "~/mode";
  using hippo_control_msgs::msg::FailureControlModeStamped;
  mode_pub_ = create_publisher<FailureControlModeStamped>(topic, qos);

  topic = "~/debug";
  using hippo_control_msgs::msg::FailureControlDebug;
  debug_pub_ = create_publisher<FailureControlDebug>(topic, qos);
}

void ControlNode::InitSubscriptions() {
  std::string topic;
  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);

  topic = "thrust_setpoint";
  using hippo_control_msgs::msg::ActuatorSetpoint;
  thrust_sub_ = create_subscription<ActuatorSetpoint>(
      topic, qos,
      [this](const ActuatorSetpoint::SharedPtr msg) { OnThrustSetpoint(msg); });

  topic = "angular_velocity_setpoint";
  using geometry_msgs::msg::Vector3Stamped;
  angular_velocity_setpoint_sub_ = create_subscription<Vector3Stamped>(
      topic, qos, [this](const Vector3Stamped::SharedPtr msg) {
        OnAngularVelocitySetpoint(msg);
      });

  topic = "odometry";
  using nav_msgs::msg::Odometry;
  odometry_sub_ = create_subscription<Odometry>(
      topic, qos, [this](const Odometry::SharedPtr msg) { OnOdometry(msg); });
}

void ControlNode::InitServices() {
  std::string name;

  name = "toggle_untangling";
  untangling_service_ = create_service<std_srvs::srv::Trigger>(
      name, [this](const std_srvs::srv::Trigger_Request::SharedPtr request,
                   std_srvs::srv::Trigger_Response::SharedPtr response) {
        ServeUntangling(request, response);
      });

  name = "toggle_mission";
  start_mission_service_ = create_service<std_srvs::srv::Trigger>(
      name, [this](const std::shared_ptr<rmw_request_id_t> header,
                   const std_srvs::srv::Trigger_Request::SharedPtr request) {
        ServeStartMission(header, request);
      });

  name = "path_follower/start";
  start_path_follower_client_ = create_client<std_srvs::srv::Trigger>(name);

  name = "attitude_controller/set_parameters";
  set_att_ctrl_params_client_ =
      create_client<rcl_interfaces::srv::SetParameters>(name);
}

void ControlNode::StartUntangling() {
  CancelMission();
  mission_running_ = true;
  controller_.SetMode(mode::kUntangling);
}

void ControlNode::StartMission() {
  mission_timer_.reset();
  phase_index_ = 0;
  duration_index_ = 0;
  mission_running_ = true;
  mission_timer_ = rclcpp::create_timer(
      this, get_clock(),
      std::chrono::milliseconds(params_.phase_duration_ms.at(phase_index_)),
      [this]() { UpdateMission(); });
  mode::Mode new_mode =
      static_cast<mode::Mode>(params_.phase_order.at(phase_index_));
  controller_.SetMode(new_mode);
  SetRollWeightParameter(new_mode);
}

void ControlNode::UpdateMission() {
  // delete current timer. gets recreated depending on next mission phase
  mission_timer_.reset();
  ++phase_index_;
  ++duration_index_;
  if (phase_index_ >= params_.phase_order.size() ||
      duration_index_ >= params_.phase_duration_ms.size()) {
    // mission completed. No new timer required.
    CancelMission();
    return;
  }
  mode::Mode new_mode =
      static_cast<mode::Mode>(params_.phase_order.at(phase_index_));
  auto new_duration =
      std::chrono::milliseconds(params_.phase_duration_ms.at(duration_index_));
  SetRollWeightParameter(new_mode);
  controller_.SetMode(new_mode);
  mission_timer_ = rclcpp::create_timer(this, get_clock(), new_duration,
                                        [this]() { UpdateMission(); });
}

void ControlNode::CancelMission() {
  phase_index_ = 0;
  duration_index_ = 0;
  mission_timer_.reset();
  controller_.SetMode(mode::kIdle);
  mission_running_ = false;
}

void ControlNode::SetRollWeightParameter(mode::Mode _mode) {
  double roll_weight;
  switch (_mode) {
    case mode::kSingleFailureDetected:
    case mode::kDoubleFailureDetected:
      // we have to give up roll control completely
      roll_weight = 0.0;
      break;
    default:
      roll_weight = 1.0;
      break;
  }
  auto request =
      std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  auto parameter = rcl_interfaces::msg::Parameter();
  parameter.name = "roll_weight";
  parameter.value.type =
      rcl_interfaces::msg::ParameterType::Type::PARAMETER_DOUBLE;
  parameter.value.double_value = roll_weight;
  request->parameters.push_back(parameter);
  rclcpp::Time t_now = now();
  if (!set_att_ctrl_params_client_->wait_for_service(
          std::chrono::milliseconds(500))) {
    RCLCPP_ERROR(get_logger(), "Failed to wait for service: [%s]",
                 set_att_ctrl_params_client_->get_service_name());
    return;
  }
  rclcpp::Duration duration = now() - t_now;
  set_att_ctrl_params_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Service call took %.3lf seconds.",
              duration.nanoseconds() * 1e-9);
}

void ControlNode::SetControllerGains() {
  controller_.SetPGains(params_.gains.p.surge, params_.gains.p.roll,
                        params_.gains.p.pitch, params_.gains.p.yaw);
}

void ControlNode::SetControllerModel() {
  controller_.SetTranslationalInertia(params_.model.inertia.surge);
  controller_.SetTranslationalDampingLinear(params_.model.damping.linear.surge);
  controller_.SetRotationalInertia(Eigen::Vector3d{params_.model.inertia.roll,
                                                   params_.model.inertia.pitch,
                                                   params_.model.inertia.yaw});
  controller_.SetRotationalDampingLinear(Eigen::Vector3d{
      params_.model.damping.linear.roll, params_.model.damping.linear.pitch,
      params_.model.damping.linear.yaw});
}

void ControlNode::PublishDebugMsg(const rclcpp::Time &_now,
                                  const Eigen::Vector4d &_allocated_thrust) {
  hippo_control_msgs::msg::FailureControlDebug msg;
  msg.header.stamp = _now;
  // msg.p_gain_surge = params_.gains.p.surge;
  // msg.surge_inertia = params_.model.inertia.surge;
  // msg.surge_damping_linear = params_.model.damping.linear.surge;

  // msg.p_gain_pitch = params_.gains.p.pitch;
  // msg.pitch_inertia = params_.model.inertia.pitch;
  // msg.pitch_damping_linear = params_.model.damping.linear.pitch;

  // msg.p_gain_yaw = params_.gains.p.yaw;
  // msg.yaw_inertia = params_.model.inertia.yaw;
  // msg.yaw_damping_linear = params_.model.damping.linear.yaw;

  using hippo_common::convert::EigenToRos;
  msg.controllability_scaler = controller_.Controllability();
  EigenToRos(controller_.DesiredTorque(), msg.desired_torque);
  EigenToRos(controller_.DesiredForce(), msg.desired_thrust);
  for (int i = 0; i < 4; ++i) {
    msg.allocated_thrust[i] = _allocated_thrust(i);
  }

  debug_pub_->publish(msg);
}

void ControlNode::PublishMode(const rclcpp::Time &_now, mode::Mode _mode) {
  hippo_control_msgs::msg::FailureControlModeStamped msg;
  msg.header.stamp = _now;
  msg.mode.mode = _mode;
  mode_pub_->publish(msg);
}

void ControlNode::OnAngularVelocitySetpoint(
    const geometry_msgs::msg::Vector3Stamped::SharedPtr _msg) {
  hippo_common::convert::RosToEigen(_msg->vector, angular_velocity_setpoint_);
}

void ControlNode::OnThrustSetpoint(
    const hippo_control_msgs::msg::ActuatorSetpoint::SharedPtr _msg) {
  hippo_common::convert::RosToEigen(*_msg, thrust_setpoint_);
}

void ControlNode::OnOdometry(const nav_msgs::msg::Odometry::SharedPtr _msg) {
  using hippo_common::convert::RosToEigen;
  RosToEigen(_msg->twist.twist.angular, angular_velocity_);
  RosToEigen(_msg->twist.twist.linear, linear_velocity_);
  RosToEigen(_msg->pose.pose.orientation, orientation_);
  controller_.SetTarget(angular_velocity_setpoint_, thrust_setpoint_.x());
  Eigen::Vector4d thrusts =
      controller_.Update(angular_velocity_, 0.0, orientation_);
  PublishDebugMsg(_msg->header.stamp, thrusts);
  for (int i = 0; i < 4; ++i) {
    // thrusts(i) = thruster_model_.ThrustToEscCommand(thrusts(i));
    thrusts(i) = std::min(1.0, std::max(-1.0, thrusts(i)));
    // TODO: think about scaling in case we have values outside [-1, 1]
  }
  PublishThrusterCommand(_msg->header.stamp, thrusts);
  PublishMode(_msg->header.stamp, controller_.Mode());
}

void ControlNode::ServeUntangling(
    [[maybe_unused]] const std_srvs::srv::Trigger_Request::SharedPtr _request,
    std_srvs::srv::Trigger_Response::SharedPtr _response) {
  if (mission_running_) {
    CancelMission();
    _response->message = "Canceled untangling.";
  } else {
    StartUntangling();
    _response->message = "Started untangling.";
  }
  _response->success = true;
}
void ControlNode::ServeStartMission(
    const std::shared_ptr<rmw_request_id_t> _header,
    [[maybe_unused]] const std_srvs::srv::Trigger_Request::SharedPtr _request) {
  RCLCPP_INFO(get_logger(), "Serving Toggle Mission Request.");
  auto callback =
      [_header, _request,
       this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        auto request_params = future.get();
        std_srvs::srv::Trigger_Response response;
        response.success = request_params->success;
        RCLCPP_INFO(get_logger(), "Received path_follower start response: %d",
                    response.success);
        if (response.success) {
          StartMission();
          response.message = "Mission started: " + request_params->message;
        } else {
          CancelMission();
          response.message = "Mission start failed: " + request_params->message;
        }
        RCLCPP_INFO(get_logger(), "Sending mission toggle response");
        start_mission_service_->send_response(*_header, response);
      };
  if (mission_running_) {
    CancelMission();
    std_srvs::srv::Trigger_Response response;
    response.success = true;
    response.message = "Canceled mission.";
    start_mission_service_->send_response(*_header, response);
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  if (!start_path_follower_client_) {
    std::string text =
        "Required service client to start path follower not created";
    RCLCPP_ERROR_STREAM(get_logger(), text);
    std_srvs::srv::Trigger_Response response;
    response.success = true;
    response.message = text;
    start_mission_service_->send_response(*_header, response);
    return;
  }
  start_path_follower_client_->async_send_request(request, std::move(callback));
  RCLCPP_INFO(get_logger(),
              "Waiting for path follower start response to complete mission "
              "toggle service");
}

void ControlNode::PublishThrusterCommand(const rclcpp::Time &_now,
                                         Eigen::Vector4d &_cmds) {
  hippo_control_msgs::msg::ActuatorControls msg;
  msg.header.stamp = _now;
  for (int i = 0; i < 4; ++i) {
    msg.control[i] = _cmds(i);
  }
  actuator_controls_pub_->publish(msg);
}
}  // namespace motor_failure
}  // namespace hippo_control

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hippo_control::motor_failure::ControlNode)
