#include "actuator_mixer_bluerov_node.hpp"

using namespace hippo_control;
using namespace hippo_common;
using namespace hippo_msgs::msg;
using namespace rcl_interfaces;
using std::placeholders::_1;

static constexpr int kTimeoutMs = 300;

namespace hippo_control {
namespace mixer_bluerov {

ActuatorMixerNode::ActuatorMixerNode(rclcpp::NodeOptions const &_options)
    : Node("actuator_command_mixer", _options) {
  RCLCPP_INFO(get_logger(), "Declaring Paramters");
  DeclareParams();
  auto qos = rclcpp::SystemDefaultsQoS();
  std::string name;

  t_last_thrust_setpoint_ = t_last_torque_setpoint_ = now();

  name = "thruster_command";
  actuator_controls_pub_ = create_publisher<ActuatorControls>(name, qos);

  name = "thrust_setpoint";
  thrust_setpoint_sub_ = create_subscription<hippo_msgs::msg::ActuatorSetpoint>(
      name, rclcpp::SensorDataQoS(),
      std::bind(&ActuatorMixerNode::OnThrustSetpoint, this,
                std::placeholders::_1));

  name = "torque_setpoint";
  torque_setpoint_sub_ = create_subscription<hippo_msgs::msg::ActuatorSetpoint>(
      name, rclcpp::SensorDataQoS(),
      std::bind(&ActuatorMixerNode::OnTorqueSetpoint, this,
                std::placeholders::_1));

  watchdog_timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(kTimeoutMs),
      std::bind(&ActuatorMixerNode::WatchdogTimeout, this));
  RCLCPP_INFO(get_logger(), "Initialization complete.");
  InitializeParamCallbacks();
}

void ActuatorMixerNode::WatchdogTimeout() {
  auto t_now = now();
  static bool timed_out_prev{false};
  bool timed_out{true};
  if ((t_now - t_last_thrust_setpoint_).nanoseconds() * 1e-6 > kTimeoutMs) {
    ResetThrust();
    timed_out = true;
  } else if ((t_now - t_last_torque_setpoint_).nanoseconds() * 1e-6 >
             kTimeoutMs) {
    ResetTorque();
    timed_out = true;
  } else {
    timed_out = false;
  }
  if (timed_out && !timed_out_prev) {
    RCLCPP_WARN_STREAM(get_logger(),
                       "Input messages timed out. Waiting for new messages");
  } else if (!timed_out && timed_out_prev) {
    RCLCPP_INFO(get_logger(),
                "Received new input messages. Not timed out anymore.");
  }

  if (timed_out) {
    PublishActuatorCommand(t_now);
  }
  timed_out_prev = timed_out;
}

void ActuatorMixerNode::PublishActuatorCommand(const rclcpp::Time &_now) {
  hippo_msgs::msg::ActuatorControls msg;
  msg.control = mixer_.Mix(inputs_);
  msg.header.stamp = _now;
  actuator_controls_pub_->publish(msg);
}

void ActuatorMixerNode::ResetThrust() {
  inputs_[mixer_bluerov::InputChannels::kThrustX] = 0.0;
  inputs_[mixer_bluerov::InputChannels::kThrustY] = 0.0;
  inputs_[mixer_bluerov::InputChannels::kThrustZ] = 0.0;
}
void ActuatorMixerNode::ResetTorque() {
  inputs_[mixer_bluerov::InputChannels::kTorqueX] = 0.0;
  inputs_[mixer_bluerov::InputChannels::kTorqueY] = 0.0;
  inputs_[mixer_bluerov::InputChannels::kTorqueZ] = 0.0;
}

void ActuatorMixerNode::OnThrustSetpoint(
    const hippo_msgs::msg::ActuatorSetpoint::SharedPtr _msg) {
  if (!_msg->ignore_x) {
    inputs_[mixer_bluerov::InputChannels::kThrustX] = _msg->x;
  }
  if (!_msg->ignore_y) {
    inputs_[mixer_bluerov::InputChannels::kThrustY] = _msg->y;
  }
  if (!_msg->ignore_z) {
    inputs_[mixer_bluerov::InputChannels::kThrustZ] = _msg->z;
  }
  t_last_thrust_setpoint_ = now();
  PublishActuatorCommand(t_last_thrust_setpoint_);
}

void ActuatorMixerNode::OnTorqueSetpoint(
    const hippo_msgs::msg::ActuatorSetpoint::SharedPtr _msg) {
  if (!_msg->ignore_x) {
    inputs_[mixer_bluerov::InputChannels::kTorqueX] = _msg->x;
  }
  if (!_msg->ignore_y) {
    inputs_[mixer_bluerov::InputChannels::kTorqueY] = _msg->y;
  }
  if (!_msg->ignore_z) {
    inputs_[mixer_bluerov::InputChannels::kTorqueZ] = _msg->z;
  }
  t_last_torque_setpoint_ = now();
  PublishActuatorCommand(t_last_torque_setpoint_);
}

}  // namespace mixer_bluerov
}  // namespace hippo_control

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hippo_control::mixer_bluerov::ActuatorMixerNode>(
      rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
