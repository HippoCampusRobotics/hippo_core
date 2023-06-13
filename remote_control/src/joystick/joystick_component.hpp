#pragma once

#include <stdint.h>

#include <hippo_msgs/msg/actuator_setpoint.hpp>
#include <hippo_msgs/msg/newton_gripper_command.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace remote_control {
namespace joystick {

namespace axes {
static constexpr size_t kLeftStickLeftRight = 0;
static constexpr size_t kLeftStickUpDown = 1;
static constexpr size_t kLT = 2;
static constexpr size_t kRightStickLeftRight = 3;
static constexpr size_t kRightStickUpDown = 4;
static constexpr size_t kRT = 5;
static constexpr size_t kCrossLeftRight = 6;
static constexpr size_t kCrossUpDown = 7;

static constexpr size_t kNumAxes = 8;
};  // namespace axes

namespace buttons {
static constexpr size_t kA = 0;
static constexpr size_t kB = 1;
static constexpr size_t kX = 2;
static constexpr size_t kY = 3;
static constexpr size_t kLB = 4;
static constexpr size_t kRB = 5;

static constexpr size_t kNumButtons = 6;
};  // namespace buttons

class JoyStick : public rclcpp::Node {
 public:
  explicit JoyStick(rclcpp::NodeOptions const &_options);
  struct Params {
    struct Gains {
      struct Torque {
        double x{1.0};
        double y{1.0};
        double z{1.0};
      } torque;
      struct Thrust {
        double x{1.0};
        double y{1.0};
        double z{1.0};
      } thrust;
    } gains;
  };

 private:
  void DeclareParams();
  rcl_interfaces::msg::SetParametersResult OnGainParams(
      const std::vector<rclcpp::Parameter> _parameters);
  void InitPublishers();
  void InitSubscribers();

  std::array<double, 3> ComputeThrust(const std::vector<float> &_axes,
                                      const std::vector<int32_t> &_buttons);
  std::array<double, 3> ComputeTorque(const std::vector<float> &_axes,
                                      const std::vector<int32_t> &_buttons);
  hippo_msgs::msg::NewtonGripperCommand ComputeNewtonGripperCommand(
      const std::vector<float> &_axes, const std::vector<int32_t> &_buttons);

  void PublishThrust(const std::array<double, 3> &_thrust);
  void PublishTorque(const std::array<double, 3> &_torque);
  void PublishNewtonGripperCommand(
      const hippo_msgs::msg::NewtonGripperCommand &_command);

  //////////////////////////////////////////////////////////////////////////////
  // Callbacks
  //////////////////////////////////////////////////////////////////////////////
  void OnJoy(const sensor_msgs::msg::Joy::SharedPtr _msg);

  //////////////////////////////////////////////////////////////////////////////
  // Publishers
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr thrust_pub_;
  rclcpp::Publisher<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr torque_pub_;
  rclcpp::Publisher<hippo_msgs::msg::NewtonGripperCommand>::SharedPtr
      gripper_pub_;
  //////////////////////////////////////////////////////////////////////////////
  // Subscriptions
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  Params params_;
};
}  // namespace joystick
}  // namespace remote_control
