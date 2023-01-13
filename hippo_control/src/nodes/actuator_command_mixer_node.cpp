#include <hippo_common/param_utils.hpp>
#include <hippo_msgs/msg/actuator_controls.hpp>
#include <hippo_msgs/msg/actuator_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hippo_control/mixer/simple_mixer.hpp"

using namespace hippo_control;
using namespace hippo_common;
using namespace hippo_msgs::msg;
using namespace rcl_interfaces;
using std::placeholders::_1;

static constexpr int kUpdatePeriodMs = 5;

class ActuatorCommandMixer : public rclcpp::Node {
 public:
  ActuatorCommandMixer() : Node("actuator_command_mixer") {
    RCLCPP_INFO(get_logger(), "Declaring Paramters");
    DeclareParams();
    auto qos = rclcpp::SystemDefaultsQoS();
    std::string name;

    t_last_thrust_setpoint_ = t_last_torque_setpoint_ = now();

    name = "thruster_command";
    actuator_controls_pub_ = create_publisher<ActuatorControls>(name, qos);

    name = "thrust_setpoint";
    thrust_setpoint_sub_ =
        create_subscription<hippo_msgs::msg::ActuatorSetpoint>(
            name, rclcpp::SensorDataQoS(),
            std::bind(&ActuatorCommandMixer::OnThrustSetpoint, this,
                      std::placeholders::_1));

    name = "torque_setpoint";
    torque_setpoint_sub_ =
        create_subscription<hippo_msgs::msg::ActuatorSetpoint>(
            name, rclcpp::SensorDataQoS(),
            std::bind(&ActuatorCommandMixer::OnTorqueSetpoint, this,
                      std::placeholders::_1));

    update_timer_ = rclcpp::create_timer(
        this, get_clock(), std::chrono::milliseconds(kUpdatePeriodMs),
        std::bind(&ActuatorCommandMixer::Update, this));
    RCLCPP_INFO(get_logger(), "Initialization complete.");
  }
  void DeclareParams() {
    std::string name = "mixer_matrix";
    size_t matrix_size;
    auto mixer_matrix = declare_parameter<std::vector<double>>(
        name, param_utils::Description("Mixer Matrix", true));

    matrix_size = mixer::kOutputChannels * mixer::InputChannels::kCount;
    if (mixer_matrix.size() != matrix_size) {
      throw std::runtime_error("Invalid size of Mixer Matrix. Expected " +
                               std::to_string(matrix_size) + " but got " +
                               std::to_string(mixer_matrix.size()));
    }

    name = "limit_matrix";
    auto limit_matrix = declare_parameter<std::vector<double>>(
        name, param_utils::Description("Limit Matrix", true));
    matrix_size = mixer::kOutputChannels * mixer::InputChannels::kCount;
    if (limit_matrix.size() != matrix_size) {
      throw std::runtime_error("Invalid size of Limit Matrix. Expected " +
                               std::to_string(matrix_size) + " but got " +
                               std::to_string(limit_matrix.size()));
    }

    static constexpr int cols = mixer::InputChannels::kCount;
    static constexpr int rows = mixer::kOutputChannels;
    for (int i = 0; i < rows; ++i) {
      mixer::Mapping mapping;
      for (int j = 0; j < cols; ++j) {
        mapping.scalings[j] = mixer_matrix[i * cols + j];
        mapping.limits[j] = limit_matrix[i * cols + j];
      }
      mixer_.SetMapping(i, mapping);
    }

    std::string descr;
    rcl_interfaces::msg::ParameterDescriptor param;

    name = "zero_thrust_threshold";
    descr = "Thrust threshold until which zero output is sent.";
    param = hippo_common::param_utils::Description(descr);
    auto zero_thrust_threshold = declare_parameter<double>(name, param);
    mixer_.SetZeroThrustThreshold(zero_thrust_threshold);

    name = "constant_coefficient";
    descr = "Constant coefficient c of thrust function F(n) = ax^2 + bx + c.";
    param = hippo_common::param_utils::Description(descr);
    auto constant_coefficient = declare_parameter<double>(name, param);
    mixer_.SetConstantCoefficient(constant_coefficient);

    name = "linear_coefficient";
    descr = "Linear coefficient b of thrust function F(n) = ax^2 + bx + c.";
    param = hippo_common::param_utils::Description(descr);
    auto linear_coefficient = declare_parameter<double>(name, param);
    mixer_.SetLinearCoefficient(linear_coefficient);

    name = "quadratic_coefficient";
    descr = "Quadratic coefficient a of thrust function F(n) = ax^2 + bx + c.";
    param = hippo_common::param_utils::Description(descr);
    auto quadratic_coefficient = declare_parameter<double>(name, param);
    mixer_.SetQuadraticCoefficient(quadratic_coefficient);

    name = "max_rotations_per_second";
    descr =
        "The thrusters maximum rotations per second used for normalization.";
    param = hippo_common::param_utils::Description(descr);
    auto max_rotations_per_second = declare_parameter<double>(name, param);
    mixer_.SetMaxRotationsPerSecond(max_rotations_per_second);
  }

  void Update() {
    auto t_now = now();
    if ((t_now - t_last_thrust_setpoint_).nanoseconds() * 1e-9 > 0.3) {
      ResetThrust();
    }
    if ((t_now - t_last_torque_setpoint_).nanoseconds() * 1e-9 > 0.3) {
      ResetTorque();
    }
    hippo_msgs::msg::ActuatorControls out_msg;
    out_msg.control = mixer_.Mix(inputs_);
    out_msg.header.stamp = now();
    actuator_controls_pub_->publish(out_msg);
  }

  void ResetThrust() {
    inputs_[mixer::InputChannels::kThrustX] = 0.0;
    inputs_[mixer::InputChannels::kThrustY] = 0.0;
    inputs_[mixer::InputChannels::kThrustZ] = 0.0;
  }
  void ResetTorque() {
    inputs_[mixer::InputChannels::kTorqueX] = 0.0;
    inputs_[mixer::InputChannels::kTorqueY] = 0.0;
    inputs_[mixer::InputChannels::kTorqueZ] = 0.0;
  }

  void OnThrustSetpoint(
      const hippo_msgs::msg::ActuatorSetpoint::SharedPtr _msg) {
    if (!_msg->ignore_x) {
      inputs_[mixer::InputChannels::kThrustX] = _msg->x;
    }
    if (!_msg->ignore_y) {
      inputs_[mixer::InputChannels::kThrustY] = _msg->y;
    }
    if (!_msg->ignore_z) {
      inputs_[mixer::InputChannels::kThrustZ] = _msg->z;
    }
    t_last_thrust_setpoint_ = now();
  }

  void OnTorqueSetpoint(
      const hippo_msgs::msg::ActuatorSetpoint::SharedPtr _msg) {
    if (!_msg->ignore_x) {
      inputs_[mixer::InputChannels::kTorqueX] = _msg->x;
    }
    if (!_msg->ignore_y) {
      inputs_[mixer::InputChannels::kTorqueY] = _msg->y;
    }
    if (!_msg->ignore_z) {
      inputs_[mixer::InputChannels::kTorqueZ] = _msg->z;
    }
    t_last_torque_setpoint_ = now();
  }

  rcl_interfaces::msg::SetParametersResult OnThrustParams(
      const std::vector<rclcpp::Parameter> &_parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Undhandled";
    for (const rclcpp::Parameter &parameter : _parameters) {
      double tmp_double;
      if (param_utils::AssignIfMatch(parameter, "linear_coefficient",
                                     tmp_double)) {
        mixer_.SetLinearCoefficient(tmp_double);
        result.reason = "Set linear_coefficient.";
        continue;
      }

      if (param_utils::AssignIfMatch(parameter, "quadratic_coefficient",
                                     tmp_double)) {
        mixer_.SetQuadraticCoefficient(tmp_double);
        result.reason = "Set quadratic_coefficient.";
        continue;
      }

      if (param_utils::AssignIfMatch(parameter, "constant_coefficient",
                                     tmp_double)) {
        mixer_.SetConstantCoefficient(tmp_double);
        result.reason = "Set constant_coefficient.";
        continue;
      }

      if (param_utils::AssignIfMatch(parameter, "zero_thrust_threshold",
                                     tmp_double)) {
        mixer_.SetZeroThrustThreshold(tmp_double);
        result.reason = "Set zero_thrust_threshold";
        continue;
      }
    }
    return result;
  }

 private:
  mixer::SimpleMixer mixer_;
  rclcpp::Subscription<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr
      torque_setpoint_sub_;
  rclcpp::Subscription<hippo_msgs::msg::ActuatorSetpoint>::SharedPtr
      thrust_setpoint_sub_;
  rclcpp::Publisher<ActuatorControls>::SharedPtr actuator_controls_pub_;

  rclcpp::TimerBase::SharedPtr update_timer_;

  std::array<double, mixer::InputChannels::kCount> inputs_;
  rclcpp::Time t_last_thrust_setpoint_;
  rclcpp::Time t_last_torque_setpoint_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActuatorCommandMixer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
