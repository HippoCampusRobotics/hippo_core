#include <hippo_common/param_utils.hpp>
#include <hippo_msgs/msg/actuator_controls.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hippo_control/mixer/simple_mixer.hpp"

using namespace hippo_control;
using namespace hippo_common;
using namespace hippo_msgs::msg;
using namespace rcl_interfaces;
using std::placeholders::_1;

class ActuatorCommandMixer : public rclcpp::Node {
 public:
  ActuatorCommandMixer() : Node("actuator_command_mixer") {
    RCLCPP_INFO(get_logger(), "Declaring Paramters");
    DeclareParams();
    auto qos = rclcpp::SystemDefaultsQoS();
    actuator_controls_pub_ =
        create_publisher<ActuatorControls>("thruster_command", qos);
    actuator_controls_sub_ = create_subscription<ActuatorControls>(
        "actuator_control", qos,
        std::bind(&ActuatorCommandMixer::OnActuatorControls, this, _1));
    RCLCPP_INFO(get_logger(), "Initialization complete.");
  }
  void DeclareParams() {
    std::string name = "mixer_matrix";
    auto mixer_matrix = declare_parameter<std::vector<double>>(
        name, param_utils::Description("Mixer Matrix", true));
    if (mixer_matrix.size() != mixer::kChannels * mixer::kChannels) {
      throw std::runtime_error(
          "Invalid size of Mixer Matrix. Expected " +
          std::to_string(mixer::kChannels * mixer::kChannels) + " but got " +
          std::to_string(mixer_matrix.size()));
    }

    name = "limit_matrix";
    auto limit_matrix = declare_parameter<std::vector<double>>(
        name, param_utils::Description("Limit Matrix", true));
    if (limit_matrix.size() != mixer::kChannels * mixer::kChannels) {
      throw std::runtime_error(
          "Invalid size of Limit Matrix. Expected " +
          std::to_string(mixer::kChannels * mixer::kChannels) + " but got " +
          std::to_string(limit_matrix.size()));
    }
    for (int i = 0; i < mixer::kChannels; ++i) {
      mixer::Mapping mapping;
      for (int j = 0; j < mixer::kChannels; ++j) {
        mapping.scalings[j] = mixer_matrix[i * mixer::kChannels + j];
        mapping.limits[j] = limit_matrix[i * mixer::kChannels + j];
      }
      mixer_.SetMapping(i, mapping);
    }

    std::string descr;
    rcl_interfaces::msg::ParameterDescriptor param;
    name = "linear_coefficient";
    descr = "Linear coefficient b of thrust function F(n) = ax^2 + bx.";
    param = hippo_common::param_utils::Description(descr);
    auto linear_coefficient = declare_parameter<double>(name, param);
    mixer_.SetLinearCoefficient(linear_coefficient);

    name = "quadratic_coefficient";
    descr = "Quadratic coefficient a of thrust function F(n) = ax^2 + bx.";
    param = hippo_common::param_utils::Description(descr);
    auto quadratic_coefficient = declare_parameter<double>(name, param);
    mixer_.SetQuadraticCoefficient(quadratic_coefficient);
    
    name = "max_rotations_per_second";
    descr = "The thrusters maximum rotations per second used for normalization.";
    param = hippo_common::param_utils::Description(descr);
    auto max_rotations_per_second = declare_parameter<double>(name, param);
    mixer_.SetMaxRotationsPerSecond(max_rotations_per_second);
  }

  void OnActuatorControls(const ActuatorControls::SharedPtr _msg) {
    ActuatorControls out_msg;
    out_msg.control = mixer_.Mix(_msg->control);
    out_msg.header = _msg->header;
    actuator_controls_pub_->publish(out_msg);
  }

  rcl_interfaces::msg::SetParametersResult OnThrustParams(
      const std::vector<rclcpp::Parameter> &_parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Undhandled";
    for (const rclcpp::Parameter &parameter : _parameters) {
      double tmp_double;
      if (param_utils::AssignIfMatch(parameter, "linear_coefficient", tmp_double)) {
        mixer_.SetLinearCoefficient(tmp_double);
        result.reason = "Set linear_coefficient.";
        continue;
      }

      if (param_utils::AssignIfMatch(parameter, "quadratic_coefficient", tmp_double)) {
        mixer_.SetQuadraticCoefficient(tmp_double);
        result.reason = "Set quadratic_coefficient.";
        continue;
      }
    }
    return result;
  }

 private:
  mixer::SimpleMixer mixer_;
  rclcpp::Subscription<ActuatorControls>::SharedPtr actuator_controls_sub_;
  rclcpp::Publisher<ActuatorControls>::SharedPtr actuator_controls_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActuatorCommandMixer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
