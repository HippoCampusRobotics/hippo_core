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
    actuator_controls_pub_ = create_publisher<ActuatorControls>("thruster_command", qos);
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
  }

  void OnActuatorControls(const ActuatorControls::SharedPtr _msg) {
    ActuatorControls out_msg;
    out_msg.control = mixer_.Mix(_msg->control);
    out_msg.header = _msg->header;
    actuator_controls_pub_->publish(out_msg);
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
