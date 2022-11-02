#include <rapid_trajectories/trajectory_generator/generator.h>

#include <hippo_msgs/msg/attitude_target.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rapid_trajectories {
using namespace trajectory_generator;
using namespace hippo_msgs::msg;

class SingleTrackerNode : public rclcpp::Node {
 public:
  SingleTrackerNode() : Node("single_tracker") {
    RCLCPP_INFO(get_logger(), "Node created.");
  }
  void InitPublishers() {
    std::string topic;
    rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

    topic = "body_rates_setpoint";
    body_rates_pub_ = create_publisher<AttitudeTarget>(topic, qos);
  }
  void Run() {}

 private:
  rclcpp::Publisher<AttitudeTarget>::SharedPtr body_rates_pub_;
};

}  // namespace rapid_trajectories

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::shutdown();
  return 0;
}
