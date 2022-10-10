#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

using nav_msgs::msg::Odometry;
using std::placeholders::_1;

class FakeEstimatorNode : public rclcpp::Node {
 public:
  FakeEstimatorNode() : Node("state_estimator") {
    rclcpp::SystemDefaultsQoS qos;
    publisher_ = create_publisher<Odometry>("odometry", qos);
    subscription_ = create_subscription<Odometry>(
        "ground_truth/odometry", qos,
        std::bind(&FakeEstimatorNode::OnOdometry, this, _1));
  }

  void OnOdometry(const Odometry::SharedPtr _msg) {
    publisher_->publish(*_msg);
  }

 private:
  rclcpp::Publisher<Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<Odometry>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FakeEstimatorNode>();
  rclcpp::spin(node);
  return 0;
}
