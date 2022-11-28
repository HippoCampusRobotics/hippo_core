#include <hippo_common/tf_publisher.hpp>
#include <memory>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto n = std::make_shared<hippo_common::TfPublisher>(rclcpp::NodeOptions{});
  rclcpp::spin(n);
  rclcpp::shutdown();
  n = nullptr;
  return 0;
}
