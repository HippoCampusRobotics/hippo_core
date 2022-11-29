#include <vision/vision_apriltag_simple.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vision::AprilTagSimple>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
