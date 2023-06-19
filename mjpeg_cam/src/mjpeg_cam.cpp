#include "mjpeg_cam/mjpeg_cam.hpp"

namespace mjpeg_cam {
MjpegCam::MjpegCam(const rclcpp::NodeOptions &_options)
    : rclcpp::Node("mjpeg_cam", _options) {
  if (_options.use_intra_process_comms() || true) {
    image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "image_raw/compressed", 10);
    info_pub_ =
        create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
  }
  camera_ = std::make_shared<Device>("/dev/video0", 1280, 720);

  capture_thread_ = std::thread{[this]() -> void {
    while (rclcpp::ok()) {
      auto img = camera_->Capture();
      if (img == nullptr) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      img->header.stamp = now();
      image_pub_->publish(std::move(img));
    }
  }};
}

}  // namespace mjpeg_cam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mjpeg_cam::MjpegCam)
