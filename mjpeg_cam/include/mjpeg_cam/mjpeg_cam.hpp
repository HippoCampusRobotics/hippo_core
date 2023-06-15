#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "mjpeg_cam/device.hpp"
#include <thread>

namespace mjpeg_cam {
class MjpegCam : public rclcpp::Node {
 public:
  explicit MjpegCam(const rclcpp::NodeOptions &_options);
  // virtual ~MjpegCam();

 private:
  // void InitPublishers();
  // void InitSubscriptions();
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;

  std::shared_ptr<Device> camera_;
  std::thread capture_thread_;
};
}  // namespace mjpeg_cam
