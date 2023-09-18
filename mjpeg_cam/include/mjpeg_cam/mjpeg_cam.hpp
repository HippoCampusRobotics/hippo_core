#pragma once
#include <hippo_common/param_utils.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <thread>

#include "mjpeg_cam/device.hpp"

namespace mjpeg_cam {
namespace camera_controls {
static constexpr char kAutoExposure[] = "auto_exposure";
}
class MjpegCam : public rclcpp::Node {
 public:
  explicit MjpegCam(const rclcpp::NodeOptions &_options);
  // virtual ~MjpegCam();

 private:
  struct Params {
    int device_id;
    int test;
    int discrete_size;
    double fps;
  };

  std::string DeviceName() {
    return "/dev/video" + std::to_string(params_.device_id);
  }
  void InitParams();
  void InitCameraParams();
  // void InitPublishers();
  // void InitSubscriptions();
  void LogAvailableFormats();
  void LogAvailableFrameSizes();
  void InitFrameSizes();

  rcl_interfaces::msg::SetParametersResult SetCameraControls(
      const std::vector<rclcpp::Parameter> &parameters);
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;

  std::unordered_map<std::string, int> control_name_to_id_map_;

  std::shared_ptr<Device> camera_;
  std::thread capture_thread_;
  std::vector<std::pair<std::size_t, std::size_t>> frame_sizes_;
  Params params_;
  OnSetParametersCallbackHandle::SharedPtr control_param_cb_;
};
}  // namespace mjpeg_cam
