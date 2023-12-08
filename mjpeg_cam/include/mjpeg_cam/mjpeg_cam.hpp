// Copyright (C) 2023 Thies Lennart Alff
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
// USA

#pragma once
#include <camera_info_manager/camera_info_manager.hpp>
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
    int discrete_size;
    int fps;
    int publish_nth_frame;
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

  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

  std::unordered_map<std::string, int> control_name_to_id_map_;

  bool CameraInfoIsOkay(const sensor_msgs::msg::CameraInfo &info);

  std::shared_ptr<Device> camera_;
  std::thread capture_thread_;
  std::vector<std::pair<std::size_t, std::size_t>> frame_sizes_;
  Params params_;
  OnSetParametersCallbackHandle::SharedPtr control_param_cb_;
};
}  // namespace mjpeg_cam
