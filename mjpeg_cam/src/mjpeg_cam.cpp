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

#include "mjpeg_cam/mjpeg_cam.hpp"

#include <hippo_common/tf2_utils.hpp>

namespace mjpeg_cam {
MjpegCam::MjpegCam(const rclcpp::NodeOptions &_options)
    : rclcpp::Node("mjpeg_cam", _options) {
  if (_options.use_intra_process_comms() || true) {
    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);
    image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "~/image_raw/compressed", qos);
    info_pub_ =
        create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", 10);
  }
  InitParams();
  InitFrameSizes();
  camera_info_manager_ =
      std::make_shared<camera_info_manager::CameraInfoManager>(this,
                                                               get_name());
  auto frame_size = frame_sizes_.at(std::clamp<std::size_t>(
      params_.discrete_size, 0, frame_sizes_.size() - 1));
  camera_ = std::make_shared<Device>(DeviceName(), frame_size.first,
                                     frame_size.second, params_.fps);
  RCLCPP_INFO(get_logger(), "Opened %s with size [%lu, %lu]",
              DeviceName().c_str(), camera_->GetWidth(), camera_->GetHeight());
  LogAvailableFrameSizes();
  InitCameraParams();

  capture_thread_ = std::thread{[this]() -> void {
    static int frame_counter = 0;
    while (rclcpp::ok()) {
      auto img = camera_->Capture();
      if (img == nullptr) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      frame_counter++;
      if (frame_counter < params_.publish_nth_frame) {
        continue;
      }
      frame_counter = 0;
      img->header.stamp = now();
      auto camera_info = camera_info_manager_->getCameraInfo();
      if (!CameraInfoIsOkay(camera_info)) {
        camera_info = sensor_msgs::msg::CameraInfo{};

        camera_info.width = camera_->GetWidth();
        camera_info.height = camera_->GetHeight();
      }
      camera_info.header.stamp = img->header.stamp;

      if (camera_info.header.frame_id.empty()) {
        camera_info.header.frame_id = "uncalibrated_camera";
      }
      img->header.frame_id = hippo_common::tf2_utils::frame_id::Prefix(this) +
                             '/' + camera_info.header.frame_id;

      info_pub_->publish(camera_info);
      image_pub_->publish(std::move(img));
    }
  }};
}

void MjpegCam::InitFrameSizes() {
  auto camera = std::make_shared<Device>(DeviceName(), 0, 0, params_.fps);
  frame_sizes_ = camera->AvailableFrameSizes();
  camera.reset();
  if (frame_sizes_.size() <= 0) {
    throw std::runtime_error(
        "No discrete frame sizes available. Does the camera support MJPEG?");
  }
}

void MjpegCam::LogAvailableFormats() {
  std::string available_formats;
  available_formats = "Available formats:\n";
  std::vector<std::string> formats = camera_->AvailableFormats();
  for (auto const &format : formats) {
    available_formats += format + "\n";
  }
  RCLCPP_INFO_STREAM(get_logger(), available_formats);
}

void MjpegCam::LogAvailableFrameSizes() {
  std::string text = "\nFrame sizes:\n";
  auto sizes = camera_->AvailableFrameSizes();
  int i = 0;
  for (auto const &size : sizes) {
    text += std::to_string(i) + ": [" + std::to_string(size.first) + ", " +
            std::to_string(size.second) + "]\n";
    ++i;
  }
  RCLCPP_INFO_STREAM(get_logger(), text);
}

bool MjpegCam::CameraInfoIsOkay(const sensor_msgs::msg::CameraInfo &info) {
  return (camera_->GetWidth() == info.width) &&
         (camera_->GetHeight() == info.height);
}

}  // namespace mjpeg_cam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mjpeg_cam::MjpegCam)
