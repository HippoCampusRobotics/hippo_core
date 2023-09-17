#pragma once
#include <linux/videodev2.h>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <string>
#include <vector>

#include "camera_control.hpp"

struct Buffer {
  void *start;
  size_t length;
  unsigned int index;
};

namespace mjpeg_cam {
class Device {
 public:
  Device(std::string _device, int width, int height);
  ~Device();

  sensor_msgs::msg::CompressedImage::UniquePtr Capture();
  void SetV4L2Param(std::string _name, int _value);
  size_t GetWidth() { return width_; }
  size_t GetHeight() { return height_; }
  std::vector<std::string> AvailableFormats();
  std::vector<std::pair<std::size_t, std::size_t>> AvailableFrameSizes();
  std::vector<Control> Controls() { return controls_; }
  int ControlValue(unsigned int id);
  bool SetControlValue(unsigned int id, int value);

 private:
  void InitMemoryMap();
  void Open();
  void Close();
  void Init();
  void DeInit();
  void StartCapturing();
  void StopCapturing();
  bool ReadFrame();
  void InitControls();
  std::vector<Control> InitUserControls();
  std::vector<Control> InitCameraControls();
  Control GetControl(unsigned int id, bool &status);

  std::string device_;
  int file_descriptor_;
  std::vector<Buffer> buffers_;

  size_t width_;
  size_t height_;
  v4l2_format format_;
  const bool force_format_{true};
  std::vector<Control> controls_;
};
}  // namespace mjpeg_cam
