#pragma once
#include <linux/videodev2.h>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <string>
#include <vector>

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

 private:
  void InitMemoryMap();
  void Open();
  void Close();
  void Init();
  void DeInit();
  void StartCapturing();
  void StopCapturing();
  bool ReadFrame();
  void SetV4L2Param(std::string _name, std::string _value);

  std::string device_;
  int file_descriptor_;
  std::vector<Buffer> buffers_;

  size_t width_;
  size_t height_;
  v4l2_format format_;
  const bool force_format_{true};
};
}  // namespace mjpeg_cam
