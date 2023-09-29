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

#include "mjpeg_cam/device.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>  // close function

#include <cstring>  // reuired for memset
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

namespace mjpeg_cam {

static int xioctl(int _file_handle, unsigned long int _request, void *_arg) {
  int result;
  do {
    result = ioctl(_file_handle, _request, _arg);
  } while (result == -1 && EINTR == errno);
  return result;
}

Device::Device(std::string _device, int _width, int _height)
    : device_(_device), width_(_width), height_(_height) {
  Open();
  Init();
  StartCapturing();
}

Device::~Device() {
  StopCapturing();
  DeInit();
  Close();
}

std::vector<std::string> Device::AvailableFormats() {
  std::vector<std::string> output;
  struct v4l2_fmtdesc fmt_description;
  memset(&fmt_description, 0, sizeof(fmt_description));
  fmt_description.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  while (xioctl(file_descriptor_, VIDIOC_ENUM_FMT, &fmt_description) != -1) {
    output.push_back(
        std::string(reinterpret_cast<char *>(fmt_description.description)));
    ++fmt_description.index;
  }
  return output;
}

std::vector<std::pair<std::size_t, std::size_t>> Device::AvailableFrameSizes() {
  std::vector<std::pair<std::size_t, std::size_t>> output;
  struct v4l2_frmsizeenum frame_size;
  memset(&frame_size, 0, sizeof(frame_size));
  // only list frame sizes for jpeg format
  frame_size.pixel_format = V4L2_PIX_FMT_MJPEG;
  while (xioctl(file_descriptor_, VIDIOC_ENUM_FRAMESIZES, &frame_size) == 0) {
    if (frame_size.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
      output.push_back(std::pair<std::size_t, std::size_t>(
          frame_size.discrete.width, frame_size.discrete.height));
    }
    ++frame_size.index;
  }
  return output;
}

sensor_msgs::msg::CompressedImage::UniquePtr Device::Capture() {
  auto buf = v4l2_buffer{};
  buf.type = V4L2_MEMORY_MMAP;
  if (xioctl(file_descriptor_, VIDIOC_DQBUF, &buf) == -1) {
    return nullptr;
  }
  auto image = std::make_unique<sensor_msgs::msg::CompressedImage>();
  const Buffer &buffer = buffers_[buf.index];
  image->data.resize(buf.bytesused);
  std::copy((uint8_t *)buffer.start,
            (uint8_t *)buffer.start + image->data.size(), image->data.begin());

  if (xioctl(file_descriptor_, VIDIOC_QBUF, &buf) == -1) {
    throw std::runtime_error("VIDIOC_QBUF");
  }

  image->format = "jpeg";
  return image;
}

void Device::Open() {
  file_descriptor_ = open(device_.c_str(), O_RDWR | O_NONBLOCK);
  if (file_descriptor_ < 0) {
    throw std::runtime_error("Cannot open '" + device_ + "' (" +
                             std::to_string(errno) + ")");
  }
}

void Device::Init() {
  struct v4l2_capability capability;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;

  if (xioctl(file_descriptor_, VIDIOC_QUERYCAP, &capability) == -1) {
    throw std::runtime_error("VIDIOC_QUERYCAP errno: " + std::to_string(errno));
  }

  if (!(capability.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    throw std::runtime_error("Missing capability: VIDEO_CAPTURE.");
  }
  if (!(capability.capabilities & V4L2_CAP_STREAMING)) {
    throw std::runtime_error("Missing capability: STREAMING.");
  }

  cropcap = {};
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(file_descriptor_, VIDIOC_CROPCAP, &cropcap) == 0) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect;
    xioctl(file_descriptor_, VIDIOC_S_CROP, &crop);
  }
  format_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (force_format_) {
    format_.fmt.pix.width = width_;
    format_.fmt.pix.height = height_;
    format_.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    // non interlaced video
    format_.fmt.pix.field = V4L2_FIELD_NONE;

    if (xioctl(file_descriptor_, VIDIOC_S_FMT, &format_) == -1) {
      throw std::runtime_error("VIDIOC_S_FMT");
    }
    if (format_.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
      throw std::runtime_error("MJPEG format not supported.");
    }
  } else {
    if (xioctl(file_descriptor_, VIDIOC_G_FMT, &format_) == -1) {
      throw std::runtime_error("VIDIOC_G_FMT");
    }
  }
  // check for actually applied resolution
  width_ = format_.fmt.pix.width;
  height_ = format_.fmt.pix.height;
  InitMemoryMap();
  InitControls();
}

void Device::DeInit() {
  for (auto const &buffer : buffers_) {
    munmap(buffer.start, buffer.length);
  }
  buffers_.clear();
  auto request = v4l2_requestbuffers{};
  request.count = 0;
  request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  request.memory = V4L2_MEMORY_MMAP;
  xioctl(file_descriptor_, VIDIOC_REQBUFS, &request);
}

void Device::StartCapturing() {
  for (const auto &buffer : buffers_) {
    auto buf = v4l2_buffer{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = buffer.index;
    if (xioctl(file_descriptor_, VIDIOC_QBUF, &buf) == -1) {
      throw std::runtime_error("VIDIOC_QBUF");
    }
  }

  v4l2_buf_type type;
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(file_descriptor_, VIDIOC_STREAMON, &type) == -1) {
    throw std::runtime_error("VIDIOC_STREAMON");
  }
}

void Device::StopCapturing() {
  v4l2_buf_type type;
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(file_descriptor_, VIDIOC_STREAMOFF, &type) == -1) {
    throw std::runtime_error("VIDIOC_STREAMOFF");
  }
}

void Device::InitMemoryMap() {
  struct v4l2_requestbuffers request;
  request = {};
  request.count = 4;
  request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  request.memory = V4L2_MEMORY_MMAP;

  if (xioctl(file_descriptor_, VIDIOC_REQBUFS, &request) == -1) {
    throw std::runtime_error("VIDOC_REQBUFS");
  }

  if (request.count < 2) {
    throw std::runtime_error("Not enough buffers available.");
  }

  buffers_ = std::vector<Buffer>(request.count);
  for (auto i = 0U; i < request.count; ++i) {
    auto video_buffer = v4l2_buffer{};
    video_buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    video_buffer.memory = V4L2_MEMORY_MMAP;
    video_buffer.index = i;
    if (xioctl(file_descriptor_, VIDIOC_QUERYBUF, &video_buffer) == -1) {
      throw std::runtime_error("VIDIOC_QUERYBUF");
    }
    buffers_[i].length = video_buffer.length;
    buffers_[i].index = video_buffer.index;
    buffers_[i].start =
        mmap(NULL, video_buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED,
             file_descriptor_, video_buffer.m.offset);
    if (buffers_[i].start == MAP_FAILED) {
      throw std::runtime_error("MMAP faile.");
    }
  }
}

void Device::Close() {
  if (close(file_descriptor_) == -1) {
    throw std::runtime_error("Could not close camera device.");
  }
  // invalidate file descriptor after closing
  file_descriptor_ = -1;
}

std::vector<Control> Device::InitCameraControls() {
  std::vector<Control> controls;
  auto id = V4L2_CID_CAMERA_CLASS_BASE;
  while (true) {
    bool status{false};
    auto control = GetControl(id, status);
    if (!control.disabled && status) {
      controls.push_back(control);
    }
    id++;
    if (V4L2_CTRL_ID2CLASS(id) != V4L2_CTRL_CLASS_CAMERA) {
      break;
    }
  }
  return controls;
}

std::vector<Control> Device::InitUserControls() {
  std::vector<Control> controls;
  auto id = V4L2_CID_USER_BASE;
  while (true) {
    bool status{false};
    auto control = GetControl(id, status);
    if (!control.disabled && status) {
      controls.push_back(control);
    }
    ++id;
    if (V4L2_CTRL_ID2CLASS(id) != V4L2_CTRL_CLASS_USER) {
      break;
    }
  }
  return controls;
}

void Device::InitControls() {
  auto camera_controls = InitCameraControls();
  controls_.insert(controls_.end(), camera_controls.begin(),
                   camera_controls.end());
  auto user_controls = InitUserControls();
  controls_.insert(controls_.end(), user_controls.begin(), user_controls.end());
}

Control Device::GetControl(unsigned int id, bool &status) {
  auto query = v4l2_queryctrl{};
  query.id = id;
  if (xioctl(file_descriptor_, VIDIOC_QUERYCTRL, &query) == -1) {
    status = false;
    return {};
  }
  auto menu_items = std::map<int, std::string>{};
  if (query.type == V4L2_CTRL_TYPE_MENU) {
    auto menu = v4l2_querymenu{};
    menu.id = query.id;
    for (auto i = query.minimum; i <= query.maximum; ++i) {
      menu.index = i;
      if (xioctl(file_descriptor_, VIDIOC_QUERYMENU, &menu) == 0) {
        menu_items[i] = std::string{reinterpret_cast<const char *>(menu.name)};
      }
    }
  }
  auto control = Control{};
  control.id = query.id;
  control.name = std::string{reinterpret_cast<char *>(query.name)};
  control.type = static_cast<ControlType>(query.type);
  control.min = query.minimum;
  control.max = query.maximum;
  control.default_value = query.default_value;
  control.step = query.step;
  control.disabled = (query.flags & V4L2_CTRL_FLAG_DISABLED) != 0;
  control.menu_items = std::move(menu_items);
  control.value = ControlValue(control.id);
  status = true;
  return control;
}

int Device::ControlValue(unsigned int id) {
  auto control = v4l2_control{};
  control.id = id;
  if (xioctl(file_descriptor_, VIDIOC_G_CTRL, &control) == -1) {
    return -1;
  }
  return control.value;
}

bool Device::SetControlValue(unsigned int id, int value) {
  auto control = v4l2_control{};
  control.id = id;
  control.value = value;

  if (xioctl(file_descriptor_, VIDIOC_S_CTRL, &control) == -1) {
    return false;
  }
  return true;
}
}  // namespace mjpeg_cam
