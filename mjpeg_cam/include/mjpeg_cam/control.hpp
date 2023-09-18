#pragma once
#include <map>
#include <string>

namespace mjpeg_cam {
enum class ControlType { kInt = 1, kBool, kMenu, kUnknown };
struct Control {
  unsigned int id;
  std::string name;
  ControlType type;
  std::map<int, std::string> menu_items;
  int min;
  int max;
  int step;
  bool disabled;
  int default_value;
  int value;
};
}  // namespace mjpeg_cam
