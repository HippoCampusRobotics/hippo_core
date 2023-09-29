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

#include <hippo_common/param_utils.hpp>
#include <mjpeg_cam/mjpeg_cam.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mjpeg_cam {
void MjpegCam::InitParams() {
  HIPPO_COMMON_DECLARE_PARAM_READONLY(device_id);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(discrete_size);
}

void MjpegCam::InitCameraParams() {
  if (!camera_) {
    RCLCPP_ERROR(get_logger(), "No camera initialized. Cannot read controls.");
    return;
  }
  auto declare_params = [this](std::vector<Control> controls) {
    for (auto const &control : controls) {
      switch (control.type) {
        case ControlType::kInt: {
          auto descr = hippo_common::param_utils::DescriptionLimit(
              control.name, control.min, control.max, false);
          descr.integer_range.at(0).step = control.step;
          declare_parameter(control.name, control.value, descr);
          break;
        }
        case ControlType::kBool: {
          declare_parameter<bool>(control.name, control.value != 0);
          break;
        }
        case ControlType::kMenu: {
          auto stream = std::ostringstream{};
          for (auto const &entry : control.menu_items) {
            stream << entry.first << " - " << entry.second << "\n";
          }
          std::string text = stream.str();
          auto descr = hippo_common::param_utils::Description(text);
          declare_parameter(control.name, control.value, descr);
          break;
        }
        default:
          RCLCPP_WARN(get_logger(), "Unsupported type of control %s",
                      control.name.c_str());
          continue;
      }
      control_name_to_id_map_[control.name] = control.id;
    }
  };
  declare_params(camera_->Controls());
  control_param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> param) {
        return SetCameraControls(param);
      });
}

rcl_interfaces::msg::SetParametersResult MjpegCam::SetCameraControls(
    const std::vector<rclcpp::Parameter> &parameters) {
  auto is_control = [this](rclcpp::Parameter const &param) {
    return control_name_to_id_map_.find(param.get_name()) !=
           control_name_to_id_map_.end();
  };

  for (const auto &parameter : parameters) {
    if (is_control(parameter)) {
      int id = control_name_to_id_map_[parameter.get_name()];
      switch (parameter.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
          camera_->SetControlValue(id, parameter.as_bool());
          break;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
          camera_->SetControlValue(id, parameter.as_int());
          break;
        default:
          break;
      }
    }
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}
}  // namespace mjpeg_cam
