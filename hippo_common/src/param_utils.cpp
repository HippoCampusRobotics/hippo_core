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

#include "hippo_common/param_utils.hpp"

namespace hippo_common {
namespace param_utils {

bool AssignIfMatch(const rclcpp::Parameter &_param, const std::string &_name,
                   double &_var, std::string &_log_text) {
  if (_param.get_name() == _name) {
    Assign(_param, _var);
    _log_text = "Set [" + _name + "]=" + std::to_string(_var);
    return true;
  }
  return false;
}

bool AssignIfMatch(const rclcpp::Parameter &_param, const std::string &_name,
                   int &_var, std::string &_log_text) {
  if (_param.get_name() == _name) {
    Assign(_param, _var);
    _log_text = "Set [" + _name + "]=" + std::to_string(_var);
    return true;
  }
  return false;
}

bool AssignIfMatch(const rclcpp::Parameter &_param, const std::string &_name,
                   bool &_var, std::string &_log_text) {
  if (_param.get_name() == _name) {
    Assign(_param, _var);
    _log_text = "Set [" + _name + "]=" + std::to_string(_var);
    return true;
  }
  return false;
}

rcl_interfaces::msg::ParameterDescriptor Description(
    const std::string &_description, const bool &read_only) {
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = _description;
  d.read_only = read_only;
  return d;
}

rcl_interfaces::msg::ParameterDescriptor DescriptionLimit(
    const std::string &_description, const double _lower, const double _upper,
    const bool &_read_only) {
  rcl_interfaces::msg::ParameterDescriptor d =
      Description(_description, _read_only);
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = _lower;
  range.to_value = _upper;
  d.floating_point_range.push_back(range);
  return d;
}

rcl_interfaces::msg::ParameterDescriptor DescriptionLimit(
    const std::string &_description, const int _lower, const int _upper,
    const bool &_read_only) {
  rcl_interfaces::msg::ParameterDescriptor d =
      Description(_description, _read_only);
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = _lower;
  range.to_value = _upper;
  d.integer_range.push_back(range);
  return d;
}

}  // namespace param_utils
}  // namespace hippo_common
