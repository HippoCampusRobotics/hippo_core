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
#include <rclcpp/rclcpp.hpp>

// assumes parameters are stored in this.params_ and names are identical.
#define HIPPO_COMMON_DECLARE_PARAM_DESCR(x, y) \
  do {                                         \
    auto &param = params_.x;                   \
    param = declare_parameter(#x, param, y);   \
  } while (false)

#define HIPPO_COMMON_DECLARE_PARAM(x)     \
  do {                                    \
    auto &param = params_.x;              \
    param = declare_parameter(#x, param); \
  } while (false)

// will throw an exception if no runtime override by the user is provided
#define HIPPO_COMMON_DECLARE_PARAM_NO_DEFAULT(x)            \
  do {                                                      \
    params_.x = declare_parameter<decltype(params_.x)>(#x); \
  } while (false)

#define HIPPO_COMMON_DECLARE_PARAM_READONLY(x)                      \
  do {                                                              \
    auto &param = params_.x;                                        \
    rcl_interfaces::msg::ParameterDescriptor descriptor;            \
    descriptor.read_only = true;                                    \
    param = declare_parameter<decltype(params_.x)>(#x, descriptor); \
  } while (false)

// assumes rcl_interfaces::msg::PSetParametersResult result does exist.
#define HIPPO_COMMON_ASSIGN_SIMPLE_LOG(name, updated, text)                    \
  if (hippo_common::param_utils::AssignIfMatch(parameter, #name, params_.name, \
                                               text)) {                        \
    RCLCPP_INFO_STREAM(get_logger(), text);                                    \
    result.reason = text;                                                      \
    updated = true;                                                            \
    continue;                                                                  \
  }

namespace hippo_common {
namespace param_utils {

// inspired by
// https://github.com/christianrauch/apriltag_ros/blob/master/src/AprilTagNode.cpp
template <typename T>
void Assign(const rclcpp::Parameter &_param, T &_var) {
  _var = _param.get_value<T>();
}

/**
 * @brief Assigns parameter value to the variable \p _var if name of the
 * parameter and \p _name are equal.
 *
 * @tparam T
 * @param _param Input parameter.
 * @param _name Name of the parameter associated with \a _var.
 * @param _var[out] Reference to the variable the parameter value is written to.
 * @retval true if names match.
 * @retval false if names do not match.
 */
template <typename T>
bool AssignIfMatch(const rclcpp::Parameter &_param, const std::string &_name,
                   T &_var) {
  if (_param.get_name() == _name) {
    Assign<T>(_param, _var);
    return true;
  }
  return false;
}

bool AssignIfMatch(const rclcpp::Parameter &_param, const std::string &_name,
                   double &_var, std::string &_log_text);

bool AssignIfMatch(const rclcpp::Parameter &_param, const std::string &_name,
                   int &_var, std::string &_log_text);

bool AssignIfMatch(const rclcpp::Parameter &_param, const std::string &_name,
                   bool &_var, std::string &_log_text);

/**
 * @brief Simplifying the construction a parameter descriptor.
 *
 * @param _description Description text.
 * @param read_only Declares the paramter as read only.
 * @return rcl_interfaces::msg::ParameterDescriptor
 */
rcl_interfaces::msg::ParameterDescriptor Description(
    const std::string &_description, const bool &_read_only = false);

rcl_interfaces::msg::ParameterDescriptor DescriptionLimit(
    const std::string &_description, const double _lower, const double _upper,
    const bool &_read_only = false);

rcl_interfaces::msg::ParameterDescriptor DescriptionLimit(
    const std::string &_description, const int _lower, const int _upper,
    const bool &_read_only = false);

}  // namespace param_utils
}  // namespace hippo_common
