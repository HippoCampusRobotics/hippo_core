#pragma once
#include <rclcpp/rclcpp.hpp>

// inspired by
// https://github.com/christianrauch/apriltag_ros/blob/master/src/AprilTagNode.cpp

namespace hippo_common {
namespace param_utils {

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

/**
 * @brief Simplifying the construction a parameter descriptor.
 *
 * @param _description Description text.
 * @param read_only Declares the paramter as read only.
 * @return rcl_interfaces::msg::ParameterDescriptor
 */
rcl_interfaces::msg::ParameterDescriptor Description(
    const std::string &_description, const bool &_read_only = false);

}  // namespace param_utils
}  // namespace hippo_common
