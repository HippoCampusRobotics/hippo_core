#include "hippo_common/param_utils.hpp"

namespace hippo_common {
namespace param_utils {

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
