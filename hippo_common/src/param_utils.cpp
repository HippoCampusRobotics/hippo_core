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

}  // namespace param_utils
}  // namespace hippo_common
