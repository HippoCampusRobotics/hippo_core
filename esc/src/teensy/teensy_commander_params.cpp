#include <hippo_common/param_utils.hpp>

#include "teensy_commander.hpp"

namespace esc {
namespace teensy {
void TeensyCommander::DeclareParams() {
  std::string name;
  std::string description;
  rcl_interfaces::msg::ParameterDescriptor descriptor;

  name = "serial_port";
  description = "Full filename of the serial port (e.g. /dev/ttyACM0)";
  descriptor = hippo_common::param_utils::Description(description, true);
  {
    auto &param = params_.serial_port;
    param = declare_parameter(name, param, descriptor);
  }
}
}  // namespace teensy
}  // namespace esc
