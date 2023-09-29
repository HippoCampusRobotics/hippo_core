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
