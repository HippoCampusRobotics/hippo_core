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

  HIPPO_COMMON_DECLARE_PARAM_READONLY(serial_port);
  HIPPO_COMMON_DECLARE_PARAM_READONLY(apply_pwm_to_thrust_mapping);

  name = "zero_rpm_threshold";
  description = "Threshold below which the input is interpreted as zero";
  descriptor = hippo_common::param_utils::Description(description, true);
  zero_rpm_threshold_ =
      this->declare_parameter(name, zero_rpm_threshold_, descriptor);

  std::vector<std::string> prefixes = {"lower.", "upper."};
  std::vector<std::string> description_prefixes = {"lower", "upper"};
  std::vector<std::string> suffix_coeffs = {"quad", "lin", "const"};
  for (int i = 0; i < int(prefixes.size()); i++) {
    Coefficients *coeffs;
    if (i == 0) {
      coeffs = &mapping_coeffs_.lower;
    } else {
      coeffs = &mapping_coeffs_.upper;
    }

    name = "coeffs_rpm_pwm." + prefixes[i] + "voltage";
    description =
        description_prefixes[i] + "voltage bound for determined coefficients";
    descriptor = hippo_common::param_utils::Description(description, true);
    coeffs->voltage = this->declare_parameter(name, 15.0, descriptor);

    std::array<double, n_coeffs> default_coeffs = {0.0, 500.0, 1500.0};
    for (int j = 0; j < n_coeffs; j++) {
      name = "coeffs_rpm_pwm." + prefixes[i] + "forward." + suffix_coeffs[j];
      description = description_prefixes[i] + suffix_coeffs[j] +
                    " coefficient for forward turning direction";
      descriptor = hippo_common::param_utils::Description(description, true);
      coeffs->forward[j] =
          this->declare_parameter(name, default_coeffs[j], descriptor);

      name = "coeffs_rpm_pwm." + prefixes[i] + "backward." + suffix_coeffs[j];
      description = description_prefixes[i] + suffix_coeffs[j] +
                    " coefficients for forward turning direction";
      descriptor = hippo_common::param_utils::Description(description, true);
      coeffs->backward[j] =
          this->declare_parameter(name, default_coeffs[j], descriptor);
    }
  }
}
}  // namespace teensy
}  // namespace esc
