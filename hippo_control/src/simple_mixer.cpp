#include "hippo_control/mixer/simple_mixer.hpp"

#include <math.h>

namespace hippo_control {
namespace mixer {

SimpleMixer::SimpleMixer(){};

void SimpleMixer::SetMapping(int _index, const Mapping &_mapping) {
  if ((_index >= kOutputChannels)) {
    return;
  }
  mappings_[_index] = _mapping;
}

void SimpleMixer::ResetOutputs() {
  for (Output &output : outputs_) {
    output.total = 0.0;
    output.channels.fill(0.0);
  }
}

double SimpleMixer::ApplyInput(
    const std::array<double, InputChannels::kCount> &_actuator_controls) {
  ResetOutputs();
  double scaling = 1.0;
  for (int i_out = 0; i_out < kOutputChannels; ++i_out) {
    for (int i_in = 0; i_in < InputChannels::kCount; ++i_in) {
      double tmp = _actuator_controls[i_in] * mappings_[i_out].scalings[i_in];
      outputs_[i_out].total += tmp;
      outputs_[i_out].channels[i_in] += tmp;
    }
    double tmp = abs(outputs_[i_out].total);
    // normalize the output by applying the reverse of F(n) = an²+bn, n is
    // revolutions per second and divide by max revs per second to normalize
    // output to [-1;1].
    if (linear_coefficient_ == 0.0) {
      if (quadratic_coefficient_ == 0.0) {
        tmp = 0.0;
      } else {
        tmp = sqrt(tmp / quadratic_coefficient_);
      }
    } else {
      if (quadratic_coefficient_ == 0.0) {
        tmp = tmp / linear_coefficient_;
      } else {
        tmp = (-linear_coefficient_ +
               sqrt(4 * quadratic_coefficient_ * tmp +
                    linear_coefficient_ * linear_coefficient_)) /
              (2 * quadratic_coefficient_);
      }
    }
    tmp /= max_rotations_per_second_;
    if (outputs_[i_out].total < 0) {
      outputs_[i_out].total = -1 * tmp;
    } else {
      outputs_[i_out].total = tmp;
    }
    scaling = std::max(tmp, scaling);
  }
  return scaling;
}

void SimpleMixer::ScaleOutputs(double _scale) {
  for (Output &output : outputs_) {
    output.total /= _scale;
  }
}

std::array<double, kOutputChannels> SimpleMixer::Mix(
    const std::array<double, InputChannels::kCount> &_actuator_controls) {
  double scale = ApplyInput(_actuator_controls);
  if (scale > 1.0) {
    ScaleOutputs(scale);
  }
  std::array<double, kOutputChannels> out;
  for (int i = 0; i < kOutputChannels; ++i) {
    out[i] = outputs_[i].total;
  }
  return out;
}

}  // namespace mixer
}  // namespace hippo_control
