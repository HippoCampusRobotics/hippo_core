#include "hippo_control/mixer/simple_mixer.hpp"

#include <math.h>

namespace hippo_control {
namespace mixer {

SimpleMixer::SimpleMixer(){};

void SimpleMixer::SetMapping(int _index, const Mapping &_mapping) {
  if ((_index >= kChannels)) {
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
    const std::array<double, kChannels> &_actuator_controls) {
  ResetOutputs();
  double scaling = 1.0;
  for (int i_out = 0; i_out < kChannels; ++i_out) {
    for (int i_in = 0; i_in < kChannels; ++i_in) {
      double tmp = _actuator_controls[i_in] * mappings_[i_out].scalings[i_in];
      outputs_[i_out].total += tmp;
      outputs_[i_out].channels[i_in] += tmp;
    }
    double tmp = abs(outputs_[i_out].total);
    // normalize the output by applying the reverse of F(n) = an²+bn, n is
    // revolutions per second and divide by max revs per second to normalize
    // output to [-1;1].
    const double a = 2.6e-4;
    const double b = 4.2e-1;
    tmp = (-b + sqrt(4 * a * tmp + b * b)) / (2 * a) / 50;
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

std::array<double, kChannels> SimpleMixer::Mix(
    const std::array<double, kChannels> &_actuator_controls) {
  double scale = ApplyInput(_actuator_controls);
  if (scale > 1.0) {
    ScaleOutputs(scale);
  }
  std::array<double, kChannels> out;
  for (int i = 0; i < kChannels; ++i) {
    out[i] = outputs_[i].total;
  }
  return out;
}

}  // namespace mixer
}  // namespace hippo_control
